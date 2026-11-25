#include "tcs/communication/Communications.hpp"

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <ft2build.h>
#include FT_FREETYPE_H

#include <array>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
namespace imu = tcs::imu;
namespace mcu = tcs::mcu;
using tcs::ImuCommunication;
using tcs::McuCommunication;
using tcs::McuDataPreprocessor;
using tcs::YawChassisFusion;
constexpr double pi = 3.14159265358979323846;
constexpr double degrees = 180.0 / pi;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t running = 1;

// 示意模型尺寸，单位为米；替换为实车尺寸后可以表达实际部件间距。
constexpr double chassis_bottom_radius = 0.32;
constexpr double chassis_top_radius = 0.23;
constexpr double chassis_height = 0.22;
constexpr double yaw_pivot_height = 0.32;
constexpr double pitch_pivot_height = 0.10;
constexpr double head_width = 0.18;
constexpr double head_length = 0.42;
constexpr double head_height = 0.14;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void onSignal(int) { running = 0; }

struct Snapshot {
    imu::ReceivePacket imu{};
    mcu::ReceivePacket raw_mcu{};
    mcu::ReceivePacket mcu{};
    YawChassisFusion::Output fused{};
    YawChassisFusion::StrictPose strict{};
    Clock::time_point imu_time{}, mcu_time{};
    uint64_t imu_count = 0, mcu_count = 0;
};

class Receiver {
    // 所有回调和快照共用互斥锁，避免显示跨越不同更新的数据。
    std::mutex mutex_;
    Snapshot state_;
    McuDataPreprocessor preprocessor_;
    YawChassisFusion fusion_;
    McuCommunication mcu_;
    ImuCommunication imu_;

public:
    Receiver()
        : mcu_([this](const mcu::ReceivePacket& packet) {
              std::lock_guard<std::mutex> lock(mutex_);
              state_.raw_mcu = packet;
              state_.mcu = preprocessor_.processReceive(packet);
              const auto& p = state_.mcu;
              fusion_.onMcu(p.yaw_angle, p.yaw_omega, p.pitch_angle,
                            p.chassis_imu_yaw, p.chassis_imu_omega);
              state_.mcu_time = Clock::now();
              ++state_.mcu_count;
          }, false),
          imu_([this](const imu::ReceivePacket& packet) {
              std::lock_guard<std::mutex> lock(mutex_);
              state_.imu = packet;
              fusion_.onImu(packet.euler_yaw, packet.euler_pitch, packet.euler_roll,
                            packet.gx, packet.gy, packet.gz);
              state_.imu_time = Clock::now();
              ++state_.imu_count;
          }, false) {
        mcu_.startWorker();
        imu_.startWorker();
    }

    ~Receiver() {
        mcu_.stopWorker();
        imu_.stopWorker();
    }

    Snapshot snapshot() {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = state_;
        result.fused = fusion_.output();
        result.strict = fusion_.strictPose();
        return result;
    }
};

class DemoReceiver {
    Clock::time_point start_ = Clock::now();
    Snapshot state_;
    McuDataPreprocessor preprocessor_;
    YawChassisFusion fusion_;

public:
    Snapshot snapshot() {
        const auto now = Clock::now();
        const double t = std::chrono::duration<double>(now - start_).count();
        const double chassis_yaw = 0.35 * std::sin(0.4 * t);
        const double chassis_rate = 0.14 * std::cos(0.4 * t);
        const double yaw = 0.6 + 0.7 * std::sin(0.8 * t);
        const double yaw_rate = 0.56 * std::cos(0.8 * t);
        const double pitch = 0.25 + 0.18 * std::sin(0.6 * t);
        // 模拟水平底盘与双关节云台，IMU 角速度由该运动链解析计算。
        auto& imu = state_.imu;
        imu.euler_yaw = chassis_yaw + yaw;
        imu.euler_pitch = pitch;
        imu.euler_roll = 0;
        imu.gx = static_cast<float>(0.108 * std::cos(0.6 * t));
        imu.gy = static_cast<float>((chassis_rate + yaw_rate) * std::sin(pitch));
        imu.gz = static_cast<float>((chassis_rate + yaw_rate) * std::cos(pitch));
        imu.dt_one_tenth_ms = 160;
        // 根据编译配置生成传感器坐标系数据，再通过实际安装补偿路径。
        auto sensor = imu;
#if IS_INFANTRY_2
        sensor.gx = -sensor.gx;
        sensor.gz = -sensor.gz;
        sensor.euler_roll -= pi;
#endif
        imu = tcs::compensateImuMount(sensor);
        auto& raw = state_.raw_mcu;
        const auto params = McuDataPreprocessor::defaultParams();
        raw.yaw_angle = yaw;
        raw.yaw_omega = static_cast<float>(yaw_rate);
        raw.pitch_angle = static_cast<float>((pitch - params.recv_pitch_offset) / params.recv_pitch_scale);
        raw.chassis_imu_yaw = static_cast<float>(std::fmod(chassis_yaw + 2 * pi, 2 * pi));
        raw.chassis_imu_omega = static_cast<float>(chassis_rate);
        state_.mcu = preprocessor_.processReceive(raw);
        fusion_.onImu(imu.euler_yaw, imu.euler_pitch, imu.euler_roll, imu.gx, imu.gy, imu.gz);
        const auto& mcu = state_.mcu;
        fusion_.onMcu(mcu.yaw_angle, mcu.yaw_omega, mcu.pitch_angle,
                      mcu.chassis_imu_yaw, mcu.chassis_imu_omega);
        state_.imu_time = state_.mcu_time = now;
        ++state_.imu_count;
        ++state_.mcu_count;
        state_.fused = fusion_.output();
        state_.strict = fusion_.strictPose();
        return state_;
    }
};

class Font {
    struct Glyph { GLuint texture = 0; int width, height, left, top, advance; };
    std::array<Glyph, 128> glyphs_{};

public:
    explicit Font(const char* path) {
        FT_Library library;
        FT_Face face;
        require(FT_Init_FreeType(&library) == 0, "Cannot initialize FreeType");
        require(FT_New_Face(library, path, 0, &face) == 0, "Cannot open font: " + std::string(path));
        require(FT_Set_Pixel_Sizes(face, 0, 17) == 0, "Cannot set font size");
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        for (unsigned int c = 32; c < 127; ++c) {
            require(FT_Load_Char(face, c, FT_LOAD_RENDER) == 0, "Cannot render font glyph");
            const auto g = face->glyph;
            auto& out = glyphs_[c];
            out.width = static_cast<int>(g->bitmap.width);
            out.height = static_cast<int>(g->bitmap.rows);
            out.left = g->bitmap_left;
            out.top = g->bitmap_top;
            out.advance = static_cast<int>(g->advance.x >> 6);
            if (!out.width || !out.height) continue;
            std::vector<unsigned char> pixels(out.width * out.height);
            for (int row = 0; row < out.height; ++row)
                std::copy_n(g->bitmap.buffer + row * g->bitmap.pitch, out.width,
                            pixels.data() + row * out.width);
            glGenTextures(1, &out.texture);
            glBindTexture(GL_TEXTURE_2D, out.texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, out.width, out.height, 0,
                         GL_ALPHA, GL_UNSIGNED_BYTE, pixels.data());
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        }
        FT_Done_Face(face);
        FT_Done_FreeType(library);
    }

    ~Font() {
        for (const auto& g : glyphs_) if (g.texture) glDeleteTextures(1, &g.texture);
    }

    void text(float x, float y, const std::string& value) const {
        glEnable(GL_TEXTURE_2D);
        for (unsigned char c : value) {
            require(c >= 32 && c < 127, "Unsupported display character");
            const auto& g = glyphs_[c];
            if (g.texture) {
                const float left = x + g.left, top = y - g.top;
                glBindTexture(GL_TEXTURE_2D, g.texture);
                glBegin(GL_QUADS);
                glTexCoord2f(0, 0); glVertex2f(left, top);
                glTexCoord2f(1, 0); glVertex2f(left + g.width, top);
                glTexCoord2f(1, 1); glVertex2f(left + g.width, top + g.height);
                glTexCoord2f(0, 1); glVertex2f(left, top + g.height);
                glEnd();
            }
            x += g.advance;
        }
        glDisable(GL_TEXTURE_2D);
    }
};

void rotation(double yaw, double pitch, double roll) {
    // 与 FusionFilter 使用相同的 ZXY 约定，pitch 绕 x 轴。
    glRotated(yaw * degrees, 0, 0, 1);
    glRotated(pitch * degrees, 1, 0, 0);
    glRotated(roll * degrees, 0, 1, 0);
}

void axes(double length) {
    glBegin(GL_LINES);
    glColor3f(1, 0.25f, 0.25f); glVertex3d(0, 0, 0); glVertex3d(length, 0, 0);
    glColor3f(0.25f, 1, 0.4f); glVertex3d(0, 0, 0); glVertex3d(0, length, 0);
    glColor3f(0.3f, 0.5f, 1); glVertex3d(0, 0, 0); glVertex3d(0, 0, length);
    glEnd();
}

void chassis() {
    constexpr int segments = 64;
    for (int i = 0; i < segments; ++i) {
        const double a = 2 * pi * i / segments, b = 2 * pi * (i + 1) / segments;
        const float shade = static_cast<float>(0.75 + 0.20 * std::cos(a - 0.7));
        glColor3f(0.16f * shade, 0.66f * shade, 0.86f * shade);
        glBegin(GL_QUADS);
        glVertex3d(chassis_bottom_radius * std::cos(a), chassis_bottom_radius * std::sin(a), 0);
        glVertex3d(chassis_bottom_radius * std::cos(b), chassis_bottom_radius * std::sin(b), 0);
        glVertex3d(chassis_top_radius * std::cos(b), chassis_top_radius * std::sin(b), chassis_height);
        glVertex3d(chassis_top_radius * std::cos(a), chassis_top_radius * std::sin(a), chassis_height);
        glEnd();
    }
    for (int top = 0; top < 2; ++top) {
        const double radius = top ? chassis_top_radius : chassis_bottom_radius;
        glColor3f(0.15f, 0.49f, 0.64f);
        glBegin(GL_TRIANGLE_FAN);
        glVertex3d(0, 0, top * chassis_height);
        for (int i = 0; i <= segments; ++i) {
            const double a = 2 * pi * i / segments;
            glVertex3d(radius * std::cos(a), radius * std::sin(a), top * chassis_height);
        }
        glEnd();
    }
    glTranslated(0, 0, chassis_height + 0.003);
    axes(0.28);
    glTranslated(0, 0, -chassis_height - 0.003);
}

void head(bool wire) {
    const double x = head_width / 2, y = head_length / 2, z = head_height / 2;
    const std::array<std::array<double, 3>, 8> vertices{{
        {{-x, -y, -z}}, {{x, -y, -z}}, {{x, y, -z}}, {{-x, y, -z}},
        {{-x, -y, z}}, {{x, -y, z}}, {{x, y, z}}, {{-x, y, z}}}};
    constexpr int faces[6][4] = {{0,1,2,3}, {4,5,6,7}, {0,1,5,4},
                               {2,3,7,6}, {1,2,6,5}, {0,3,7,4}};
    if (wire) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3f(0.95f, 0.95f, 0.95f);
    }
    for (int f = 0; f < 6; ++f) {
        // +Y 为云台朝向，前表面使用亮黄色。
        if (!wire) glColor3f(0.95f, f == 3 ? 0.85f : 0.42f + 0.04f * f, 0.15f);
        glBegin(GL_QUADS);
        for (int index : faces[f]) glVertex3dv(vertices[index].data());
        glEnd();
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if (!wire) axes(0.30);
}

double age(Clock::time_point time, uint64_t count) {
    return count ? std::chrono::duration<double>(Clock::now() - time).count() : -1;
}

std::string triple(const std::string& label, double a, double b, double c) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << label << "  "
        << std::setw(8) << a << " " << std::setw(8) << b << " " << std::setw(8) << c;
    return out.str();
}

std::string pair(const std::string& label, double a, double b, int precision = 2) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << label << "  "
        << std::setw(8) << a << " " << std::setw(8) << b;
    return out.str();
}

void draw(const Snapshot& s, const Font& font, int width, int height,
          double azimuth, double elevation, double distance, bool strict, bool demo) {
    const double imu_age = age(s.imu_time, s.imu_count), mcu_age = age(s.mcu_time, s.mcu_count);
    const bool ready = s.imu_count && s.mcu_count && s.fused.valid;
    const bool fresh = ready && imu_age < 0.5 && mcu_age < 0.5;
    const int scene_width = width - 560;
    glViewport(0, 0, scene_width, height);
    glClearColor(0.045f, 0.06f, 0.085f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const double aspect = static_cast<double>(scene_width) / height;
    glFrustum(-0.045 * aspect, 0.045 * aspect, -0.045, 0.045, 0.1, 30);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0, -0.2, -distance);
    glRotated(elevation, 1, 0, 0);
    glRotated(azimuth, 0, 0, 1);
    glColor3f(0.18f, 0.23f, 0.29f);
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; ++i) {
        const double p = i * 0.1;
        glVertex3d(p, -1, -0.01); glVertex3d(p, 1, -0.01);
        glVertex3d(-1, p, -0.01); glVertex3d(1, p, -0.01);
    }
    glEnd();
    axes(0.55);
    if (ready) {
        glPushMatrix();
        if (strict) rotation(s.strict.chassis_yaw, s.strict.chassis_pitch, s.strict.chassis_roll);
        else rotation(s.fused.chassis_yaw, s.fused.chassis_pitch, s.fused.chassis_roll);
        chassis();
        glTranslated(0, 0, yaw_pivot_height);
        glRotated(s.fused.yaw_pos * degrees, 0, 0, 1);
        glTranslated(0, 0, pitch_pivot_height);
        // 保存云台中心位置，在相同位置叠加 IMU 实测朝向。
        GLdouble transform[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, transform);
        glRotated(s.mcu.pitch_angle * degrees, 1, 0, 0);
        head(false);
        glPopMatrix();
        glPushMatrix();
        glLoadIdentity();
        glTranslated(0, -0.2, -distance);
        glRotated(elevation, 1, 0, 0);
        glRotated(azimuth, 0, 0, 1);
        GLdouble view[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, view);
        // 平移已经在眼坐标中，保持视角旋转，仅替换部件旋转。
        view[12] = transform[12]; view[13] = transform[13]; view[14] = transform[14];
        glLoadMatrixd(view);
        rotation(s.imu.euler_yaw, s.imu.euler_pitch, s.imu.euler_roll);
        glScaled(1.025, 1.025, 1.025);
        head(true);
        glPopMatrix();
    }
    glViewport(0, 0, width, height);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    const float panel = static_cast<float>(width - 545);
    glColor4f(0.025f, 0.035f, 0.055f, 0.94f);
    glBegin(GL_QUADS);
    glVertex2f(panel - 12, 0); glVertex2f(width, 0);
    glVertex2f(width, height); glVertex2f(panel - 12, height);
    glEnd();
    float y = 30;
    auto line = [&](const std::string& value) { font.text(panel, y, value); y += 25; };
    glColor3f(0.85f, 0.9f, 0.95f);
    line(demo ? "test_control | DEMO - simulated data" : "test_control | receive only");
    glColor3f(fresh ? 0.3f : 1.0f, fresh ? 0.95f : 0.6f, 0.3f);
    line(!ready ? "WAITING FOR IMU + MCU + FUSION" : demo ? "SIMULATION - serial ports closed" :
         fresh ? "LIVE" : "STALE: last received pose");
    glColor3f(0.85f, 0.9f, 0.95f);
    line(pair("Age(s) IMU / MCU", imu_age, mcu_age));
    line("Age -1: no packet received");
    line("Packets IMU=" + std::to_string(s.imu_count) + " MCU=" + std::to_string(s.mcu_count));
    line("Angles: degrees | rates: rad/s");
    line("                   yaw    pitch     roll");
    line(s.imu_count ? triple("IMU   ", s.imu.euler_yaw * degrees, s.imu.euler_pitch * degrees,
                             s.imu.euler_roll * degrees) : "IMU: waiting");
    line(s.mcu_count ? triple("MCU raw", s.raw_mcu.yaw_angle * degrees, s.raw_mcu.pitch_angle * degrees,
                             s.raw_mcu.chassis_imu_yaw * degrees) : "MCU: waiting");
    line("MCU columns: joint yaw/pitch, chassis yaw");
    line(s.mcu_count ? triple("MCU cal", s.mcu.yaw_angle * degrees, s.mcu.pitch_angle * degrees,
                             s.mcu.chassis_imu_yaw * degrees) : "MCU calibrated: waiting");
    line(s.mcu_count ? pair("Chassis rate raw/cal", s.raw_mcu.chassis_imu_omega,
                           s.mcu.chassis_imu_omega, 4) : "Chassis rate: waiting");
    if (ready) {
        line(triple("Fused  ", s.fused.chassis_yaw * degrees, s.fused.chassis_pitch * degrees,
                    s.fused.chassis_roll * degrees));
        line(triple("Strict ", s.strict.chassis_yaw * degrees, s.strict.chassis_pitch * degrees,
                    s.strict.chassis_roll * degrees));
        line(pair("Joint: yaw / rate", s.fused.yaw_pos * degrees, s.fused.yaw_rate));
    } else {
        line("Fused / strict: waiting");
    }
    y += 15;
    line(strict ? "View: STRICT reconstruction" : "View: FUSED reconstruction");
    line("Cyan: chassis | Orange: gimbal (+Y front)");
    line("White outline: measured IMU orientation");
    line("Axes: X red | Y green | Z blue");
    line("Drag: orbit | Wheel: zoom | R: reset view");
    line("F: fused | S: strict | Esc: exit");
    line("Schematic dimensions; world position fixed");
#if IS_INFANTRY_2
    line("IMU mount: Y 180 deg compensation ON");
#else
    line("IMU mount: standard installation");
#endif
    glDisable(GL_BLEND);
}

void screenshot(const char* path, int width, int height) {
    std::vector<unsigned char> pixels(width * height * 3), flipped(pixels.size());
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    for (int y = 0; y < height; ++y)
        std::copy_n(pixels.data() + (height - y - 1) * width * 3, width * 3,
                    flipped.data() + y * width * 3);
    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormatFrom(
        flipped.data(), width, height, 24, width * 3, SDL_PIXELFORMAT_RGB24);
    require(surface != nullptr, SDL_GetError());
    require(SDL_SaveBMP(surface, path) == 0, SDL_GetError());
    SDL_FreeSurface(surface);
}
} // namespace

int main(int argc, char** argv) {
    int frame_limit = 0;
    bool demo = false;
    const char* capture = nullptr;
    const char* font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            std::cout << "Usage: test_control [--demo] [--font PATH] [--frames N] [--screenshot PATH.bmp]\n"
                         "Receive-only IMU/MCU fusion viewer. Drag: orbit; wheel: zoom;\n"
                         "F: fused; S: strict; R: reset camera; Esc: exit.\n"
                         "Geometry uses schematic metres; translation stays at the origin.\n";
            return 0;
        }
        if (arg == "--demo") { demo = true; continue; }
        require(i + 1 < argc, "Missing argument value: " + arg);
        if (arg == "--frames") {
            std::size_t consumed = 0;
            const std::string value = argv[++i];
            frame_limit = std::stoi(value, &consumed);
            require(frame_limit > 0 && consumed == value.size(), "--frames must be a positive integer");
        } else if (arg == "--font") font_path = argv[++i];
        else if (arg == "--screenshot") capture = argv[++i];
        else throw std::invalid_argument("Unknown argument: " + arg);
    }
    require(!capture || frame_limit, "--screenshot requires --frames");
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    require(SDL_Init(SDL_INIT_VIDEO) == 0, SDL_GetError());
    require(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2) == 0, SDL_GetError());
    require(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1) == 0, SDL_GetError());
    require(SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) == 0, SDL_GetError());
    require(SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24) == 0, SDL_GetError());
    SDL_Window* window = SDL_CreateWindow("test_control - IMU / MCU pose debug",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 800,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    require(window != nullptr, SDL_GetError());
    if (demo) SDL_SetWindowTitle(window, "test_control - DEMO - simulated IMU / MCU");
    SDL_SetWindowMinimumSize(window, 1100, 760);
    SDL_GLContext context = SDL_GL_CreateContext(window);
    require(context != nullptr, SDL_GetError());
    {
        Font font(font_path);
        std::unique_ptr<Receiver> receiver;
        if (!demo) receiver = std::make_unique<Receiver>();
        DemoReceiver demo_receiver;
        double azimuth = -30, elevation = -65, distance = 2.5;
        bool strict = false;
        int frame = 0;
        while (running && (!frame_limit || frame < frame_limit)) {
            const auto start = Clock::now();
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) running = 0;
                if (event.type == SDL_KEYDOWN) {
                    switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE: running = 0; break;
                    case SDLK_f: strict = false; break;
                    case SDLK_s: strict = true; break;
                    case SDLK_r: azimuth = -30; elevation = -65; distance = 2.5; break;
                    default: break;
                    }
                }
                if (event.type == SDL_MOUSEMOTION && (event.motion.state & SDL_BUTTON_LMASK)) {
                    azimuth += event.motion.xrel * 0.4;
                    elevation = std::clamp(elevation + event.motion.yrel * 0.4, -175.0, -5.0);
                }
                if (event.type == SDL_MOUSEWHEEL)
                    distance = std::clamp(distance * std::exp(-event.wheel.y * 0.1), 1.2, 8.0);
            }
            int width, height;
            SDL_GL_GetDrawableSize(window, &width, &height);
            if (!width || !height) { SDL_Delay(16); continue; }
            const auto state = demo ? demo_receiver.snapshot() : receiver->snapshot();
            draw(state, font, width, height, azimuth, elevation, distance, strict, demo);
            require(glGetError() == GL_NO_ERROR, "OpenGL rendering failed");
            ++frame;
            if (capture && frame == frame_limit) screenshot(capture, width, height);
            SDL_GL_SwapWindow(window);
            std::this_thread::sleep_until(start + std::chrono::milliseconds(16));
        }
    }
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
