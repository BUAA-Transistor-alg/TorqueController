// control_demo.cpp — 使用 RobotController 封装类的正弦目标演示程序
//
// - 参数与 pygame_control_mpc.py 一致
// - 以 3s 周期正弦控制 target_yaw（±30°）与 pitch_target_angle（-10°~+20°），
//   两者相位差 90°
// - 内部：RobotController 建立通信 + 融合 + yaw MPC + 后台发送线程（100Hz），
//   外部只需 set() 设置目标即可
// - 运行: ./build/control_demo，Ctrl+C 退出
#include "tcs/RobotController.h"

#include <cmath>
#include <cstdio>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>

namespace tcs {

namespace {

std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double PERIOD_S = 3.0;              // 正弦周期（秒）
constexpr double OMEGA = 2.0 * M_PI / PERIOD_S;

class SineCenterWindow {
public:
    struct Estimate {
        double yaw;
        double chassis_rate;
    };

    void clear() { samples_.clear(); }

    std::optional<Estimate> update(double t, double yaw, double chassis_rate) {
        if (!std::isfinite(t) || !std::isfinite(yaw) || !std::isfinite(chassis_rate) ||
            (!samples_.empty() && t <= samples_.back().t)) {
            throw std::invalid_argument("invalid sine center sample");
        }
        samples_.push_back({t, yaw, chassis_rate});
        const double cutoff = t - PERIOD_S;
        while (samples_.size() > 1 && samples_[1].t <= cutoff) {
            samples_.pop_front();
        }
        if (samples_.size() < 2 || samples_.front().t > cutoff) {
            return std::nullopt;
        }

        // 按实际时间积分，并插值截取窗口边界，避免循环周期抖动改变统计权重。
        double yaw_integral = 0.0;
        double rate_integral = 0.0;
        for (size_t i = 1; i < samples_.size(); ++i) {
            const auto& a = samples_[i - 1];
            const auto& b = samples_[i];
            const double begin = std::max(a.t, cutoff);
            const double fraction = (begin - a.t) / (b.t - a.t);
            const double yaw_begin = a.yaw + fraction * (b.yaw - a.yaw);
            const double rate_begin = a.chassis_rate +
                                      fraction * (b.chassis_rate - a.chassis_rate);
            const double dt = b.t - begin;
            yaw_integral += 0.5 * (yaw_begin + b.yaw) * dt;
            rate_integral += 0.5 * (rate_begin + b.chassis_rate) * dt;
        }
        return Estimate{yaw_integral / PERIOD_S, rate_integral / PERIOD_S};
    }

private:
    struct Sample {
        double t;
        double yaw;
        double chassis_rate;
    };
    std::deque<Sample> samples_;
};

// ── 参数（与 pygame_control_mpc.py 一致）──
constexpr double DT_CTRL = 0.01;              // 控制周期 (100 Hz)
constexpr double FEEDBACK_DELAY_S = 0.010;    // 融合反馈滞后估计（秒），0 关闭外推
constexpr int    MPC_PRED_N = 20;             // int(DELAY_TIME / DT_CTRL)，DELAY_TIME=0.2s
constexpr double J      = 0.016541;
constexpr double TAU_C  = 0.097297;
constexpr double B_FRIC = 0.032100;
constexpr double TAU_D  = 0.0;
constexpr double MAX_TORQUE      = 1.0;
constexpr double MAX_TORQUE_RATE = 40.0;
constexpr double Q = 5.0;
constexpr double R = 0.01;
constexpr double Rd = 0.1;
constexpr double VELOCITY_RESIDUAL_SCALE = 1.4; // 角速度残差系数，0 关闭
constexpr int    MAX_ITER = 30;
// 位置跟踪误差平滑绝对值常数 a（代价 = Q*sqrt(err²+a)；很小正数）
constexpr double SMOOTH_EPS = 1e-6;

// ── 正弦目标 ──
// target_yaw: ±30° 正弦
inline double targetYaw(double t) {
    return 30.0 * DEG2RAD * std::sin(OMEGA * t);
}

// pitch_target_angle: 范围 -10°~+20°（中点 5°、幅值 15°），与 yaw 相位差 90°
inline double targetPitch(double t) {
    return (5.0 + 15.0 * std::sin(OMEGA * t - M_PI / 2.0)) * DEG2RAD;
}

} // namespace

} // namespace tcs

// main() 必须留在全局命名空间（否则不是程序入口）；
// 下面把 namespace tcs 内的名字引入作用域，便于 main() 直接使用。
using namespace tcs;

int main() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    printf("=== control_demo (RobotController) ===\n");
    printf("融合反馈恒速外推: %.1f ms\n", FEEDBACK_DELAY_S * 1000.0);
    printf("角速度残差系数: %.3f（代价权重 %.3f）\n",
           VELOCITY_RESIDUAL_SCALE, VELOCITY_RESIDUAL_SCALE * VELOCITY_RESIDUAL_SCALE);
    printf("正弦周期 %.1fs: target_yaw ±30°, pitch_target -10°~+20°, 相位差 90°\n", PERIOD_S);
    printf("yaw_world/sine_center 单位为度，chassis_rate/center_chassis_rate 单位为 rad/s；\n"
           "sine_center 为最近 %.1fs 世界系连续 yaw 的时间平均值，目标中心为 0°；\n"
           "center_chassis_rate 为同窗口平均底盘角速度，center_t 为窗口中点；"
           "窗口完整后 center_valid=1。\n", PERIOD_S);

    // 一体化控制封装：通信 + 融合滤波器 + yaw MPC + 后台发送线程
    // integral_gain: yaw 力矩积分补偿比例系数（默认 0.01）
    // mpc_loop_period: McuMpcController 后台 loop 周期（秒，必须传参；0.01 = 100Hz）
    // mcu_linear_params 使用当前默认标定值（LinearParams{} 与默认构造等价）
    RobotController rc(DT_CTRL, MPC_PRED_N,
                       J, TAU_C, B_FRIC, TAU_D,
                       MAX_TORQUE, MAX_TORQUE_RATE,
                       Q, R, Rd, MAX_ITER,
                       /*integral_gain=*/0.01,
                       /*mpc_loop_period=*/0.01,
                       McuDataPreprocessor::LinearParams{},
                       /*sequence_mode=*/false,
                       /*smooth_eps=*/SMOOTH_EPS,
                       /*feedback_delay_s=*/FEEDBACK_DELAY_S,
                       /*velocity_residual_scale=*/VELOCITY_RESIDUAL_SCALE);

    // 等待融合数据就绪
    printf("等待融合数据就绪...\n");
    while (g_running) {
        if (rc.getState().fused.valid) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!g_running) return 0;
    printf("融合就绪，开始正弦控制（Ctrl+C 退出）\n");

    auto t0 = std::chrono::steady_clock::now();
    int loop = 0;
    SineCenterWindow center_window;
    while (g_running) {
        auto start = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(start - t0).count();

        // 设置发送参数 + mpc 目标（后台线程 100Hz 求解并发送）
        double yaw   = targetYaw(t);
        double pitch = targetPitch(t);
        rc.set(/*auto_aim_enable=*/true, /*yaw_torque_only_mode=*/false,
               yaw, pitch, /*fire=*/false, /*integral_enable=*/false);

        auto st = rc.getState();
        std::optional<SineCenterWindow::Estimate> center;
        if (st.fused.valid && st.mcu.valid) {
            center = center_window.update(t, st.fused.imu_yaw_unwrapped,
                                          st.mcu.chassis_imu_omega);
        } else {
            center_window.clear();
        }

        // 每 0.1s 打印一次
        if (++loop % 10 == 0) {
            const double unavailable = std::numeric_limits<double>::quiet_NaN();
            printf("[t=%6.2fs] target_yaw=%+7.2f° pitch=%+6.2f° | "
                   "yaw_pos=%+.3f yaw_rate=%+.3f torque=%+.4f | "
                   "yaw_world=%+.2f° chassis_rate=%+.3f mcu=%d | "
                   "sine_center=%+.3f° center_chassis_rate=%+.3f "
                   "center_t=%.2fs center_valid=%d | fused=%d | temperature=%d\n",
                   t, yaw / DEG2RAD, pitch / DEG2RAD,
                   st.fused.yaw_pos, st.fused.yaw_rate, st.mpc.yaw_torque,
                   std::remainder(st.fused.imu_yaw_unwrapped, 2.0 * M_PI) / DEG2RAD,
                   st.mcu.chassis_imu_omega, (int)st.mcu.valid,
                   center ? std::remainder(center->yaw, 2.0 * M_PI) / DEG2RAD : unavailable,
                   center ? center->chassis_rate : unavailable,
                   center ? t - PERIOD_S / 2.0 : unavailable, (int)center.has_value(),
                   (int)st.fused.valid, st.mcu.yaw_temperature);
        }

        // 循环结束处等待到 start + 10ms（100Hz），不累计误差
        std::this_thread::sleep_until(start + std::chrono::milliseconds(10));
    }

    printf("\n退出。\n");
    return 0;
}
