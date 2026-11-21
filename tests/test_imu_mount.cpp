#include "tcs/communication/ImuMountCompensation.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {

constexpr double pi = 3.14159265358979323846;

void check(bool condition) {
    if (!condition) throw std::runtime_error("IMU 安装补偿验证失败");
}

Eigen::Matrix3d rotation(const tcs::imu::ReceivePacket& packet) {
    return (Eigen::AngleAxisd(packet.euler_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(packet.euler_pitch, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(packet.euler_roll, Eigen::Vector3d::UnitY())).toRotationMatrix();
}

} // namespace

int main() {
    const Eigen::Matrix3d mount =
        Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitY()).toRotationMatrix();
    for (double yaw : {-pi, -0.7, 0.0, 1.2, pi}) {
        for (double pitch : {-pi / 2, -0.3, 0.0, 0.6, pi / 2}) {
            for (double roll : {-4 * pi, -pi, -0.4, 0.0, 0.9, pi, 4 * pi}) {
                tcs::imu::ReceivePacket packet{};
                packet.data_size = 52;
                packet.gx = 1.25f; packet.gy = -2.5f; packet.gz = 3.75f;
                packet.ax = -4.5f; packet.ay = 5.25f; packet.az = -9.8f;
                packet.euler_yaw = yaw;
                packet.euler_pitch = pitch;
                packet.euler_roll = roll;
                packet.dt_one_tenth_ms = 10;
                packet.crc32 = 0x12345678;
                const auto result = tcs::compensateImuMount(packet);
#if IS_INFANTRY_2
                check(rotation(result).isApprox(rotation(packet) * mount, 1e-12));
                check(result.gx == -packet.gx && result.gz == -packet.gz);
                check(result.ax == -packet.ax && result.az == -packet.az);
                check(result.euler_roll > -pi && result.euler_roll <= pi);
                const auto restored = tcs::compensateImuMount(result);
                check(rotation(restored).isApprox(rotation(packet), 1e-12));
                check(restored.gx == packet.gx && restored.gz == packet.gz);
#else
                check(result.gx == packet.gx && result.gz == packet.gz);
                check(result.ax == packet.ax && result.az == packet.az);
                check(result.euler_roll == packet.euler_roll);
#endif
                check(result.gy == packet.gy && result.ay == packet.ay);
                check(result.euler_yaw == yaw && result.euler_pitch == pitch);
                check(result.frame_header1 == packet.frame_header1 &&
                      result.frame_header2 == packet.frame_header2 &&
                      result.frame_header3 == packet.frame_header3 &&
                      result.data_size == packet.data_size &&
                      result.dt_one_tenth_ms == packet.dt_one_tenth_ms &&
                      result.crc32 == packet.crc32);
            }
        }
    }
    std::printf("IMU 安装补偿验证通过，启用状态：%d\n", IS_INFANTRY_2);
}
