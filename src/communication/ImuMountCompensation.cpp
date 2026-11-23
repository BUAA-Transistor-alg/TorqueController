#include "tcs/communication/ImuMountCompensation.h"

#include <cmath>

namespace tcs {

imu::ReceivePacket compensateImuMount(const imu::ReceivePacket& packet) {
    auto result = packet;
#if IS_INFANTRY_2
    result.gx = -packet.gx;
    result.gz = -packet.gz;
    result.ax = -packet.ax;
    result.az = -packet.az;
    // ZXY 姿态右乘 Ry(π)，仅改变最后的 roll，归一化到 (-π, π]。
    constexpr double pi = 3.14159265358979323846;
    result.euler_roll = std::remainder(packet.euler_roll + pi, 2.0 * pi);
    if (result.euler_roll <= -pi) result.euler_roll = pi;
#endif
    return result;
}

} // namespace tcs
