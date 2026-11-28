#pragma once

#include "tcs/communication/Protocol.hpp"

#include <chrono>
#include <cstdint>
#include <deque>

namespace tcs {

/// 同一个有效 IMU 接收包的原始数据与安装补偿数据。
struct ImuReceiveSample {
    std::uint64_t sample_id = 0;
    std::chrono::steady_clock::time_point received_at;
    imu::ReceivePacket packet{};
    imu::ReceivePacket compensated_packet{};
};

/// 接收队列一次取出的 IMU 诊断记录。
struct ImuReceiveBatch {
    std::deque<ImuReceiveSample> samples;
    std::uint64_t dropped_samples = 0;
};

} // namespace tcs
