#pragma once

#include "tcs/communication/Protocol.hpp"

#include <chrono>
#include <cstdint>
#include <deque>

namespace tcs {

/// 一个通过 CRC 检查的 MCU 原始接收包，尚未应用标定映射。
struct McuReceiveSample {
    std::uint64_t sample_id = 0;
    std::chrono::steady_clock::time_point received_at;
    mcu::ReceivePacket packet{};
};

/// 接收回调产生的诊断记录；取出后由调用方负责持久化。
struct McuReceiveBatch {
    std::deque<McuReceiveSample> samples;
    std::uint64_t dropped_samples = 0;
};

} // namespace tcs
