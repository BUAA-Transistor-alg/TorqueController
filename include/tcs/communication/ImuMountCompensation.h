#pragma once

#include "tcs/communication/Protocol.hpp"

namespace tcs {

// 输入为通过 CRC 校验的报文，输出为原安装坐标系的数据；crc32 保留接收值。
imu::ReceivePacket compensateImuMount(const imu::ReceivePacket& packet);

} // namespace tcs
