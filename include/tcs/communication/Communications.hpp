// Communications.hpp — 基于 SerialProtocol 模板的具体通信类型定义
//
// McuCommunication : 与电控（MCU）通信，CRC8，前导 0x42 0x52 0x02
// ImuCommunication  : 与 IMU 模块通信，CRC32，前导 0xA7 0xB6 0xC5
//
#ifndef TCS_COMMUNICATIONS_HPP
#define TCS_COMMUNICATIONS_HPP

#include "tcs/communication/SerialProtocol.hpp"
#include "tcs/communication/Protocol.hpp"
#include "tcs/communication/CRC.h"
#include "tcs/communication/McuDataPreprocessor.h"
#include "tcs/communication/FusionFilter.h"
#include "tcs/communication/ImuMountCompensation.h"
#include "tcs/communication/McuDiagnostics.h"
#include "tcs/communication/ImuDiagnostics.h"
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <string>
#include <mutex>

namespace tcs {

// ── 端口筛选函数 ──

// 电控（MCU）：选择不是 IMU 的串口
inline bool mcuPortSelector(const std::string& product_info) {
    return product_info != "AutoAim_IMU_Com";
}

// IMU：选择是 IMU 的串口
inline bool imuPortSelector(const std::string& product_info) {
    return product_info == "AutoAim_IMU_Com";
}

// ── 具体通信类型别名 ──

// 电控（MCU）通信：CRC8，前导字节见 mcu::SendPacket / mcu::ReceivePacket
using McuCommunication = SerialProtocol<
    mcu::SendPacket,
    mcu::ReceivePacket,
    CRC8_Check_Sum,
    mcuPortSelector,
    mcu::PREAMBLE_SIZE
>;

// IMU 通信：CRC32，前导字节见 imu::SendPacket / imu::ReceivePacket
using ImuSerialProtocol = SerialProtocol<
    imu::SendPacket,
    imu::ReceivePacket,
    CRC32_Calculate,
    imuPortSelector,
    imu::PREAMBLE_SIZE
>;

class ImuCommunication : public ImuSerialProtocol {
public:
    explicit ImuCommunication(std::function<void(const imu::ReceivePacket&)> callback,
                              bool auto_start = true)
        : ImuSerialProtocol(
              [callback = std::move(callback)](const imu::ReceivePacket& packet) {
                  callback(compensateImuMount(packet));
              }, auto_start) {}
};

// ============================================================================
// RobotCommunication — 组合 MCU 与 IMU 通信，封装数据预处理
// ============================================================================
// - 回调中直接存储预处理后的 MCU 数据（latest_mcu_packet_ 不再存原始数据）
// - McuDataPreprocessor 在接收回调与发送 MCU 数据 (sendToMcu) 时使用
// - 提供统一的最新数据获取接口和分离的发送接口
// ============================================================================
class RobotCommunication {
public:
    struct LatestData {
        bool               imu_valid = false;
        imu::ReceivePacket imu_packet{};
        std::optional<ImuReceiveSample> imu_sample;
        bool               mcu_valid = false;
        mcu::ReceivePacket mcu_packet{};
        std::optional<McuReceiveSample> mcu_sample;
    };

    // 融合权重使用 YawChassisFusion 构造默认值（imu_weight=1.0）
    // mcu_linear_params：MCU 数据线性映射标定参数（默认使用当前标定值）
    explicit RobotCommunication(const McuDataPreprocessor::LinearParams& mcu_linear_params = McuDataPreprocessor::LinearParams{})
        : preprocessor_(mcu_linear_params)
        , mcu_serial_([this](const mcu::ReceivePacket& pkt) { onMcuReceive(pkt); }, false)
        , imu_serial_([this](const imu::ReceivePacket& pkt) { onImuReceive(pkt); }, false)
        , fusion_()
    {
        mcu_serial_.startWorker();
        imu_serial_.startWorker();
    }

    ~RobotCommunication() {
        mcu_serial_.stopWorker();
        imu_serial_.stopWorker();
    }

    // 获取最新数据（MCU 已完成标定映射，IMU 已按安装配置补偿）
    LatestData getLatestData() {
        LatestData data;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (has_imu_data_) {
                data.imu_packet = latest_imu_packet_;
                data.imu_valid  = true;
                data.imu_sample = latest_imu_sample_;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            if (has_mcu_data_) {
                data.mcu_packet = latest_mcu_packet_;   // 已预处理
                data.mcu_valid  = true;
                data.mcu_sample = latest_mcu_sample_;
            }
        }
        return data;
    }

    /// 诊断开关只控制接收队列，样本编号始终随有效报文递增。
    void setImuRecordingEnabled(bool enabled) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (enabled && !imu_recording_enabled_ && latest_imu_sample_ &&
            (imu_samples_.empty() || imu_samples_.back().sample_id != latest_imu_sample_->sample_id))
            appendImuSampleLocked(*latest_imu_sample_);
        imu_recording_enabled_ = enabled;
    }

    ImuReceiveBatch takeImuRecords() {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        ImuReceiveBatch batch;
        batch.samples.swap(imu_samples_);
        batch.dropped_samples = dropped_imu_samples_;
        dropped_imu_samples_ = 0;
        return batch;
    }

    /// MCU 与 IMU 的采集开关分别控制。
    void setMcuRecordingEnabled(bool enabled) {
        std::lock_guard<std::mutex> lock(mcu_mutex_);
        if (enabled && !mcu_recording_enabled_ && latest_mcu_sample_ &&
            (mcu_samples_.empty() || mcu_samples_.back().sample_id != latest_mcu_sample_->sample_id))
            appendMcuSampleLocked(*latest_mcu_sample_);
        mcu_recording_enabled_ = enabled;
    }

    McuReceiveBatch takeMcuRecords() {
        std::lock_guard<std::mutex> lock(mcu_mutex_);
        McuReceiveBatch batch;
        batch.samples.swap(mcu_samples_);
        batch.dropped_samples = dropped_mcu_samples_;
        dropped_mcu_samples_ = 0;
        return batch;
    }

    // 发送 MCU 数据（发送前预处理）
    bool sendToMcu(mcu::SendPacket packet) {
        mcu::SendPacket processed = preprocessor_.processSend(packet);
        return mcu_serial_.sendData(processed);
    }

    // 发送 IMU 数据（心跳等，无预处理）
    bool sendToImu(imu::SendPacket packet) {
        return imu_serial_.sendData(packet);
    }

    void stop() {
        mcu_serial_.stopWorker();
        imu_serial_.stopWorker();
    }

    // 融合输出：高频 yaw 关节解卷绕位置/速度 + 底盘 world 系姿态
    // （IMU 高频积分 + MCU yaw 绝对位置校正，线程安全）
    YawChassisFusion::Output getFused() const {
        return fusion_.output();
    }

    // 重新锚定 imu_yaw_unwrapped 到与 yaw_pos 夹角最近的圈内（线程安全）
    void reanchorImuYaw() {
        fusion_.reanchorImuYaw();
    }

    // 严格反解数据包（独立输出；始终有效，缺失数据以 0 参与，角度 wrap 到 (-π, π]）
    YawChassisFusion::StrictPose getStrictPose() const {
        return fusion_.strictPose();
    }

private:
    void appendImuSampleLocked(const ImuReceiveSample& sample) {
        if (imu_samples_.size() >= kImuSampleCapacity) {
            ++dropped_imu_samples_;
            throw std::overflow_error("IMU 原始诊断接收队列已满");
        }
        imu_samples_.push_back(sample);
    }

    void appendMcuSampleLocked(const McuReceiveSample& sample) {
        if (mcu_samples_.size() >= kMcuSampleCapacity) {
            ++dropped_mcu_samples_;
            throw std::overflow_error("MCU 原始诊断接收队列已满");
        }
        mcu_samples_.push_back(sample);
    }

    // 原始报文与安装补偿后的数据使用相同的接收编号和时间。
    void onImuReceive(const imu::ReceivePacket& packet) {
        const auto received_at = std::chrono::steady_clock::now();
        const auto processed = compensateImuMount(packet);
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (next_imu_sample_id_ == std::numeric_limits<std::uint64_t>::max())
                throw std::overflow_error("IMU 接收样本编号已经耗尽");
            latest_imu_sample_ = ImuReceiveSample{next_imu_sample_id_++, received_at, packet, processed};
            latest_imu_packet_ = processed;
            has_imu_data_      = true;
            if (imu_recording_enabled_) appendImuSampleLocked(*latest_imu_sample_);
        }
        fusion_.onImu(processed.euler_yaw, processed.euler_pitch, processed.euler_roll,
                      processed.gx, processed.gy, processed.gz);
    }

    void onMcuReceive(const mcu::ReceivePacket& packet) {
        const auto received_at = std::chrono::steady_clock::now();
        mcu::ReceivePacket processed = preprocessor_.processReceive(packet);
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            if (next_mcu_sample_id_ == std::numeric_limits<std::uint64_t>::max())
                throw std::overflow_error("MCU 接收样本编号已经耗尽");
            latest_mcu_sample_ = McuReceiveSample{next_mcu_sample_id_++, received_at, packet};
            latest_mcu_packet_ = processed;
            has_mcu_data_      = true;
            if (mcu_recording_enabled_) appendMcuSampleLocked(*latest_mcu_sample_);
        }
        // 喂融合滤波器的数据同样为预处理后的（与 latest_mcu_packet_ 语义一致）
        fusion_.onMcu(processed.yaw_angle, processed.yaw_omega, processed.pitch_angle,
                      processed.chassis_imu_yaw, processed.chassis_imu_omega);
    }

    // ── 成员变量 ──
    McuDataPreprocessor  preprocessor_;   // MCU 数据线性标定预处理
    McuCommunication mcu_serial_;
    ImuSerialProtocol imu_serial_;

    std::mutex         imu_mutex_;
    imu::ReceivePacket latest_imu_packet_{};
    bool               has_imu_data_ = false;
    static constexpr std::size_t kImuSampleCapacity = 4096;
    std::optional<ImuReceiveSample> latest_imu_sample_;
    std::uint64_t next_imu_sample_id_ = 0;
    bool imu_recording_enabled_ = false;
    std::deque<ImuReceiveSample> imu_samples_;
    std::uint64_t dropped_imu_samples_ = 0;

    std::mutex         mcu_mutex_;
    mcu::ReceivePacket latest_mcu_packet_{};
    bool               has_mcu_data_ = false;
    static constexpr std::size_t kMcuSampleCapacity = 4096;
    std::optional<McuReceiveSample> latest_mcu_sample_;
    std::uint64_t next_mcu_sample_id_ = 0;
    bool mcu_recording_enabled_ = false;
    std::deque<McuReceiveSample> mcu_samples_;
    std::uint64_t dropped_mcu_samples_ = 0;

    YawChassisFusion   fusion_;   // IMU 高频 + MCU 低频融合滤波器
};


} // namespace tcs
#endif // TCS_COMMUNICATIONS_HPP
