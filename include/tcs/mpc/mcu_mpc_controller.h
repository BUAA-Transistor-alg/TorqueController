#ifndef TCS_MCU_MPC_CONTROLLER_H
#define TCS_MCU_MPC_CONTROLLER_H

#include <vector>
#include <thread>
#include <atomic>
#include <cstdint>
#include <optional>
#include <mutex>
#include <deque>
#include <chrono>
#include <cstddef>
#include "tcs/communication/Communications.hpp"
#include "tcs/mpc/yaw_mpc_controller.h"
#include "tcs/common/FrameRateCounter.h"

namespace tcs {

/// 一个实际控制周期使用的参考关联信息。
struct ControlStamp {
    /// 原始图像编号；调用 set() 时未提供则为空。
    std::optional<std::uint64_t> frame_id;
    /// 预测计划编号；调用 set() 时未提供则为空。
    std::optional<std::uint64_t> plan_id;
    /// 从 0 开始持续递增的控制周期编号；首次完成控制周期前为空。
    std::optional<std::uint64_t> control_tick_id;
    /// 本周期消费的 yaw 序列下标；耗尽后保留末值，单目标或未消费序列时为空。
    std::optional<std::uint32_t> reference_index;
};

/// 一个实际后台控制周期的调试结果；求解器未提供的数据保持缺省。
struct ControlRecord {
    /// 与本周期实际参考共同取得的关联信息。
    ControlStamp stamp;
    /// 本周期开始时刻，采用 steady_clock。
    std::chrono::steady_clock::time_point started_at;
    /// 本周期组装好报文、准备调用通信接口的时刻。
    std::chrono::steady_clock::time_point command_submitted_at;
    /// 通信调用结束时刻；没有通信设备时为跳过发送后的时刻。
    std::chrono::steady_clock::time_point finished_at;
    /// 本周期组装进报文的 yaw 预测位置，单位为弧度。
    double yaw_target_angle = 0.0;
    /// 本周期组装进报文的 yaw 预测速度，单位为弧度每秒。
    double yaw_target_velocity = 0.0;
    /// 本周期组装进报文的最终 yaw 力矩，单位为 N·m。
    double yaw_torque = 0.0;
    /// 本周期组装进报文的 pitch 参考，单位为弧度。
    double pitch_target_angle = 0.0;
    /// 本周期结束时的 yaw 积分补偿力矩，单位为 N·m。
    double integral = 0.0;
    /// 本周期实际采用的积分补偿开关。
    bool integral_enabled = false;
    /// 本周期报文中的自瞄开关。
    bool auto_aim_enable = false;
    /// 本周期报文中的 yaw 力矩模式开关。
    bool yaw_torque_only_mode = false;
    /// 本周期报文中的开火位。
    bool fire = false;
    /// 本次 MPC 使用的首个转换后参考；求解未提供参考时为空。
    std::optional<double> reference_position;
    /// 通信接口返回的提交结果；没有通信设备时为空，不表示下位机执行确认。
    std::optional<bool> command_send_succeeded;
};

/// 一次批量取出的控制调试记录，记录按实际周期顺序排列。
struct ControlRecordBatch {
    /// 自上次取出后仍被保留的记录，所有权转交调用方。
    std::deque<ControlRecord> records;
    /// 自上次取出后因队列容量限制而移除的最早记录数量。
    std::uint64_t dropped_records = 0;
};

// ============================================================================
// McuMpcController — 实车 MCU 控制封装（yaw MPC + 发送参数维护 + 后台发送线程）
//
// - set(auto_aim_enable, yaw_torque_only_mode, target_yaw, pitch_target_angle,
//      fire): 一次性设置全部发送参数与 mpc 目标。除 target_yaw 传给
//      YawMpcController 外，其余参数由本类自己维护（后台线程读取使用）。
//      target_yaw 在设置时自动转换到与 imu_yaw_unwrapped 角度差最小的等效角
//      （|差| ≤ π，且与 target_yaw 同向：target_adj ≡ target_yaw (mod 2π)）。
// - 后台线程以构造传入的 loop_period（秒，默认 0.01 = 100Hz）为周期运行：
//   从本类取最新 target_yaw 调用 YawMpcController::step，
//   以 mpc 结果配合最新设置的发送参数构造 McuSendPacket 发送给 MCU。
// - 线程循环：开始处取 steady_clock::now()，结束处
//   sleep_until(start + loop_period)，不严格跟随绝对时间点，避免误差累计。
// - 循环真实帧率由 FrameRateCounter 记录（滑动窗口 60 帧），最新值随
//   State::loop_fps 返回。
// - 设置参数与最新结果（last_state_）使用独立的锁保护。
// ============================================================================
class McuMpcController {
public:
    // 最新 mpc 求解结果（供显示/日志）
    struct State {
        /// 与本次缓存结果共同产生的控制周期关联信息。
        ControlStamp stamp;
        double yaw_target_angle = 0.0;
        double yaw_target_velocity = 0.0;
        double yaw_torque = 0.0;
        double delayed_target = 0.0;
        double integral = 0.0;      // 当前积分值（积分补偿累积量）
        double loop_fps = 0.0;      // 后台 loop 线程真实帧率（滑动平均，帧/秒）
        uint64_t ticks_since_set = 0;   // 距上一次 set() 已过去的后台 loop 次数（loop 每循环一次 +1）
        std::vector<double> ref_sequence;   // 本次参考（目标）序列（N 个）
        std::vector<double> pred_sequence;  // 本次预测位置序列（N 个）
    };

    McuMpcController(RobotCommunication* comm, double dt_control, int N,
                     double J, double tau_c, double b, double tau_d,
                     double max_torque, double max_torque_rate,
                     double Q, double R, double Rd, int max_iter,
                     double integral_gain = 0.01,
                     double loop_period = 0.01,
                     double smooth_eps = 1e-6,
                     double feedback_delay_s = 0.0,
                     double velocity_residual_scale = 0.0);
    ~McuMpcController();

    void start();   // 启动后台发送线程（可重复调用，幂等）
    void stop();    // 停止并 join

    /// 线程安全地设置发送参数、MPC 单目标及积分补偿开关。
    /// target_yaw 转换到与 imu_yaw_unwrapped 角度差最小的等效角。
    /// frame_id、plan_id 必须同时提供或同时省略，与参考在同一临界区保存。
    void set(bool auto_aim_enable, bool yaw_torque_only_mode, double target_yaw,
             double pitch_target_angle, bool fire, bool integral_enable = false,
             std::optional<std::uint64_t> frame_id = std::nullopt,
             std::optional<std::uint64_t> plan_id = std::nullopt);

    /// 设置 yaw、pitch、fire 序列及积分补偿开关；各通道独立存储、按序消费。
    /// yaw 首值转换到 imu_yaw ±π 内，后续值转换到前值 ±π 内；序列耗尽后保持末值。
    /// frame_id、plan_id 必须同时提供或同时省略，与序列在同一临界区保存。
    void set(bool auto_aim_enable, bool yaw_torque_only_mode,
             const std::vector<double>& target_yaw_seq,
             const std::vector<double>& pitch_seq,
             const std::vector<bool>& fire_seq,
             bool integral_enable = false,
             std::optional<std::uint64_t> frame_id = std::nullopt,
             std::optional<std::uint64_t> plan_id = std::nullopt);

    // 最新 mpc 结果（线程安全，显示用）
    State state() const;

    /// 开启或关闭逐周期调试记录，默认关闭；切换时保留尚未取出的记录。
    void setControlRecordingEnabled(bool enabled);

    /// 取出当前全部调试记录及溢出计数，并清空内部待取队列，不等待新的控制周期。
    /// 只在独立记录锁内交换队列；调用方在锁外处理数据，不影响 state() 内容。
    ControlRecordBatch takeControlRecords();

    /// 内部最多保留的调试记录数量，超过容量时移除最早记录并累计溢出数量。
    static constexpr std::size_t kControlRecordCapacity = 4096;

private:
    void loop();

    RobotCommunication* comm_;
    YawMpcController mpc_;

    // 设置参数锁（后台线程读取）
    mutable std::mutex set_mtx_;
    bool   auto_aim_enable_ = true;
    bool   yaw_torque_only_mode_ = false;
    bool   integral_enable_ = false;   // 积分补偿开关（后台线程读取）
    double target_yaw_ = 0.0;
    double pitch_target_angle_ = 0.0;
    bool   fire_ = false;
    /// 当前参考对应的原始图像编号，由 set_mtx_ 保护。
    std::optional<std::uint64_t> frame_id_;
    /// 当前参考对应的预测计划编号，由 set_mtx_ 保护。
    std::optional<std::uint64_t> plan_id_;
    /// 当前计划最近消费的 yaw 序列下标，由 set_mtx_ 保护；set() 时清空。
    std::optional<std::uint32_t> reference_index_;

    // 序列模式成员（非空时 loop 优先消费）
    std::deque<double> target_yaw_seq_;
    std::deque<double> pitch_seq_;
    std::deque<bool>   fire_seq_;

    // 最新结果锁（显示线程读取）
    mutable std::mutex state_mtx_;
    State  last_state_;

    double loop_period_ = 0.01;   // 后台 loop 周期（秒，构造传入）
    FrameRateCounter fps_counter_;  // loop 真实帧率统计（滑动窗口 60 帧）
    /// 下一控制周期编号，仅由后台循环修改，set() 和停止后重启均不清零。
    std::uint64_t next_control_tick_id_ = 0;

    /// 调试记录的独立锁，不与参考设置锁或最新状态锁共同持有。
    std::mutex control_records_mtx_;
    /// 是否接收逐周期记录；关闭时后台循环跳过记录组装。
    std::atomic<bool> control_recording_enabled_{false};
    /// 尚未被调试读取方取出的记录。
    std::deque<ControlRecord> control_records_;
    /// 自上次批量读取后移除的最早记录数量，由 control_records_mtx_ 保护。
    std::uint64_t dropped_control_records_ = 0;

    // 距上一次 set() 已过去的后台 loop 次数（set 时清零，loop 每循环一次 +1；
    // 原子计数：set 线程写、loop 线程读写）
    std::atomic<uint64_t> ticks_since_set_{0};

    std::thread thread_;
    std::atomic<bool> running_{false};
};


} // namespace tcs
#endif // TCS_MCU_MPC_CONTROLLER_H
