// RobotCommunicationC.cpp — C API 实现，通过 memcpy 在 C/C++ 类型间转换
//
// 注意（与其他源文件不同）: 本文件是 **C ABI 实现**，所有 tcs_* 导出函数必须
// 保持 C 链接名并留在全局命名空间，因此**不**把本文件包进 namespace tcs；
// 用到的 C++ 类型一律写成 tcs::... 全限定名（与 C 类型 Tcs..._C 明确区分）。

#include "tcs/c_api/RobotCommunicationC.h"
#include "tcs/communication/Communications.hpp"
#include "tcs/mpc/mpc_controller.h"
#include "tcs/mpc/yaw_mpc_controller.h"
#include "tcs/mpc/mcu_mpc_controller.h"
#include <cstring>
#include <new>
#include <algorithm>

// ============================================================================
// 编译期验证：C 结构体与 C++ 结构体逐字段偏移一致
// ============================================================================
#define TCS_ASSERT_OFFSET(CType, CppType, field) \
    static_assert(offsetof(CType, field) == offsetof(CppType, field), "offset mismatch: " #field)

TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, frame_header1);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, frame_header2);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, protocol_version);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, data_size);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, auto_aim_enable);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, fire);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, pitch_target_angle);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, yaw_torque_only_mode);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, yaw_target_angle);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, yaw_target_velocity);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, yaw_torque);
TCS_ASSERT_OFFSET(TcsMcuSendPacket_C, tcs::mcu::SendPacket, crc8);

TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, frame_header1);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, frame_header2);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, protocol_version);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, data_size);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, bullet_velocity);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, pitch_angle);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, yaw_angle);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, yaw_omega);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, chassis_imu_yaw);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, chassis_imu_omega);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, mark);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, color);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, auto_aim_switch);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, yaw_temperature);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, fold);
TCS_ASSERT_OFFSET(TcsMcuReceivePacket_C, tcs::mcu::ReceivePacket, crc8);

static_assert(sizeof(TcsMcuSendPacket_C)    == sizeof(tcs::mcu::SendPacket),    "McuSendPacket size mismatch");
static_assert(sizeof(TcsMcuReceivePacket_C) == sizeof(tcs::mcu::ReceivePacket), "McuReceivePacket size mismatch");
static_assert(sizeof(TcsImuSendPacket_C)    == sizeof(tcs::imu::SendPacket),    "ImuSendPacket size mismatch");
static_assert(sizeof(TcsImuReceivePacket_C) == sizeof(tcs::imu::ReceivePacket), "ImuReceivePacket size mismatch");

// ============================================================================
// 内部实现：持有 RobotCommunication 实例
// ============================================================================
struct TcsRobotCommHandle {
    tcs::RobotCommunication impl;
};

// ============================================================================
// 辅助转换函数
// ============================================================================
static inline void copyToCpp(const TcsMcuSendPacket_C& src, tcs::mcu::SendPacket& dst) {
    std::memcpy(&dst, &src, sizeof(src));
}
static inline void copyToC(const tcs::mcu::ReceivePacket& src, TcsMcuReceivePacket_C& dst) {
    std::memcpy(&dst, &src, sizeof(src));
}
static inline void copyToCpp(const TcsImuSendPacket_C& src, tcs::imu::SendPacket& dst) {
    std::memcpy(&dst, &src, sizeof(src));
}
static inline void copyToC(const tcs::imu::ReceivePacket& src, TcsImuReceivePacket_C& dst) {
    std::memcpy(&dst, &src, sizeof(src));
}

// ============================================================================
// API 实现
// ============================================================================

extern "C" {

TcsRobotCommHandle* tcs_robot_comm_create(void) {
    return new (std::nothrow) TcsRobotCommHandle;
}

void tcs_robot_comm_destroy(TcsRobotCommHandle* handle) {
    delete handle;
}

TcsRobotLatestData_C tcs_robot_comm_get_latest_data(TcsRobotCommHandle* handle) {
    TcsRobotLatestData_C result{};
    if (!handle) return result;

    auto data = handle->impl.getLatestData();

    result.imu_valid = data.imu_valid;
    result.mcu_valid = data.mcu_valid;

    if (data.imu_valid)
        copyToC(data.imu_packet, result.imu_packet);
    if (data.mcu_valid)
        copyToC(data.mcu_packet, result.mcu_packet);

    return result;
}

TcsRobotFusedData_C tcs_robot_comm_get_fused_data(TcsRobotCommHandle* handle) {
    TcsRobotFusedData_C result{};
    if (!handle) return result;

    auto f = handle->impl.getFused();

    result.valid             = f.valid;
    result.yaw_pos           = f.yaw_pos;
    result.yaw_rate          = f.yaw_rate;
    result.chassis_yaw       = f.chassis_yaw;
    result.chassis_pitch     = f.chassis_pitch;
    result.chassis_roll      = f.chassis_roll;
    result.imu_yaw_unwrapped = f.imu_yaw_unwrapped;

    return result;
}

TcsRobotStrictPose_C tcs_robot_comm_get_strict_pose(TcsRobotCommHandle* handle) {
    TcsRobotStrictPose_C result{};
    if (!handle) return result;

    auto sp = handle->impl.getStrictPose();

    result.imu_euler_yaw    = sp.imu_euler_yaw;
    result.imu_euler_pitch  = sp.imu_euler_pitch;
    result.imu_euler_roll   = sp.imu_euler_roll;
    result.yaw_pos          = sp.yaw_pos;
    result.pitch_angle      = sp.pitch_angle;
    result.chassis_yaw      = sp.chassis_yaw;
    result.chassis_pitch    = sp.chassis_pitch;
    result.chassis_roll     = sp.chassis_roll;

    return result;
}

bool tcs_robot_comm_send_to_mcu(TcsRobotCommHandle* handle, const TcsMcuSendPacket_C* packet) {
    if (!handle || !packet) return false;

    tcs::mcu::SendPacket cpp_pkt;
    copyToCpp(*packet, cpp_pkt);

    return handle->impl.sendToMcu(cpp_pkt);
}

bool tcs_robot_comm_send_to_imu(TcsRobotCommHandle* handle, const TcsImuSendPacket_C* packet) {
    if (!handle || !packet) return false;

    tcs::imu::SendPacket cpp_pkt;
    copyToCpp(*packet, cpp_pkt);
    return handle->impl.sendToImu(cpp_pkt);
}

void tcs_robot_comm_stop(TcsRobotCommHandle* handle) {
    if (handle)
        handle->impl.stop();
}

// ============================================================================
// 实车 MPC 控制器 C API（替代旧 mpc_c_api）
// ============================================================================

TcsYawMpcHandle_C tcs_yaw_mpc_create(TcsRobotCommHandle* comm, double dt_control, int N,
                              double J, double tau_c, double b, double tau_d,
                              double max_torque, double max_torque_rate,
                              double Q, double R, double Rd, int max_iter,
                              double smooth_eps) {
    if (!comm) return nullptr;
    try {
        return static_cast<TcsYawMpcHandle_C>(new tcs::YawMpcController(
            &comm->impl, dt_control, N,
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            Q, R, Rd, max_iter,
            /*integral_gain=*/0.01, smooth_eps));
    } catch (...) {
        return nullptr;
    }
}

void tcs_yaw_mpc_destroy(TcsYawMpcHandle_C handle) {
    delete static_cast<tcs::YawMpcController*>(handle);
}

TcsYawMpcStepResult_C tcs_yaw_mpc_step(TcsYawMpcHandle_C handle, double target_yaw) {
    TcsYawMpcStepResult_C out{};
    if (!handle) return out;
    auto r = static_cast<tcs::YawMpcController*>(handle)->step(target_yaw);
    out.yaw_target_angle    = r.yaw_target_angle;
    out.yaw_target_velocity = r.yaw_target_velocity;
    out.yaw_torque          = r.yaw_torque;
    out.delayed_target      = r.delayed_target;
    return out;
}

// ============================================================================
// 实车 MCU 控制封装 C API（设置 + 后台发送线程，周期默认 100Hz）
// ============================================================================

TcsMcuMpcHandle_C tcs_mcu_mpc_create(TcsRobotCommHandle* comm, double dt_control, int N,
                              double J, double tau_c, double b, double tau_d,
                              double max_torque, double max_torque_rate,
                              double Q, double R, double Rd, int max_iter,
                              double smooth_eps) {
    if (!comm) return nullptr;
    try {
        auto* ctl = new tcs::McuMpcController(
            &comm->impl, dt_control, N,
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            Q, R, Rd, max_iter,
            /*integral_gain=*/0.01, /*loop_period=*/0.01, smooth_eps);
        ctl->start();   // 自动启动后台发送线程（周期默认 100Hz）
        return static_cast<TcsMcuMpcHandle_C>(ctl);
    } catch (...) {
        return nullptr;
    }
}

void tcs_mcu_mpc_destroy(TcsMcuMpcHandle_C handle) {
    delete static_cast<tcs::McuMpcController*>(handle);   // 析构内 stop + join
}

void tcs_mcu_mpc_step(TcsMcuMpcHandle_C handle,
                  uint8_t auto_aim_enable, uint8_t yaw_torque_only_mode,
                  double target_yaw, float pitch_target_angle, uint8_t fire) {
    if (!handle) return;
    static_cast<tcs::McuMpcController*>(handle)->set(
        auto_aim_enable != 0, yaw_torque_only_mode != 0, target_yaw,
        pitch_target_angle, fire != 0);
}

TcsMcuMpcState_C tcs_mcu_mpc_get_state(TcsMcuMpcHandle_C handle) {
    TcsMcuMpcState_C out{};
    if (!handle) return out;
    auto s = static_cast<tcs::McuMpcController*>(handle)->state();
    out.yaw_target_angle    = s.yaw_target_angle;
    out.yaw_target_velocity = s.yaw_target_velocity;
    out.yaw_torque          = s.yaw_torque;
    out.delayed_target      = s.delayed_target;
    return out;
}

// ============================================================================
// 直接暴露 MPCController（纯求解器，不依赖通信；供仿真/离线求解）
// ============================================================================

TcsMpcControllerHandle_C tcs_mpc_controller_create(double dt_control, double dt_sim,
                                            double J, double tau_c, double b, double tau_d,
                                            double max_torque, double max_torque_rate,
                                            int N, double Q, double R, double Rd, int max_iter,
                                            double smooth_eps) {
    try {
        return static_cast<TcsMpcControllerHandle_C>(new tcs::MPCController(
            dt_control, dt_sim, J, tau_c, b, tau_d,
            max_torque, max_torque_rate, N, Q, R, Rd, max_iter, smooth_eps));
    } catch (...) {
        return nullptr;
    }
}

void tcs_mpc_controller_destroy(TcsMpcControllerHandle_C handle) {
    delete static_cast<tcs::MPCController*>(handle);
}

TcsMpcStepResult_C tcs_mpc_controller_step(TcsMpcControllerHandle_C handle,
                                    double theta, double omega,
                                    const double* theta_ref, int ref_len) {
    TcsMpcStepResult_C out{};
    if (!handle || !theta_ref || ref_len <= 0) return out;
    try {
        std::vector<double> ref(theta_ref, theta_ref + ref_len);
        auto r = static_cast<tcs::MPCController*>(handle)->step(theta, omega, ref);
        out.torque = r.torque;
        out.theta  = r.theta;
        out.omega  = r.omega;
    } catch (...) {
        // 求解异常：返回 0 力矩与当前状态
        out.torque = 0.0;
        out.theta  = theta;
        out.omega  = omega;
    }
    return out;
}

int tcs_mpc_controller_get_last_ref(TcsMpcControllerHandle_C handle, double* out, int max_len) {
    if (!handle || !out || max_len <= 0) return 0;
    const auto& ref = static_cast<tcs::MPCController*>(handle)->lastRef();
    int n = static_cast<int>(std::min<size_t>(ref.size(), static_cast<size_t>(max_len)));
    std::memcpy(out, ref.data(), sizeof(double) * n);
    return n;
}

int tcs_mpc_controller_get_last_pred(TcsMpcControllerHandle_C handle, double* out, int max_len) {
    if (!handle || !out || max_len <= 0) return 0;
    const auto& pred = static_cast<tcs::MPCController*>(handle)->lastPred();
    int n = static_cast<int>(std::min<size_t>(pred.size(), static_cast<size_t>(max_len)));
    std::memcpy(out, pred.data(), sizeof(double) * n);
    return n;
}

void tcs_mpc_controller_reset(TcsMpcControllerHandle_C handle) {
    if (!handle) return;
    static_cast<tcs::MPCController*>(handle)->reset();
}

} // extern "C"
