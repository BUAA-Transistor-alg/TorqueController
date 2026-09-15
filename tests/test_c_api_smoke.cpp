// test_c_api_smoke.cpp — 前缀化（模块标识 tcs）后的自检，**不依赖任何串口硬件**
//
// 覆盖:
//   1. C++ 协议结构体默认值 / 尺寸（tcs::mcu::*、tcs::imu::*）与 C 结构体（Tcs*_C）
//      逐字段一致（与 RobotCommunicationC.cpp 里的 static_assert 同源）；
//   2. tcs::CRC8_Check_Sum / tcs::CRC32_Calculate 的数值（独立参考实现算出的期望值）；
//   3. C ABI 纯求解器路径: tcs_mpc_controller_create / _step / _get_last_ref / _get_last_pred /
//      _reset / _destroy（不涉及通信，无硬件也能跑，且结果确定）；
//   4. C ABI 通信路径: tcs_robot_comm_create / _get_latest_data / _get_fused_data /
//      _get_strict_pose / _send_to_mcu / _stop / _destroy（无硬件时 valid 全 0 属预期）；
//   5. C++ 高层封装 tcs::RobotController（构造 / set / getState / 析构，含后台线程，
//      无硬件时 fused.valid == 0）。
//
// 用法: ./build/tcs_test_c_api_smoke（退出码 0 = 全部通过，1 = 有失败项）

#include "tcs/RobotController.h"
#include "tcs/c_api/RobotCommunicationC.h"
#include "tcs/communication/CRC.h"
#include "tcs/communication/Protocol.hpp"
#include "tcs/mpc/mpc_controller.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

int g_failed = 0;

void check(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failed;
}

bool isFinite(double v) { return std::isfinite(v); }

} // namespace

int main() {
    std::printf("=== tcs C API / C++ 命名空间自检（无硬件）===\n");

    // ── 1. 协议结构体（C++ 侧默认值 + C/C++ 布局一致）──
    {
        tcs::mcu::SendPacket mcu_tx{};
        tcs::mcu::ReceivePacket mcu_rx{};
        tcs::imu::SendPacket imu_tx{};
        tcs::imu::ReceivePacket imu_rx{};

        check(mcu_tx.frame_header1 == 0x42 && mcu_tx.frame_header2 == 0x52 &&
                  mcu_tx.protocol_version == 0x02 && mcu_tx.data_size == 23,
              "tcs::mcu::SendPacket 默认前导/长度 = 0x42 0x52 0x02 / 23");
        check(imu_tx.frame_header1 == 0xA7 && imu_tx.frame_header2 == 0xB6 &&
                  imu_tx.frame_header3 == 0xC5,
              "tcs::imu::SendPacket 默认前导 = 0xA7 0xB6 0xC5");
        check(tcs::mcu::PREAMBLE_SIZE == 3 && tcs::imu::PREAMBLE_SIZE == 3,
              "tcs::mcu::/tcs::imu::PREAMBLE_SIZE == 3");

        check(sizeof(TcsMcuSendPacket_C) == sizeof(tcs::mcu::SendPacket) &&
                  sizeof(TcsMcuReceivePacket_C) == sizeof(tcs::mcu::ReceivePacket) &&
                  sizeof(TcsImuSendPacket_C) == sizeof(tcs::imu::SendPacket) &&
                  sizeof(TcsImuReceivePacket_C) == sizeof(tcs::imu::ReceivePacket),
              "C 结构体与 C++ 结构体 sizeof 一致");
        check(offsetof(TcsMcuSendPacket_C, yaw_target_angle) ==
                      offsetof(tcs::mcu::SendPacket, yaw_target_angle) &&
                  offsetof(TcsMcuReceivePacket_C, yaw_angle) ==
                      offsetof(tcs::mcu::ReceivePacket, yaw_angle) &&
                  offsetof(TcsImuReceivePacket_C, euler_yaw) ==
                      offsetof(tcs::imu::ReceivePacket, euler_yaw),
              "C 结构体与 C++ 结构体关键字段偏移一致");
        check(sizeof(TcsRobotLatestData_C) > 0 && sizeof(TcsRobotFusedData_C) > 0 &&
                  sizeof(TcsRobotStrictPose_C) > 0,
              "Tcs*_C 聚合结构体可实例化");

        (void)mcu_rx; (void)imu_rx;
    }

    // ── 2. CRC（期望值由独立 Python 参考实现算出，与当前表/算法一致）──
    {
        const uint8_t preamble[3] = {0x42, 0x52, 0x02};
        const uint8_t text[16] = {'T','o','r','q','u','e','C','o','n','t','r','o','l','l','e','r'};
        check(tcs::CRC8_Check_Sum(preamble, 3) == 0xF6, "tcs::CRC8_Check_Sum(preamble) == 0xF6");
        check(tcs::CRC8_Check_Sum(text, sizeof(text)) == 0x48,
              "tcs::CRC8_Check_Sum(\"TorqueController\") == 0x48");
        check(tcs::CRC32_Calculate(text, sizeof(text)) == 0x46C6A130u,
              "tcs::CRC32_Calculate(\"TorqueController\") == 0x46C6A130");
    }

    // ── 3. C ABI 纯 MPC 求解器（无通信）──
    {
        const int kN = 20;
        TcsMpcControllerHandle_C h = tcs_mpc_controller_create(
            /*dt_control=*/0.01, /*dt_sim=*/0.01,
            /*J=*/0.016541, /*tau_c=*/0.097297, /*b=*/0.032100, /*tau_d=*/0.0,
            /*max_torque=*/1.0, /*max_torque_rate=*/40.0,
            /*N=*/kN, /*Q=*/5.0, /*R=*/0.01, /*Rd=*/0.1, /*max_iter=*/30,
            /*smooth_eps=*/1e-6);
        check(h != nullptr, "tcs_mpc_controller_create() 返回非 NULL");

        std::vector<double> ref(kN, 0.5);
        TcsMpcStepResult_C r = tcs_mpc_controller_step(h, /*theta=*/0.0, /*omega=*/0.0,
                                                       ref.data(), static_cast<int>(ref.size()));
        check(isFinite(r.torque) && isFinite(r.theta) && isFinite(r.omega),
              "tcs_mpc_controller_step() 输出有限");
        check(std::fabs(r.torque) <= 1.0 + 1e-9, "求解出力矩在 ±max_torque 内");

        std::vector<double> out_ref(kN, 0.0), out_pred(kN, 0.0);
        const int n_ref = tcs_mpc_controller_get_last_ref(h, out_ref.data(), kN);
        const int n_pred = tcs_mpc_controller_get_last_pred(h, out_pred.data(), kN);
        check(n_ref == kN && n_pred == kN, "get_last_ref / get_last_pred 返回 N 个点");
        bool ref_match = true;
        for (int i = 0; i < n_ref; ++i) ref_match = ref_match && (out_ref[i] == ref[i]);
        check(ref_match, "get_last_ref 与传入参考序列逐点一致");

        tcs_mpc_controller_reset(h);
        tcs_mpc_controller_destroy(h);
        check(true, "tcs_mpc_controller_reset/ destroy 正常");
    }

    // ── 4. C ABI 通信路径（无硬件：valid 全 0 属预期）──
    {
        TcsRobotCommHandle* comm = tcs_robot_comm_create();
        check(comm != nullptr, "tcs_robot_comm_create() 返回非 NULL（无硬件亦可）");

        TcsRobotLatestData_C latest = tcs_robot_comm_get_latest_data(comm);
        TcsRobotFusedData_C fused = tcs_robot_comm_get_fused_data(comm);
        TcsRobotStrictPose_C strict = tcs_robot_comm_get_strict_pose(comm);
        check(latest.imu_valid == 0 && latest.mcu_valid == 0 && fused.valid == 0,
              "无硬件时 latest/fused valid 均为 0");
        check(isFinite(strict.yaw_pos) && isFinite(strict.chassis_yaw) &&
                  isFinite(strict.imu_euler_yaw),
              "tcs_robot_comm_get_strict_pose() 输出有限");

        TcsMcuSendPacket_C pkt{};
        pkt.frame_header1 = 0x42;
        pkt.frame_header2 = 0x52;
        pkt.protocol_version = 0x02;
        pkt.data_size = 23;
        const bool sent = tcs_robot_comm_send_to_mcu(comm, &pkt);
        check(!sent, "无串口时 tcs_robot_comm_send_to_mcu() 返回 false");

        tcs_robot_comm_stop(comm);
        tcs_robot_comm_destroy(comm);
        check(true, "tcs_robot_comm_stop/ destroy 正常");
    }

    // ── 5. C++ 高层封装 tcs::RobotController（含后台线程）──
    {
        tcs::RobotController rc(/*dt_control=*/0.01, /*N=*/20,
                               /*J=*/0.016541, /*tau_c=*/0.097297, /*b=*/0.032100, /*tau_d=*/0.0,
                               /*max_torque=*/1.0, /*max_torque_rate=*/40.0,
                               /*Q=*/5.0, /*R=*/0.01, /*Rd=*/0.1, /*max_iter=*/30,
                               /*integral_gain=*/0.01, /*mpc_loop_period=*/0.01,
                               tcs::McuDataPreprocessor::LinearParams{},
                               /*sequence_mode=*/false, /*smooth_eps=*/1e-6);
        rc.set(/*auto_aim_enable=*/true, /*yaw_torque_only_mode=*/false,
               /*target_yaw=*/0.5, /*pitch_target_angle=*/0.1, /*fire=*/false,
               /*integral_enable=*/false);
        tcs::RobotController::State st = rc.getState();
        check(isFinite(st.mpc.yaw_torque) && isFinite(st.fused.yaw_pos),
              "tcs::RobotController::getState() 输出有限");
        check(st.fused.valid == false, "无硬件时 fused.valid == false");
        check(rc.mode() == tcs::RobotController::Mode::SINGLE, "mode() == SINGLE");
    }

    if (g_failed == 0) {
        std::printf("全部通过。\n");
        return 0;
    }
    std::printf("失败项: %d\n", g_failed);
    return 1;
}
