"""torque_controller — 单 yaw 云台力矩控制（模块标识 tcs）Python ctypes 绑定包

C API 头文件: ``include/tcs/c_api/RobotCommunicationC.h``（纯 C ABI，符号前缀 ``tcs_``）
动态库产物:  ``build/libtcs_robot_comm_c.so``
桥接实现:    :mod:`torque_controller._bridge`

对外命名一律带 ``Tcs`` 前缀（与 C++ 侧 ``namespace tcs`` / C ABI 的 ``tcs_`` / ``Tcs*_C``
一一对应），这样同一进程同时使用 tcbs（双级 yaw 子模组）与本子模组的 Python 绑定时不撞名::

    from torque_controller import TcsRobotCommunication, TcsMcuSendPacket

    with TcsRobotCommunication() as comm:
        data = comm.get_latest_data()     # MCU / IMU 原始数据（valid 标志）
        fused = comm.get_fused_data()     # 融合输出（yaw_pos / yaw_rate / 底盘姿态）
        comm.send_to_mcu(TcsMcuSendPacket(auto_aim_enable=1))

无硬件时 ``TcsRobotCommunication()`` 同样可以创建（串口打开失败只在后台重连），
``get_latest_data()`` / ``get_fused_data()`` 的 ``valid`` 全为 0 —— 这是设计要求，
便于离线跑通整条调用链。

纯求解器（不依赖通信）::

    from torque_controller import TcsMPCController
    mpc = TcsMPCController(dt_control=0.01, N=20)
    r = mpc.step(theta=0.0, omega=0.0, theta_ref=[0.5] * 20)

运行环境:
  * 先构建动态库: ``cmake -S . -B build && cmake --build build``
  * 让 Python 找到包: ``PYTHONPATH=python python3 your_script.py``
  * 库不在默认位置时: ``export TCS_LIB=/abs/path/libtcs_robot_comm_c.so``
    （历史名 ``TORQUE_CONTROLLER_LIB`` 仍兼容）
"""

from torque_controller._bridge import (
    TcsRobotCommunication,
    TcsMPCController,
    TcsYawMpcController,
    TcsMcuMpcController,
    TcsRobotLatestData,
    TcsRobotFusedData,
    TcsRobotStrictPose,
    TcsMcuSendPacket,
    TcsMcuReceivePacket,
    TcsImuSendPacket,
    TcsImuReceivePacket,
)
