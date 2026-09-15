"""_bridge.py — ctypes 底层桥接，不对外暴露

对应 C 头文件: ``include/tcs/c_api/RobotCommunicationC.h``（纯 C ABI，符号前缀 ``tcs_``）。
本模块只做"名字 + 内存布局"的映射: ctypes 结构体与 C 结构体 ``Tcs*_C`` 逐字段一致
（字段顺序/对齐不可改），C++ 侧类（``tcs::RobotCommunication`` / ``tcs::MPCController`` …）
一律通过 C ABI 句柄间接使用。

库文件查找顺序（模块标识 tcs）:
  1. 环境变量 ``TCS_LIB``（本仓库专用，优先）
  2. 环境变量 ``TORQUE_CONTROLLER_LIB``（历史名，保留兼容）
  3. 仓库 ``build/libtcs_robot_comm_c.so``（以及 ``build/lib/`` 下）
  4. 交给系统加载器 ``libtcs_robot_comm_c.so``
"""

import ctypes
import os
from ctypes import Structure, c_uint8, c_float, c_double, c_uint32, c_bool, c_int

_LIB_ENVS = ("TCS_LIB", "TORQUE_CONTROLLER_LIB")
_LIB_NAME = "libtcs_robot_comm_c.so"


def _candidate_paths():
    for env_name in _LIB_ENVS:
        env = os.environ.get(env_name)
        if env:
            yield env
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    yield os.path.join(repo_root, "build", _LIB_NAME)
    yield os.path.join(repo_root, "build", "lib", _LIB_NAME)
    yield _LIB_NAME          # 裸库名 → 交给系统加载器


def _find_lib():
    tried = []
    for path in _candidate_paths():
        if os.path.sep in path:
            if os.path.isfile(path):
                return path
            tried.append(path)
            continue
        return path
    raise OSError(
        "无法定位 " + _LIB_NAME + "，请先构建（cmake -S . -B build && cmake --build build）"
        "或用环境变量 " + " / ".join(_LIB_ENVS) + " 指定路径。已尝试: " + ", ".join(tried)
    )


_lib = ctypes.CDLL(_find_lib())


class TcsMcuSendPacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1",       c_uint8),
        ("frame_header2",       c_uint8),
        ("protocol_version",    c_uint8),
        ("data_size",           c_uint8),
        ("auto_aim_enable",     c_uint8),
        ("fire",                c_uint8),
        ("pitch_target_angle",  c_float),
        ("yaw_torque_only_mode", c_uint8),
        ("yaw_target_angle",    c_double),
        ("yaw_target_velocity", c_float),
        ("yaw_torque",          c_float),
        ("crc8",                c_uint8),
    ]
    def __init__(self, auto_aim_enable=0, pitch_target_angle=0.0,
                 yaw_torque=0.0, fire=0,
                 yaw_torque_only_mode=0, yaw_target_angle=0.0, yaw_target_velocity=0.0):
        super().__init__()
        self.frame_header1 = 0x42; self.frame_header2 = 0x52
        self.protocol_version = 0x02; self.data_size = 23
        self.auto_aim_enable = auto_aim_enable
        self.fire = fire
        self.pitch_target_angle = pitch_target_angle
        self.yaw_torque_only_mode = yaw_torque_only_mode
        self.yaw_target_angle = yaw_target_angle
        self.yaw_target_velocity = yaw_target_velocity
        self.yaw_torque = yaw_torque
        self.crc8 = 0


class TcsMcuReceivePacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1",     c_uint8), ("frame_header2",     c_uint8),
        ("protocol_version",  c_uint8), ("data_size",         c_uint8),
        ("bullet_velocity",   c_float), ("pitch_angle",       c_float),
        ("yaw_angle",         c_double),("yaw_omega",         c_float),
        ("chassis_imu_yaw",   c_float), ("chassis_imu_omega", c_float),
        ("mark",              c_uint8), ("color",             c_uint8),
        ("auto_aim_switch",   c_uint8), ("yaw_temperature",   c_uint8), ("crc8",              c_uint8),
    ]


class TcsImuSendPacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1", c_uint8), ("frame_header2", c_uint8),
        ("frame_header3", c_uint8), ("data_size",     c_uint8),
        ("crc32",         c_uint32),
    ]
    def __init__(self):
        super().__init__()
        self.frame_header1 = 0xA7; self.frame_header2 = 0xB6
        self.frame_header3 = 0xC5; self.data_size = 0; self.crc32 = 0


class TcsImuReceivePacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1", c_uint8), ("frame_header2", c_uint8),
        ("frame_header3", c_uint8), ("data_size",     c_uint8),
        ("gx", c_float), ("gy", c_float), ("gz", c_float),
        ("ax", c_float), ("ay", c_float), ("az", c_float),
        ("euler_yaw", c_double), ("euler_pitch", c_double), ("euler_roll", c_double),
        ("dt_one_tenth_ms", c_uint32), ("crc32", c_uint32),
    ]


class TcsRobotLatestData(Structure):
    _pack_ = 1
    _fields_ = [
        ("imu_valid",  c_bool), ("imu_packet", TcsImuReceivePacket),
        ("mcu_valid",  c_bool), ("mcu_packet", TcsMcuReceivePacket),
    ]


class TcsRobotStrictPose(Structure):
    """严格反解数据包（独立输出）：所有角度 wrap 到 (-π,π]；始终有效，
    缺失数据以 0 参与；R_imu = R_chassis·Rz(yaw_pos)·Rx(pitch_angle) 恒成立"""
    _fields_ = [
        ("imu_euler_yaw",    c_double),   # 反解输入：IMU 欧拉角（始终为 imu 传来数据）
        ("imu_euler_pitch",  c_double),
        ("imu_euler_roll",   c_double),
        ("yaw_pos",          c_double),   # 反解输入：yaw 关节位置（wrap 后）
        ("pitch_angle",      c_double),   # 反解输入：pitch 关节角（wrap 后）
        ("chassis_yaw",      c_double),   # 严格反解底盘欧拉角（wrap 后）
        ("chassis_pitch",    c_double),
        ("chassis_roll",     c_double),
    ]


class TcsRobotFusedData(Structure):
    """融合输出：高频 yaw 位置/速度 + 底盘 world 系姿态"""
    _fields_ = [
        ("valid",              c_bool),
        ("yaw_pos",            c_double),   # yaw 关节解卷绕位置 (rad, 多圈)
        ("yaw_rate",           c_double),   # yaw 关节速度 (rad/s)
        ("chassis_yaw",        c_double),   # 底盘 world 系 yaw（解卷绕）
        ("chassis_pitch",      c_double),   # 底盘 world 系 pitch
        ("chassis_roll",       c_double),   # 底盘 world 系 roll
        ("imu_yaw_unwrapped",  c_double),   # IMU euler yaw 解卷绕
    ]


class TcsYawMpcStepResult(Structure):
    """yaw MPC 一步结果（返回将要发送的值，不发送）"""
    _fields_ = [
        ("yaw_target_angle",    c_double),   # 预测位置 → yaw_target_angle (rad)
        ("yaw_target_velocity", c_double),   # 预测速度 → yaw_target_velocity (rad/s)
        ("yaw_torque",          c_double),   # 控制力矩 → yaw_torque (N·m)
        ("delayed_target",      c_double),   # 当前参考（延迟 dt*N 步的目标）
    ]


class TcsMcuMpcState(Structure):
    """实车 MCU 控制封装的最新 mpc 结果（后台线程更新）"""
    _fields_ = [
        ("yaw_target_angle",    c_double),
        ("yaw_target_velocity", c_double),
        ("yaw_torque",          c_double),
        ("delayed_target",      c_double),
    ]


class TcsMpcStepResult(Structure):
    """TcsMPCController 一步求解结果（纯求解器，不依赖通信）"""
    _fields_ = [
        ("torque", c_double),   # 第一步控制力矩 (N·m)
        ("theta",  c_double),   # 第一步预测位置 (rad)
        ("omega",  c_double),   # 第一步预测速度 (rad/s)
    ]

# ── 函数签名 ──
_lib.tcs_robot_comm_create.restype             = ctypes.c_void_p
_lib.tcs_robot_comm_destroy.argtypes           = [ctypes.c_void_p]
_lib.tcs_robot_comm_get_latest_data.argtypes   = [ctypes.c_void_p]
_lib.tcs_robot_comm_get_latest_data.restype    = TcsRobotLatestData
_lib.tcs_robot_comm_get_fused_data.argtypes    = [ctypes.c_void_p]
_lib.tcs_robot_comm_get_fused_data.restype     = TcsRobotFusedData
_lib.tcs_robot_comm_get_strict_pose.argtypes   = [ctypes.c_void_p]
_lib.tcs_robot_comm_get_strict_pose.restype    = TcsRobotStrictPose
_lib.tcs_robot_comm_send_to_mcu.argtypes       = [ctypes.c_void_p, ctypes.POINTER(TcsMcuSendPacket)]
_lib.tcs_robot_comm_send_to_mcu.restype        = c_bool
_lib.tcs_robot_comm_send_to_imu.argtypes       = [ctypes.c_void_p, ctypes.POINTER(TcsImuSendPacket)]
_lib.tcs_robot_comm_send_to_imu.restype        = c_bool
_lib.tcs_robot_comm_stop.argtypes              = [ctypes.c_void_p]

_lib.tcs_yaw_mpc_create.argtypes = [
    ctypes.c_void_p,                       # TcsRobotCommHandle*
    c_double,                              # dt_control
    c_int,                                 # N
    c_double, c_double, c_double, c_double,   # J, tau_c, b, tau_d
    c_double, c_double,                    # max_torque, max_torque_rate
    c_double, c_double, c_double,          # Q, R, Rd
    c_int,                                 # max_iter
    c_double,                              # smooth_eps（位置误差平滑绝对值常数 a）
]
_lib.tcs_yaw_mpc_create.restype = ctypes.c_void_p
_lib.tcs_yaw_mpc_destroy.argtypes = [ctypes.c_void_p]
_lib.tcs_yaw_mpc_step.argtypes = [ctypes.c_void_p, c_double]
_lib.tcs_yaw_mpc_step.restype = TcsYawMpcStepResult

_lib.tcs_mcu_mpc_create.argtypes = _lib.tcs_yaw_mpc_create.argtypes
_lib.tcs_mcu_mpc_create.restype = ctypes.c_void_p
_lib.tcs_mcu_mpc_destroy.argtypes = [ctypes.c_void_p]
_lib.tcs_mcu_mpc_step.argtypes = [
    ctypes.c_void_p,                       # handle
    c_uint8, c_uint8,                      # auto_aim_enable, yaw_torque_only_mode
    c_double,                              # target_yaw
    c_float, c_uint8,                      # pitch_target_angle, fire
]
_lib.tcs_mcu_mpc_get_state.argtypes = [ctypes.c_void_p]
_lib.tcs_mcu_mpc_get_state.restype = TcsMcuMpcState

_lib.tcs_mpc_controller_create.argtypes = [
    c_double,                              # dt_control
    c_double,                              # dt_sim
    c_double, c_double, c_double, c_double,   # J, tau_c, b, tau_d
    c_double, c_double,                    # max_torque, max_torque_rate
    c_int,                                 # N
    c_double, c_double, c_double,          # Q, R, Rd
    c_int,                                 # max_iter
    c_double,                              # smooth_eps（位置误差平滑绝对值常数 a）
]
_lib.tcs_mpc_controller_create.restype = ctypes.c_void_p
_lib.tcs_mpc_controller_destroy.argtypes = [ctypes.c_void_p]
_lib.tcs_mpc_controller_step.argtypes = [
    ctypes.c_void_p,                       # handle
    c_double, c_double,                    # theta, omega
    ctypes.POINTER(c_double), c_int,       # theta_ref, ref_len
]
_lib.tcs_mpc_controller_step.restype = TcsMpcStepResult
_lib.tcs_mpc_controller_get_last_ref.argtypes = [
    ctypes.c_void_p, ctypes.POINTER(c_double), c_int,
]
_lib.tcs_mpc_controller_get_last_ref.restype = c_int
_lib.tcs_mpc_controller_get_last_pred.argtypes = [
    ctypes.c_void_p, ctypes.POINTER(c_double), c_int,
]
_lib.tcs_mpc_controller_get_last_pred.restype = c_int
_lib.tcs_mpc_controller_reset.argtypes = [ctypes.c_void_p]


class TcsYawMpcController:
    """yaw MPC 求解器（C++ 实现：tcs::YawMpcController；返回发送值，不发送）"""

    def __init__(self, comm_handle, dt_control, N,
                 J, tau_c, b, tau_d=0.0,
                 max_torque=1.0, max_torque_rate=10.0,
                 Q=5.0, R=0.01, Rd=0.1, max_iter=30,
                 smooth_eps=1e-6):
        self._handle = _lib.tcs_yaw_mpc_create(
            comm_handle, float(dt_control), int(N),
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            Q, R, Rd, int(max_iter), float(smooth_eps),
        )
        if not self._handle:
            raise RuntimeError("tcs_yaw_mpc_create() returned NULL")

    def step(self, target_yaw) -> TcsYawMpcStepResult:
        """传入目标位置，返回 (yaw_target_angle, yaw_target_velocity, yaw_torque, delayed_target)。"""
        return _lib.tcs_yaw_mpc_step(self._handle, float(target_yaw))

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.tcs_yaw_mpc_destroy(self._handle)
            self._handle = None


class TcsMcuMpcController:
    """实车 MCU 控制封装（C++: tcs::McuMpcController；设置维护 + 后台 100Hz 发送线程）"""

    def __init__(self, comm_handle, dt_control, N,
                 J, tau_c, b, tau_d=0.0,
                 max_torque=1.0, max_torque_rate=10.0,
                 Q=5.0, R=0.01, Rd=0.1, max_iter=30,
                 smooth_eps=1e-6):
        self._handle = _lib.tcs_mcu_mpc_create(
            comm_handle, float(dt_control), int(N),
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            Q, R, Rd, int(max_iter), float(smooth_eps),
        )
        if not self._handle:
            raise RuntimeError("tcs_mcu_mpc_create() returned NULL")

    def set(self, auto_aim_enable=1, yaw_torque_only_mode=0, target_yaw=0.0,
            pitch_target_angle=0.0, fire=0):
        """设置发送参数 + mpc 目标（顺序：auto_aim_enable, yaw_torque_only_mode,
        target_yaw, pitch_target_angle, fire）。target_yaw 由 C++ 自动转换到与
        imu_yaw_unwrapped 同一圈内；后台线程固定 100Hz 求解并发送给 MCU。"""
        _lib.tcs_mcu_mpc_step(self._handle,
                              int(auto_aim_enable), int(yaw_torque_only_mode),
                              float(target_yaw), float(pitch_target_angle), int(fire))

    def get_state(self) -> TcsMcuMpcState:
        """最新 mpc 结果（后台线程更新，供显示）"""
        return _lib.tcs_mcu_mpc_get_state(self._handle)

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.tcs_mcu_mpc_destroy(self._handle)
            self._handle = None


class TcsMPCController:
    """MPCController（C++ 侧 tcs::MPCController；纯求解器，不依赖通信）的 Python 封装。

    模型: J*dω/dt = τ - τ_c*tanh(λ·ω) - b·ω + τ_d（λ=100，dt_sim 通常等于 dt_control）。
    目标函数: 位置跟踪误差用平滑绝对值 sqrt(err²+a)（smooth_eps 参数，很小的正数），
    控制量（R）与力矩变化率（Rd）惩罚保持均方误差。
    每步调用 step(theta, omega, theta_ref)：theta/omega 为当前状态，
    theta_ref 为未来 N 个控制周期的参考（目标）序列（长度 ≥ N），返回第一步
    控制力矩与第一步预测状态。求解不发送任何数据，适合 Python 端仿真闭环。
    """

    def __init__(self, dt_control, dt_sim=None,
                 J=0.024986, tau_c=0.122514, b=0.025648, tau_d=0.0,
                 max_torque=1.0, max_torque_rate=40.0,
                 N=20, Q=5.0, R=0.01, Rd=0.1, max_iter=30,
                 smooth_eps=1e-6):
        if dt_sim is None:
            dt_sim = dt_control
        self._handle = _lib.tcs_mpc_controller_create(
            float(dt_control), float(dt_sim),
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            int(N), Q, R, Rd, int(max_iter), float(smooth_eps),
        )
        if not self._handle:
            raise RuntimeError("tcs_mpc_controller_create() returned NULL")

    def step(self, theta, omega, theta_ref) -> TcsMpcStepResult:
        """求解一步：theta_ref 为未来 N 个控制周期的参考序列（长度 ≥ N）。
        返回 TcsMpcStepResult(torque, theta, omega)。"""
        ref = (c_double * len(theta_ref))(*[float(v) for v in theta_ref])
        return _lib.tcs_mpc_controller_step(
            self._handle, float(theta), float(omega), ref, len(theta_ref))

    def get_last_ref(self):
        """最近一次 step 的参考（目标）序列（N 个）"""
        return self._get_sequence(_lib.tcs_mpc_controller_get_last_ref)

    def get_last_pred(self):
        """最近一次 step 的预测位置序列（N 个，不含当前状态）"""
        return self._get_sequence(_lib.tcs_mpc_controller_get_last_pred)

    def _get_sequence(self, fn):
        buf = (c_double * 4096)()
        n = fn(self._handle, buf, len(buf))
        return list(buf[:n])

    def reset(self):
        """重置求解器内部状态（热启动序列、上次力矩、参考/预测缓存）"""
        _lib.tcs_mpc_controller_reset(self._handle)

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.tcs_mpc_controller_destroy(self._handle)
            self._handle = None


class TcsRobotCommunication:
    """RobotCommunication（C++ 侧 tcs::RobotCommunication）的 Python 接口"""

    def __init__(self):
        self._handle = _lib.tcs_robot_comm_create()
        if not self._handle:
            raise RuntimeError("tcs_robot_comm_create() returned NULL")

    def get_latest_data(self) -> TcsRobotLatestData:
        return _lib.tcs_robot_comm_get_latest_data(self._handle)

    def get_fused_data(self) -> TcsRobotFusedData:
        return _lib.tcs_robot_comm_get_fused_data(self._handle)

    def get_strict_pose(self) -> TcsRobotStrictPose:
        """严格反解数据包（独立输出；始终有效，角度 wrap 到 (-π,π]）"""
        return _lib.tcs_robot_comm_get_strict_pose(self._handle)

    def send_to_mcu(self, packet: TcsMcuSendPacket) -> bool:
        return _lib.tcs_robot_comm_send_to_mcu(self._handle, ctypes.byref(packet))

    def send_to_imu(self, packet: TcsImuSendPacket) -> bool:
        return _lib.tcs_robot_comm_send_to_imu(self._handle, ctypes.byref(packet))

    def create_mpc(self, **kwargs) -> TcsYawMpcController:
        """创建 yaw MPC 求解器（参数见 TcsYawMpcController；返回发送值，不发送）"""
        return TcsYawMpcController(self._handle, **kwargs)

    def create_mcu_mpc(self, **kwargs) -> TcsMcuMpcController:
        """创建实车 MCU 控制封装（后台 100Hz 自动发送，参数见 TcsMcuMpcController）"""
        return TcsMcuMpcController(self._handle, **kwargs)

    def stop(self):
        _lib.tcs_robot_comm_stop(self._handle)

    def close(self):
        if self._handle:
            _lib.tcs_robot_comm_destroy(self._handle)
            self._handle = None

    def __enter__(self): return self
    def __exit__(self, *a): self.stop(); self.close()
    def __del__(self): self.close()

# ── 启动时验证 ctypes 布局 ──
assert ctypes.sizeof(TcsMcuSendPacket) == 28, f"TcsMcuSendPacket size mismatch: {ctypes.sizeof(TcsMcuSendPacket)}"
assert ctypes.sizeof(TcsMcuReceivePacket) == 37, f"TcsMcuReceivePacket size mismatch: {ctypes.sizeof(TcsMcuReceivePacket)}"
assert ctypes.sizeof(TcsImuReceivePacket) == 60, f"TcsImuReceivePacket size mismatch: {ctypes.sizeof(TcsImuReceivePacket)}"
