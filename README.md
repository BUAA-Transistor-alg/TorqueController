AI生成的说明，看了一下基本没有问题，主要用RobotController就行了，C API和python那些是测试用的，基本不用管

# TorqueController — 云台偏航轴力矩控制系统

RoboMaster 云台偏航轴（yaw）力矩控制系统：上位机通过 USB 串口与电控 MCU、独立 IMU 通信，融合高频 IMU 与低频 MCU 数据，用 **MPC（模型预测控制）** 求解力矩指令，并通过 C++ / Python 两种方式驱动实车。

核心：**`RobotController` 一体化封装类** —— 外部程序只需实例化一个类即可完成"通信 + 融合 + MPC 求解 + 后台发送（周期由 `mpc_loop_period` 指定，默认示例 100Hz）"的全部控制闭环。

---

## 1. 模块标识与"作为子模组共存"（pfx = `tcs`）

本仓库可作为**子模组**与其它子模组（例如同源的双级 yaw 仓库 `TorqueControllerForBigSmallYaw`，
pfx = `tcbs`）一起被父工程 `add_subdirectory()` 引入，并在同一进程内同时加载两个 `.so`
而不互相干扰。为此全仓库统一前缀化，约定如下：

| 项目 | 约定 | 本仓库示例 |
|---|---|---|
| CMake target | `tcs_<原名>`，并加 ALIAS `tcs::<原名>` | `tcs_communication` / `tcs::communication`、`tcs_robot_comm_c` / `tcs::robot_comm_c` |
| 产物文件名 | 库使用 `tcs_` 前缀，控制演示与标定程序保留原名 | `libtcs_robot_comm_c.so`、`libtcs_communication.a`、`control_demo`、`pitch_calibration` |
| ctest 名 | `tcs_<原名>` | `tcs_c_api_smoke` |
| C ABI 符号 | `tcs_` 前缀 | `tcs_robot_comm_create`、`tcs_mpc_controller_step` |
| C 类型 / 宏 | `Tcs` 驼峰 / `TCS_` 前缀 | `TcsRobotLatestData_C`、`TCS_ROBOT_CONTROLLER_H` |
| 头文件路径 | 物理位于 `include/tcs/...`，一律 `#include "tcs/..."` | `#include "tcs/RobotController.h"` |
| C++ 命名空间 | 全部在 `namespace tcs` 内（`mcu`/`imu` 成为子命名空间） | `tcs::RobotController`、`tcs::mcu::SendPacket` |
| C API 头 | **例外**：`include/tcs/c_api/RobotCommunicationC.h` 保持合法 C11 + `extern "C"`，不进 C++ 命名空间 | — |
| CMake cache 变量 | `TCS_` 前缀 | `TCS_UDEV_LIBRARY` |
| Python 类名 | `Tcs` 前缀 | `TcsRobotCommunication`、`TcsRobotLatestData` |
| Python 库路径环境变量 | `TCS_LIB` 优先，`TORQUE_CONTROLLER_LIB` 兼容 | — |

父工程侧用法：

```cmake
add_subdirectory(<本仓库路径> ${CMAKE_BINARY_DIR}/tcs EXCLUDE_FROM_ALL)
target_link_libraries(your_target PRIVATE tcs::robot_comm_c)
```

作为子目录时本仓库**不**调用 `project()` / `enable_testing()`，也**不**使用
`add_compile_options` / `include_directories` / 全局 `CMAKE_*` 设置；C++17 通过
`target_compile_features(... cxx_std_17)` 逐 target 指定，include 目录只暴露 `include`
这一层（不再暴露 `include/communication`、`include/mpc` 等二级目录，避免裸路径撞车）。

---

## 2. 构建

依赖：Linux、libudev、Ceres（`find_package(Ceres)`）、g++（C++17）；`test_control` 还需 SDL2、OpenGL、FreeType 开发库和 DejaVu Sans Mono 字体；Python 侧还需 `pygame`、`numpy`、`torch`（仅辨识用）。

```bash
bash ./build.sh            # 等价于 cmake + make -j$(nproc)
```

IMU 绕 Y 轴旋转 180° 安装时，启用安装补偿后构建（在子模组目录执行）：

```bash
cmake -S . -B build -DIS_INFANTRY_2=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

`IS_INFANTRY_2` 默认关闭；恢复原安装方式时配置为 `OFF` 并重新编译。
该选项也可在父工程配置时传入。`ImuCommunication` 在报文通过 CRC 校验后统一转换数据，
`control_demo`、`pitch_calibration`、`test_control` 和 `RobotController` 均使用转换后的数据。
角速度和加速度的 X、Z 分量反向，Y 分量保持原值；ZXY 姿态右乘 `Ry(π)`，
`euler_yaw`、`euler_pitch` 保持原值，`euler_roll` 增加 π 并归一化到 `(-π, π]`。
报文布局保持原值，输出结构体的 `crc32` 保留接收值，仅用于记录原始报文的校验值。

产物（`build/`）：

| 产物 | 说明 |
|---|---|
| `libtcs_robot_comm_c.so` | C API + `tcs::RobotController` 动态库（供 Python ctypes 与外部 cpp 链接） |
| `libtcs_communication.a` | 通信静态库（工具程序用） |
| `control_demo` | 正弦目标控制演示（使用 `tcs::RobotController`） |
| `tcs_test_serial` | 串口通信测试 |
| `test_control` | 只接收 IMU/MCU，实时显示底盘和云台姿态；支持 `--demo` 模拟数据 |
| `pitch_calibration` | pitch 轴标定 |
| `fold_demo` | 接收 MCU 的 `fold` 字段，首次接收及状态变化时显示：`0` 为云台折叠，`1` 为云台升起；运行 `./build/fold_demo`，Ctrl+C 退出 |
| `tcs_test_c_api_smoke` | C ABI / 命名空间自检（ctest 名 `tcs_c_api_smoke`，无硬件可跑） |

---

## 3. `RobotController` 使用方法（核心）

头文件：`include/tcs/RobotController.h`（链接 `libtcs_robot_comm_c.so`；类在 `namespace tcs` 内）。

### 3.1 构造

`RobotController` 构造函数的可选参数 `feedback_delay_s` 是融合反馈整体滞后的估计秒数，
默认 `0.0` 关闭补偿。`control_demo.cpp` 的 `FEEDBACK_DELAY_S` 设置为 `0.010` 秒，
启动时打印当前外推时间；手动比较延迟影响时可分别设置 `0.0`、`0.005`、`0.010` 并重新编译。
比较期间保持目标轨迹、MPC 权重、模型参数和约束相同。

补偿在构造 MPC 初始状态和参考序列前执行恒速外推：
`theta += yaw_rate * feedback_delay_s`，
`imu_yaw += (yaw_rate + chassis_imu_omega) * feedback_delay_s`，角速度保持原值。
关节位置与世界系云台方向使用相同的估计时刻，底盘旋转也纳入参考转换。
该参数依赖融合反馈整体滞后的假设，无法自动测量 MCU/IMU 各自的延迟，
也未建立指令发送到执行的延迟模型。原始接收数据和 `getState().fused` 保留测量语义。
恒速近似没有估计延迟期间的角加速度；已有角速度偏置也会参与外推。
该参数目前通过 C++ 构造接口传入。

构造函数末尾的 `velocity_residual_scale` 控制角速度惩罚，默认 `0.0` 关闭。
每个预测步增加残差 `velocity_residual_scale * omega_pred[k]`，
因此代价增加 `velocity_residual_scale² * omega_pred[k]²`；该项抑制关节角速度，目标速度为零。
`control_demo.cpp` 通过 `VELOCITY_RESIDUAL_SCALE = 1.4` 启用，实际代价权重为 `1.96`，
启动时显示系数与权重。残差总数为 `4*N-1`。

```cpp
#include "tcs/RobotController.h"

// 所有 C++ 类型都在 namespace tcs 内（本仓库源码内部用 tcs:: 全限定名，下同）
// dt_control: 控制周期（秒）；N: MPC 预测步数
// J/tau_c/b/tau_d: 辨识参数（params/1/Identified_parameters.txt，tau_d 通常置 0）
// max_torque/max_torque_rate: 约束（N·m / N·m/s）
// Q/R/Rd/max_iter: MPC 代价权重与迭代上限
//   （目标函数: 位置跟踪误差 = Q·sqrt(err²+smooth_eps)（平滑绝对值），
//    控制量 = R·u²、力矩变化率 = Rd·Δu²（均方误差））
// smooth_eps: 位置误差平滑绝对值常数 a（sqrt(err²+a) 中的 a，很小的正数，
//             默认 1e-6；可省略）
// integral_gain: yaw 力矩积分补偿比例系数（必须传参）
// mpc_loop_period: McuMpcController 后台 loop 周期（秒，必须传参，无默认值；
//                   0.01 = 100Hz）
// mcu_linear_params: MCU 数据线性映射标定参数（默认构造为当前标定值）
// sequence_mode: 是否选择序列模式（默认 false）
tcs::RobotController rc(0.01, 20,
                        0.016541, 0.097297, 0.0321, 0.0,
                        1.0, 40.0, 5.0, 0.01, 0.1, 30,
                        0.01,
                        0.01,
                        tcs::McuDataPreprocessor::LinearParams{});
// 如需调整平滑绝对值常数 a（默认 1e-6）:
// tcs::RobotController rc(..., tcs::McuDataPreprocessor::LinearParams{}, false, 1e-6);
```

构造内部自动完成：建立 MCU/IMU 串口通信与融合滤波器（`tcs::RobotCommunication`）、建立 yaw MPC 封装（`tcs::McuMpcController`，含参考序列 + 求解 + **后台发送线程**，周期 = `mpc_loop_period`，线程已自动启动）。

### 3.2 设置目标（直通 `tcs::McuMpcController::set`）

```cpp
// 参数顺序: auto_aim_enable, yaw_torque_only_mode, target_yaw, pitch_target_angle,
//           fire, integral_enable（必须传参）
// target_yaw 会自动转换到与 imu_yaw_unwrapped 角度差最小的等效角（与 target 同向）
// integral_enable=true: yaw 力矩积分补偿（积分值 += integral_gain * (上一步预测值 −
//   这一步实际角度)，第一次 step 不计算；yaw_torque 加积分后限幅到 ±max_torque）
// integral_enable=false: 积分值清空为 0
rc.set(true,   // auto_aim_enable
       false,  // yaw_torque_only_mode（0=力矩+位置+速度）
       1.57,   // target_yaw (rad，多圈语义)
       0.1,    // pitch_target_angle (rad)
       false,  // fire
       false); // integral_enable（积分补偿开关）
```

每次调用即更新目标，后台发送线程（周期 = `mpc_loop_period`，默认示例 100Hz）持续读取最新目标、求解 MPC 并发送 MCU。

（C ABI 用户注意：C 侧导出的是 `tcs_robot_comm_*` / `tcs_mcu_mpc_*` / `tcs_mpc_controller_*`，
结构体为 `Tcs*_C`，见 `include/tcs/c_api/RobotCommunicationC.h`。）

### ⚠️ 重点：yaw 轴控制延后 dt_control × N

- **yaw 轴**：传入 `set` 的 `target_yaw` 会进入**内部延迟缓冲**（保留最近 N 个目标，即延迟 `dt_control × N` = 0.01 × 20 = **0.2s**）。MPC 的参考序列使用**延迟后的目标**（可通过 `st.mpc.delayed_target` 查看当前实际参考）。因此 **yaw 的实际响应相对输入目标延后约 0.2s**（有意设计：MPC 在预测窗口内跟踪延迟目标，模拟目标前瞻语义）。
- **其他控制不受影响**：`pitch_target_angle`、`auto_aim_enable`、`yaw_torque_only_mode`、`fire` 均为**直接透传**，后台线程立即用最新设置值发送，**无任何延迟**。pitch 等轴的控制实时性不受影响。

若需调整 yaw 延迟时长，改变构造参数 `N`（或 `dt_control`）即可：延迟 = `dt_control × N`。

### 3.3 获取统一状态（按来源分组）

```cpp
auto st = rc.getState();

st.mcu    // MCU 原始数据（经预处理）：yaw_angle 多圈、yaw_omega、pitch_angle、chassis_imu_*、温度等
st.imu    // IMU 原始数据：gx/gy/gz、ax/ay/az、euler_yaw/pitch/roll、dt_one_tenth_ms
st.fused  // 融合滤波器输出：yaw_pos（解卷绕多圈）、yaw_rate（高频）、chassis_yaw/pitch/roll、imu_yaw_unwrapped
st.mpc    // MPC 状态：yaw_target_angle / yaw_target_velocity / yaw_torque / delayed_target
          //   + integral      （当前积分值，积分补偿累积量）
          //   + loop_fps      （McuMpcController 后台 loop 真实帧率，滑动平均，帧/秒）
          //   + ref_sequence  （最新一次运算的目标位置序列，N 个）
          //   + pred_sequence （最新一次运算的预测位置序列，N 个，与 ref 逐点对应）
```

### 3.4 完整示例

```cpp
#include "tcs/RobotController.h"
#include <cmath>
#include <thread>
#include <chrono>

int main() {
    tcs::RobotController rc(0.01, 20, 0.016541, 0.097297, 0.0321, 0.0,
                            1.0, 40.0, 5.0, 0.01, 0.1, 30,
                            0.01, 0.01, tcs::McuDataPreprocessor::LinearParams{});
    // 等待融合就绪
    while (!rc.getState().fused.valid)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    double t = 0;
    while (true) {
        rc.set(true, false, 0.5 * std::sin(t), 0.1, false, false);   // 正弦目标
        auto st = rc.getState();
        // 用 st.mcu / st.imu / st.fused / st.mpc 做任何事
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        t += 0.01;
    }
}
```

更完整的正弦单轴演示见 `src/control_demo.cpp`（其中演示了 `using namespace tcs;` 的写法）。

---

## 4. 其他可运行程序

### 4.1 C++ 程序（`build/`）

| 程序 | 说明 |
|---|---|
| **`control_demo`** | 使用 `tcs::RobotController` 的正弦控制演示：3s 周期正弦控制 `target_yaw`（±30°）与 `pitch_target_angle`（−10°~+20°），两者相位差 90°。Ctrl+C 退出 |
| **`tcs_test_serial`** | 串口通信自检：打印 MCU 回传的 yaw/pitch/温度/底盘 IMU 等字段，验证通信链路 |
| **`test_control`** | 只接收 IMU/MCU 数据，显示融合姿态；圆锥台表示底盘，长方体表示云台，白色线框表示安装补偿后的 IMU 朝向 |
| **`pitch_calibration`** | pitch 轴标定工具：采集 IMU pitch 与 MCU pitch 对应关系，用于生成 `tcs::McuDataPreprocessor` 的线性标定参数 |
| **`tcs_test_c_api_smoke`** | C ABI / 命名空间自检（`ctest -R tcs_c_api_smoke`）：协议布局、CRC 数值、C API 句柄与 MPC 求解器、`tcs::RobotController` 全链路，无硬件可跑 |

运行姿态调试程序：`./build/test_control`。鼠标左键拖动旋转视角，滚轮缩放，`R` 重置视角，`F` 显示融合姿态，`S` 显示严格反解姿态，`Esc` 退出。右侧显示安装补偿后的 IMU、原始 MCU、标定后的 MCU、融合与严格反解数据，以及接收计数和数据年龄；超过 0.5 秒未更新时显示 `STALE`。

模型使用 ZXY 旋转约定，pitch 绕 x 轴，云台朝向为 +Y。`IS_INFANTRY_2=ON` 时，IMU 安装补偿自动应用于融合输入和白色线框。车辆世界位置固定，几何尺寸与关节高度为示意值，集中定义在 `src/test_control.cpp` 开头。程序没有串口发送调用。默认字体路径为 `/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf`，可以通过 `--font PATH` 指定；`--frames N --screenshot PATH.bmp` 可在运行指定帧数后保存窗口截图并退出。

查看模型效果：`./build/test_control --demo`。该模式按照当前安装选项生成模拟 IMU 数据，经过安装补偿与实际融合滤波器后显示，窗口标注 `DEMO`，不打开串口。

独立构建默认启用姿态窗口；在父工程中构建时传入 `-DTCS_BUILD_POSE_VIEWER=ON`，然后执行 `cmake --build build --target tcs_test_control`。依赖包为 `libsdl2-dev`、`libgl-dev`、`libfreetype6-dev`、`fonts-dejavu-core`。

### 4.2 Python 脚本（`python/scripts/`，用 `PYTHONPATH=python python3 python/scripts/xxx.py` 运行）

| 脚本 | 说明 |
|---|---|
| **`pygame_control_mpc.py`** | 实车 MPC 控制：鼠标控制 yaw（MPC，可选 TrajectoryPlanner 平滑，目标延迟在 C++ 内）+ pitch，上方角度视图 + 下方 Angle/Velocity/Torque 波形。点击锁定持续捕获鼠标，TAB 切换相对/绝对模式 |
| **`pygame_control.py`** | 非 MPC 演示：yaw 用 TrajectoryPlanner 平滑轨迹（位置+速度跟踪模式，力矩为 0）+ pitch，结构与上者类似 |
| **`trajectory_viz.py`** | TrajectoryPlanner 轨迹规划器可视化（位置/速度/加速度/jerk 波形） |
| **`collect_sysid_data.py`** | 系统辨识数据采集：按目标轨迹驱动云台，100Hz 精确采样力矩/位置/角速度到 `data/sysid_samples/` |
| **`param_ident.py`** | 系统辨识：PyTorch 可微分仿真 + 梯度优化，拟合 J/τ_c/b/τ_d；终端打印参数，使用 `--output-dir` 保存参数、训练历史和对比图 |

在父工程根目录运行参数辨识：

```bash
python3 sub_module/TorqueController/python/scripts/param_ident.py \
  --data-dir /path/to/samples --initial-params 0.0056 0.006 0.0001 0 \
  --epochs 50000 --patience 1000 --output-dir analysis/sysid_fit --no-show
```

`--initial-params` 依次设置 `J`、`tau_c`、`b`、`tau_d` 的初始值，
默认值为 `0.0056 0.006 0.0001 0`；前三个初始值必须为正。
每轮在全部训练文件的固定 0.1 秒窗口上评估损失；累计改善至少 `--min-delta`
（默认 `1e-6`）时重置等待计数，连续 `--patience` 轮无充分改善时结束训练。
默认最大轮数为 50000，默认 patience 为 1000。最终返回固定窗口损失最低时的参数。
固定窗口来自训练数据。输出目录包含 `best_parameters.json`、`fit_result.json`、
`training_history.npz` 和 `fit_comparison.png`；
省略 `--no-show` 时显示绘图窗口。

### 4.3 Python 底层接口（`python/torque_controller/`）

`TcsRobotCommunication`（ctypes 封装 C API，动态库为 `build/libtcs_robot_comm_c.so`）提供：

```python
from torque_controller import TcsRobotCommunication
r = TcsRobotCommunication()
r.get_latest_data()          # MCU/IMU 原始数据
r.get_fused_data()           # 融合滤波器输出（yaw_pos/yaw_rate/chassis/imu_yaw）
r.send_to_mcu(pkt)           # 手动发送 McuSendPacket
r.create_mpc(**kw)           # yaw MPC 求解器（返回发送值，不发送）
r.create_mcu_mpc(**kw)       # 实车 MPC 控制封装（后台自动发送，周期默认 100Hz）
```

---

## 5. 目录结构

```
include/tcs/         # 唯一对外暴露的 include 根（一律 #include "tcs/..."）
  communication/   # 通信头文件（协议/串口/预处理/融合滤波器）
  mpc/             # MPC 头文件（求解器 / yaw MPC / MCU 控制封装）
  common/          # 通用工具（FrameRateCounter 帧率计数器）
  c_api/           # C 接口头文件（纯 C11 + extern "C"）
  RobotController.h  # 一体化封装类
tests/             # 自检程序（ctest 名带 tcs_ 前缀）
src/
  communication/   # 通信实现
  mpc/             # MPC 实现
  common/          # 通用工具实现
  c_api/           # C 接口实现
  RobotController.cpp
  control_demo.cpp / test_serial.cpp / pitch_calibration.cpp   # 工具程序
python/
  torque_controller/   # Python ctypes 封装
  scripts/             # 控制/采集/辨识/可视化脚本
mpc/（历史目录已并入 src/mpc + include/tcs/mpc）
```

---

## 6. 数据流

```
MCU(低频) ──┐
            ├─► YawChassisFusion（IMU 高频积分 + MCU 校正）──► fused(yaw_pos/yaw_rate/chassis/imu_yaw)
IMU(高频) ──┘
                                          │
   target_yaw ──► tcs::RobotController::set ──► tcs::McuMpcController(后台loop,周期=mpc_loop_period)
                                          │  参考序列(延迟dt*N + 底盘修正) ──► MPC 求解
                                          ▼
                                   McuSendPacket ──► MCU 执行
```

## 7. 注意事项

- **yaw 轴控制延后 `dt_control × N`（默认 0.2s）**：`set` 的 `target_yaw` 经内部延迟缓冲后才进入 MPC 参考序列；`pitch_target_angle` 等其余发送参数直接透传、无延迟（详见 2.2 节重点说明）
- `target_yaw` 为多圈连续角度（非 wrap 到 [−π,π]），`set` 内自动卷绕到与 `imu_yaw_unwrapped` 差最小的等效角
- 融合滤波器当前假设底盘与地面平行（`chassis_imu_yaw/omega` 直接利用）
- 实机使用前建议先运行 `tcs_test_serial` 确认通信，再运行 `control_demo` 验证控制，最后使用 `pygame_control_mpc.py` 交互控制。
- 辨识参数修改后需同步更新各程序的 J/τ_c/b/τ_d（`pygame_control_mpc.py`、`control_demo.cpp`、`tcs::RobotController` 构造参数）
- 所有 C++ 类型都在 `namespace tcs` 内（`tcs::RobotController`、`tcs::YawChassisFusion`、
  `tcs::mcu::SendPacket` …），C ABI 符号带 `tcs_` 前缀、C 类型带 `Tcs` 前缀——与子模组
  `tcbs` 同进程共存时不会串符号。父工程若原先写的是 `RobotController::State`，
  现应改为 `tcs::RobotController::State`
