"""
pygame_control_mpc_sim.py — pygame 鼠标控制 yaw 的 MPC 仿真程序（不连接实车）

- 与 pygame_control_mpc.py 参数一致、目标输入方式一致（但只取 yaw 轴）：
  鼠标水平移动控制 yaw 目标（点击锁定持续捕获；TAB 切换相对/绝对模式）。
- 不输出到实车：使用一个与辨识参数完全相同的仿真环境（Python 端实现，
  显式欧拉 + tanh 软符号摩擦力模型，与 C++ MPCController 内部动力学一致），
  每帧调用 C API 直接暴露的 MPCController 求解器（librobot_comm_c.so）
  求解第一步控制力矩，施加到仿真环境推进一个控制周期（dt=0.01s）。
- 目标延迟 dt*N（默认 0.2s）与参考序列构造语义与实车 C++ 逻辑一致
  （无底盘 IMU 修正项：theta_imu == theta，chassis_imu_omega == 0）。
- 绘制：上方 yaw 角度视图 + 右上 MPC 预测时域图（参考 vs 预测），
  下方 Angle / Angular Velocity / Torque 滚动波形。

运行:
    PYTHONPATH=python python3 python/scripts/pygame_control_mpc_sim.py
    （可选 --frames N：运行 N 帧后自动退出，供无头测试）
"""

import sys, os, math, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from torque_controller import MPCController

import pygame
from pygame.locals import *
from collections import deque

# ============================================================================
# 配置（与 pygame_control_mpc.py 一致，仅 yaw 轴）
# ============================================================================
WINDOW_W, WINDOW_H = 1920, 1080
DT_CTRL = 0.01          # 控制周期 (100 Hz)
DT_S = DT_CTRL
FPS = 100

LINE_LEN = 200
CIRCLE_R = 10
CIRCLE_Y = 200
YAW_SENS = 0.003        # 鼠标水平每像素 → yaw rad

# yaw 目标平滑（与 pygame_control_mpc.py 相同，默认关闭）
USE_PLANNER = False
MAX_VEL   = 30.0        # rad/s
MAX_ACCEL = 50.0        # rad/s²
MAX_JERK  = 2000.0      # rad/s³
REFINE_N  = 1000

# 目标延迟（与实车一致：C++ 内部维护延迟 dt*N 步的参考序列）
DELAY_TIME = 0.2        # 秒
MPC_PRED_N = int(DELAY_TIME / DT_CTRL)

# ================== 辨识参数 (params/1/Identified_parameters.txt) ==================
J       = 0.024986
TAU_C   = 0.122514
B_FRIC  = 0.025648
TAU_D   = 0.0            # tau_d 设为 0

# ================== 约束 ==================
MAX_TORQUE      = 1.0    # 最大力矩 (N·m)
MAX_TORQUE_RATE = 40.0   # 最大力矩变化率 (N·m/s)

# ================== 波形图参数（与 pygame_control_mpc.py 一致）==================
WAVE_LEFT   = 80
WAVE_RIGHT  = 1840
WAVE_WIDTH  = WAVE_RIGHT - WAVE_LEFT
HISTORY_S   = 3.0
HISTORY_N   = int(HISTORY_S * FPS)

RED    = (255, 60, 60)
BLUE   = (60, 120, 255)
WHITE  = (240, 240, 240)
BLACK  = (20, 20, 20)
GRAY   = (80, 80, 80)
DGRAY  = (40, 40, 40)
GREEN  = (100, 200, 100)
YELLOW = (220, 220, 60)
ORANGE = (255, 160, 40)
LGRAY  = (65, 65, 65)

TWO_PI = 2.0 * math.pi

# 绝对模式控制框（与 pygame_control_mpc.py 一致，只保留 yaw 行）
BOX_W, BOX_H = 900, 120
BOX_X = (WINDOW_W - BOX_W) // 2
BOX_Y = 460


def busy_wait_until(target_s):
    """忙等待到绝对时间点"""
    while time.perf_counter_ns() < target_s * 1e9:
        pass


# ============================================================================
# 仿真环境（Python 端，与 C++ MPCController 内部动力学完全一致）
# ============================================================================
def sim_step(theta, omega, tau, dt=DT_CTRL):
    """显式欧拉推进一个 dt：
        J*dω/dt = τ - τ_c*tanh(λ·ω) - b·ω + τ_d，λ=100
    （与 C++ MPCController::dynamicsStep 相同，不对速度做截断）"""
    tau_f = -math.tanh(100.0 * omega) * TAU_C - B_FRIC * omega
    tau_net = tau + tau_f + TAU_D
    alpha = tau_net / J
    omega_new = omega + alpha * dt
    theta_new = theta + omega_new * dt
    return theta_new, omega_new


def wrap_target_near(target, theta):
    """将 target 转换到与当前 theta 角度差最小的等效角（≡ target mod 2π），
    与实车 McuMpcController::set 的 target_adj = theta + remainder(target-theta, 2π) 一致"""
    return theta + math.remainder(target - theta, TWO_PI)


# ============================================================================
# 滚动波形缓冲区（复制自 pygame_control_mpc.py）
# ============================================================================
class RingBuffer:
    def __init__(self, capacity, fill=0.0):
        self._buf = deque([float(fill)] * capacity, maxlen=capacity)

    def push(self, value):
        self._buf.append(float(value))

    def data(self):
        return list(self._buf)


# ============================================================================
# 绘图（复制自 pygame_control_mpc.py）
# ============================================================================
def draw_angle_view(surf, cx, cy, target_angle, current_angle, title, scale=1.0, rotate_ccw90=False):
    """以 (cx,cy) 为圆心画两条射线表示目标和当前角度"""
    t_ang = target_angle * scale
    c_ang = current_angle * scale
    if rotate_ccw90:
        ex = cx - LINE_LEN * math.sin(t_ang)
        ey = cy - LINE_LEN * math.cos(t_ang)
    else:
        ex = cx + LINE_LEN * math.cos(t_ang)
        ey = cy - LINE_LEN * math.sin(t_ang)
    pygame.draw.line(surf, RED, (cx, cy), (int(ex), int(ey)), 3)
    if rotate_ccw90:
        ex = cx - LINE_LEN * math.sin(c_ang)
        ey = cy - LINE_LEN * math.cos(c_ang)
    else:
        ex = cx + LINE_LEN * math.cos(c_ang)
        ey = cy - LINE_LEN * math.sin(c_ang)
    pygame.draw.line(surf, BLUE, (cx, cy), (int(ex), int(ey)), 3)
    pygame.draw.circle(surf, WHITE, (cx, cy), CIRCLE_R)
    font = pygame.font.SysFont(None, 22)
    s = font.render(title, True, WHITE)
    surf.blit(s, (cx - s.get_width() // 2, cy + LINE_LEN + 10))


def draw_info(surf, target_yaw, sim_theta, sim_omega, mpc_res,
              delayed_target, ref_seq, pred_seq, fps, mode):
    font = pygame.font.SysFont(None, 20)
    y = 5
    def t(txt):
        nonlocal y
        s = font.render(txt, True, WHITE)
        surf.blit(s, (5, y)); y += 22

    mode_names = {0: "RELATIVE (click lock)", 1: "ABSOLUTE (box)"}
    t(f"FPS: {fps:.0f}  |  Mode: {mode_names.get(mode, '?')}  |  [TAB:switch ESC:quit R:reset]  (SIM, no robot)")
    t(f"Target  Yaw: {target_yaw:7.3f} rad")
    t(f"MPC     Yaw target_angle: {mpc_res.theta:7.3f}  velocity: {mpc_res.omega:+.3f}  torque: {mpc_res.torque:+.4f}")
    t(f"Sim     Yaw pos: {sim_theta:7.3f}  rate: {sim_omega:+.3f}")
    t(f"MPC     delayed target: {delayed_target:7.3f}  (N={len(ref_seq)}, delay={DELAY_TIME:.2f}s)")
    if ref_seq and pred_seq:
        t(f"MPC     pred head: {pred_seq[0]:7.3f}  pred tail: {pred_seq[-1]:7.3f}  "
          f"ref tail: {ref_seq[-1]:7.3f}")


def draw_control_box(surf, mx, my):
    """绘制绝对模式控制框和十字准星（复制自 pygame_control_mpc.py，只显示 yaw 行）"""
    r = pygame.Rect(BOX_X, BOX_Y, BOX_W, BOX_H)
    pygame.draw.rect(surf, GRAY, r, 1)
    cx = BOX_X + BOX_W // 2; cy = BOX_Y + BOX_H // 2
    pygame.draw.line(surf, DGRAY, (BOX_X, cy), (BOX_X + BOX_W, cy), 1)
    pygame.draw.line(surf, DGRAY, (cx, BOX_Y), (cx, BOX_Y + BOX_H), 1)
    font = pygame.font.SysFont(None, 18)
    s = font.render("yaw: L=+pi  R=-pi", True, GRAY)
    surf.blit(s, (BOX_X, BOX_Y - 18))
    cx_clamp = max(BOX_X, min(BOX_X + BOX_W, mx))
    cy_clamp = max(BOX_Y, min(BOX_Y + BOX_H, my))
    pygame.draw.line(surf, YELLOW, (cx_clamp - 10, cy_clamp), (cx_clamp + 10, cy_clamp), 2)
    pygame.draw.line(surf, YELLOW, (cx_clamp, cy_clamp - 10), (cx_clamp, cy_clamp + 10), 2)


def draw_waveform(surf, x, y, w, h, title, buf, y_label,
                  line_color, y_min=None, y_max=None, extra_traces=None):
    """滚动波形图，支持多条曲线叠加（复制自 pygame_control_mpc.py）"""
    rect = pygame.Rect(x, y, w, h)
    pygame.draw.rect(surf, DGRAY, rect, 1)
    pygame.draw.rect(surf, BLACK, rect.inflate(-2, -2))

    font = pygame.font.SysFont(None, 16)
    s = font.render(title, True, WHITE)
    surf.blit(s, (x + 4, y + 2))

    legend_x = x + s.get_width() + 10
    traces = [(buf, line_color)] + (extra_traces or [])
    for _, c in traces:
        pygame.draw.rect(surf, c, (legend_x, y + 5, 10, 10))
        legend_x += 14
    s = font.render(y_label, True, GRAY)
    surf.blit(s, (x + w - s.get_width() - 4, y + 2))

    if y_min is not None and y_max is not None:
        y_min, y_max = float(y_min), float(y_max)
    else:
        all_data = []
        for b, _ in traces:
            all_data.extend(b.data())
        if len(all_data) < 2:
            return
        y_min, y_max = min(all_data), max(all_data)
        if y_max - y_min < 1e-9:
            y_min -= 0.5; y_max += 0.5
        else:
            m = (y_max - y_min) * 0.1
            y_min -= m; y_max += m

    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        gy = int(y + h - 4 - frac * (h - 28))
        pygame.draw.line(surf, LGRAY, (x + 2, gy), (x + w - 2, gy), 1)
        val = y_min + frac * (y_max - y_min)
        sv = font.render(f"{val:.1f}", True, GRAY)
        surf.blit(sv, (x + w - sv.get_width() - 4, gy - sv.get_height()))

    for b, c in traces:
        data = b.data()
        n = len(data)
        if n < 2:
            continue
        points = []
        for i, val in enumerate(data):
            frac_x = i / max(n - 1, 1)
            frac_y = (val - y_min) / max(y_max - y_min, 1e-9)
            px = int(x + 6 + frac_x * (w - 14))
            py = int(y + h - 8 - frac_y * (h - 32))
            points.append((px, py))
        if len(points) >= 2:
            pygame.draw.lines(surf, c, False, points, 2)


def draw_prediction_horizon(surf, x, y, w, h, title, ref_seq, pred_seq):
    """MPC 预测时域图：横轴为未来控制步 0..N-1，绘制参考（目标）序列与预测位置序列"""
    rect = pygame.Rect(x, y, w, h)
    pygame.draw.rect(surf, DGRAY, rect, 1)
    pygame.draw.rect(surf, BLACK, rect.inflate(-2, -2))

    font = pygame.font.SysFont(None, 16)
    s = font.render(title, True, WHITE)
    surf.blit(s, (x + 4, y + 2))
    # 图例
    legend_x = x + s.get_width() + 10
    for label, c in (("ref", RED), ("pred", GREEN)):
        pygame.draw.rect(surf, c, (legend_x, y + 5, 10, 10))
        legend_x += 16
        sv = font.render(label, True, c)
        surf.blit(sv, (legend_x, y + 2))
        legend_x += sv.get_width() + 10

    n = min(len(ref_seq), len(pred_seq))
    if n < 2:
        return

    all_v = list(ref_seq[:n]) + list(pred_seq[:n])
    y_min, y_max = min(all_v), max(all_v)
    if y_max - y_min < 1e-9:
        y_min -= 0.5; y_max += 0.5
    else:
        m = (y_max - y_min) * 0.1
        y_min -= m; y_max += m

    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        gy = int(y + h - 4 - frac * (h - 28))
        pygame.draw.line(surf, LGRAY, (x + 2, gy), (x + w - 2, gy), 1)
        val = y_min + frac * (y_max - y_min)
        sv = font.render(f"{val:.2f}", True, GRAY)
        surf.blit(sv, (x + w - sv.get_width() - 4, gy - sv.get_height()))

    def to_points(seq):
        pts = []
        for i, val in enumerate(seq[:n]):
            frac_x = i / max(n - 1, 1)
            frac_y = (val - y_min) / max(y_max - y_min, 1e-9)
            pts.append((int(x + 6 + frac_x * (w - 14)),
                        int(y + h - 8 - frac_y * (h - 32))))
        return pts

    pts_ref = to_points(ref_seq)
    pts_pred = to_points(pred_seq)
    if len(pts_ref) >= 2:
        pygame.draw.lines(surf, RED, False, pts_ref, 1)
    if len(pts_pred) >= 2:
        pygame.draw.lines(surf, GREEN, False, pts_pred, 2)
    # 端点标记
    if pts_ref:
        pygame.draw.circle(surf, RED, pts_ref[-1], 3)
    if pts_pred:
        pygame.draw.circle(surf, GREEN, pts_pred[-1], 4)


# ============================================================================
# 主函数
# ============================================================================
def main():
    # 可选 --frames N：运行 N 帧后自动退出（无头测试用）
    frames_limit = None
    if "--frames" in sys.argv:
        try:
            frames_limit = int(sys.argv[sys.argv.index("--frames") + 1])
        except (ValueError, IndexError):
            frames_limit = None

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Yaw MPC Simulation (no robot)")

    # ── MPC 求解器（C API 直接暴露的 MPCController，参数与实车一致）──
    mpc = MPCController(
        dt_control=DT_CTRL, dt_sim=DT_CTRL, N=MPC_PRED_N,
        J=J, tau_c=TAU_C, b=B_FRIC, tau_d=TAU_D,
        max_torque=MAX_TORQUE, max_torque_rate=MAX_TORQUE_RATE,
        Q=5.0, R=0.01, Rd=0.1, max_iter=30,
    )

    # ── 可选：yaw 目标平滑（与 pygame_control_mpc.py 相同的 TrajectoryPlanner）──
    planner = None
    yaw_pos_p, yaw_vel_p, yaw_acc_p = 0.0, 0.0, 0.0
    if USE_PLANNER:
        from scripts.trajectory_planner import TrajectoryPlanner, StepRefinementWrapper
        planner = TrajectoryPlanner(max_velocity=MAX_VEL, max_acceleration=MAX_ACCEL, max_jerk=MAX_JERK)
        planner = StepRefinementWrapper(planner.step, REFINE_N)

    # ── 仿真状态（初始静止）──
    theta = 0.0
    omega = 0.0

    # ── 目标（相对当前 yaw，连续多圈；延迟参考序列在本脚本内维护）──
    target_yaw = 0.0
    target_buf = deque(maxlen=MPC_PRED_N)   # 延迟目标缓冲（最多 N 个，最旧在前）

    # ── 波形历史缓冲区 ──
    hist_delayed_target = RingBuffer(HISTORY_N, 0.0)
    hist_cur_yaw        = RingBuffer(HISTORY_N, 0.0)
    hist_vel_sent       = RingBuffer(HISTORY_N, 0.0)
    hist_omega          = RingBuffer(HISTORY_N, 0.0)
    hist_torque         = RingBuffer(HISTORY_N, 0.0)

    VEL_Y_RANGE   = (-MAX_VEL * 1.2, MAX_VEL * 1.2)
    TORQUE_Y_RANGE = (-2.0, 2.0)

    running = True
    mode = 0                     # 0=相对(点击锁定持续捕获)  1=绝对(框)
    total_time = 0.0
    loop_start_s = time.perf_counter_ns() * 1e-9
    already_exceeded_time = 0.0
    frame = 0

    while running:
        frame += 1
        total_time = time.perf_counter_ns() * 1e-9 - loop_start_s
        delay_target_s = loop_start_s + frame * DT_S + already_exceeded_time
        if time.perf_counter_ns() * 1e-9 < delay_target_s:
            busy_wait_until(delay_target_s)
        else:
            already_exceeded_time = time.perf_counter_ns() * 1e-9 - (loop_start_s + frame * DT_S)

        mx, my = pygame.mouse.get_pos()

        # ── 事件处理（与 pygame_control_mpc.py 一致：点击锁定持续捕获，TAB 切换模式）──
        for evt in pygame.event.get():
            if evt.type == QUIT:
                running = False
            elif evt.type == KEYDOWN:
                if evt.key == K_ESCAPE:
                    running = False
                elif evt.key == K_TAB:
                    if mode == 0:
                        mode = 1
                        pygame.mouse.set_visible(True)
                        pygame.event.set_grab(False)
                    else:
                        mode = 0
                elif evt.key == K_r:
                    target_yaw = theta
            elif evt.type == MOUSEBUTTONDOWN and mode == 0:
                pygame.mouse.set_visible(False)
                pygame.event.set_grab(True)

        # ── 目标角度（与 pygame_control_mpc.py 一致，只取 yaw 轴）──
        if mode == 0:
            if pygame.event.get_grab():
                dx, dy = pygame.mouse.get_rel()
                target_yaw -= float(dx) * YAW_SENS
        else:
            fx = (mx - BOX_X) / BOX_W
            fx = max(0.0, min(1.0, fx))
            target_yaw = (1.0 - fx) * TWO_PI - math.pi

        # ── 可选：yaw 目标平滑（TrajectoryPlanner）──
        mpc_target_yaw = target_yaw
        if planner is not None:
            yaw_pos_p, yaw_vel_p, yaw_acc_p, _ = planner.step(
                target_yaw, yaw_pos_p, yaw_vel_p, yaw_acc_p, DT_CTRL)
            mpc_target_yaw = yaw_pos_p

        # ── 目标卷绕到当前仿真角度附近（与实车 McuMpcController::set 一致）──
        target_adj = wrap_target_near(mpc_target_yaw, theta)
        target_buf.append(target_adj)

        # ── 延迟参考序列（与实车 C++ 一致：ref[i] = 延迟目标[i]，无底盘修正）──
        ref = list(target_buf)
        while len(ref) < MPC_PRED_N:
            ref.append(ref[-1] if ref else target_adj)
        delayed_target = ref[0]

        # ── MPC 求解（C API 直接暴露的 MPCController）──
        res = mpc.step(theta, omega, ref)
        ref_seq = mpc.get_last_ref()
        pred_seq = mpc.get_last_pred()

        # ── 仿真推进一个控制周期（施加第一步控制力矩）──
        theta, omega = sim_step(theta, omega, res.torque)

        # ── 波形历史（目标角度 = 延迟后的输入目标；目标速度 = mpc 解算值）──
        hist_delayed_target.push(delayed_target)
        hist_cur_yaw.push(theta)
        hist_vel_sent.push(res.omega)
        hist_omega.push(omega)
        hist_torque.push(res.torque)

        # ── 绘制 ──
        screen.fill(BLACK)
        mid = WINDOW_W // 2
        draw_angle_view(screen, mid // 2, CIRCLE_Y,
                        delayed_target, theta,
                        "Yaw  (R=delayed target / B=sim current)",
                        rotate_ccw90=True)

        if mode == 1:
            draw_control_box(screen, mx, my)

        # 右上：MPC 预测时域（参考 vs 预测，N 步）
        draw_prediction_horizon(screen, 900, 30, 980, 380,
                                f"MPC Prediction Horizon (N={MPC_PRED_N})",
                                ref_seq, pred_seq)

        draw_info(screen, target_yaw, theta, omega, res, delayed_target,
                  ref_seq, pred_seq,
                  int(1.0 / DT_CTRL) if total_time > 0.5 else 0, mode)

        wave_start_y = 620
        pygame.draw.line(screen, DGRAY, (0, wave_start_y), (WINDOW_W, wave_start_y), 2)

        wave_h = 135
        wave_gap = 12
        y0 = wave_start_y + 6
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Angle (delayed target + sim)", hist_delayed_target, "rad",
                      RED, extra_traces=[(hist_cur_yaw, BLUE)])
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Angular Velocity (mpc target + sim)", hist_vel_sent, "rad/s",
                      GREEN, *VEL_Y_RANGE, extra_traces=[(hist_omega, ORANGE)])
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Torque (mpc)", hist_torque, "N\u00b7m",
                      YELLOW, *TORQUE_Y_RANGE)

        pygame.display.flip()

        if frames_limit is not None and frame >= frames_limit:
            running = False

    # 清理
    if pygame.event.get_grab():
        pygame.event.set_grab(False)
    pygame.mouse.set_visible(True)
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
