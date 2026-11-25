"""
param_ident.py — 系统辨识（基于 data/sysid_samples/*.npz）

模型: J * dω/dt = τ - τ_c * sign(ω) - b * ω + τ_d
使用 PyTorch 可微分仿真 + tanh 软符号函数进行梯度优化
"""

import argparse
import json
import random
import torch, torch.nn as nn
import numpy as np, os, sys, math, matplotlib.pyplot as plt

seed = 42
random.seed(seed)
np.random.seed(seed)

SAVE_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data", "sysid_samples")
DT = 0.01
SEG_STEPS = 10  # 每次拟合只截取 0.1s（10 步）的片段
INITIAL_PARAMS = (0.0056, 0.006, 0.0001, 0.0)  # J、tau_c、b、tau_d 的初始值

# ============================================================================
# 可微分仿真环境
# ============================================================================
class DifferentiableYawSimEnv:
    def __init__(self, dt=DT):
        self.dt = dt

    def set_params(self, J, tau_c, b, tau_d):
        self.J, self.tau_c, self.b, self.tau_d = J, tau_c, b, tau_d

    def simulate(self, tau_seq, theta_init, omega_init):
        N = tau_seq.shape[0]
        theta = theta_init.clone().detach()
        omega = omega_init.clone().detach()
        theta_list = []
        omega_list = []
        lam = 100.0
        for i in range(N):
            soft_sign = torch.tanh(lam * omega)
            tau_f = -soft_sign * self.tau_c - self.b * omega
            tau_net = tau_seq[i] + tau_f + self.tau_d
            alpha = tau_net / self.J
            omega = omega + alpha * self.dt
            theta = theta + omega * self.dt
            theta_list.append(theta)
            omega_list.append(omega)
        return torch.stack(theta_list), torch.stack(omega_list)

# ============================================================================
# 数据加载
# ============================================================================
def load_samples(data_dir=SAVE_DIR):
    files = sorted(f for f in os.listdir(data_dir) if f.endswith('.npz'))
    if not files:
        raise ValueError(f"没有采样文件：{data_dir}")
    samples = []
    for f in files:
        with np.load(os.path.join(data_dir, f), allow_pickle=False) as d:
            sample = {
                'target': d['target'].copy(), 'torque': d['torque'].copy(),
                'yaw': np.unwrap(d['yaw']), 'gz': d['gz'].copy(),
            }
        length = len(sample['torque'])
        for key, values in sample.items():
            if values.ndim != 1 or len(values) != length or not np.isfinite(values).all():
                raise ValueError(f"采样字段的长度、维度或数值错误：{f}/{key}")
        samples.append(sample)
    print(f"Loaded {len(samples)} samples")
    return samples

# ============================================================================
# 优化
# ============================================================================
def fixed_evaluation_windows(samples, device):
    initial_yaw, initial_gz, inputs, yaw_values, gz_values = [], [], [], [], []
    for sample in samples:
        if len(sample['torque']) < SEG_STEPS:
            raise ValueError("采样长度不足以构造固定评估窗口")
        for start in range(0, len(sample['torque']) - SEG_STEPS + 1, SEG_STEPS):
            initial_yaw.append(sample['yaw'][start])
            initial_gz.append(sample['gz'][start])
            inputs.append(sample['torque'][start:start + SEG_STEPS])
            yaw_values.append(sample['yaw'][start:start + SEG_STEPS])
            gz_values.append(sample['gz'][start:start + SEG_STEPS])
    return (
        torch.tensor(initial_yaw, dtype=torch.float32, device=device),
        torch.tensor(initial_gz, dtype=torch.float32, device=device),
        torch.tensor(np.asarray(inputs).T, dtype=torch.float32, device=device),
        torch.tensor(np.asarray(yaw_values).T, dtype=torch.float32, device=device),
        torch.tensor(np.asarray(gz_values).T, dtype=torch.float32, device=device),
    )


def optimize(samples, num_epochs=50000, lr=1e-3, device='cpu', initial_params=INITIAL_PARAMS,
             patience=1000, min_delta=1e-6, checkpoint_path=None):
    if num_epochs <= 0 or patience <= 0:
        raise ValueError("训练轮数和 patience 必须为正")
    if not math.isfinite(min_delta) or min_delta < 0:
        raise ValueError("min_delta 必须为非负有限值")
    if len(initial_params) != 4 or not all(math.isfinite(v) for v in initial_params):
        raise ValueError("初始参数必须为四个有限数值")
    if any(v <= 0 for v in initial_params[:3]):
        raise ValueError("J、tau_c、b 的初始值必须为正")
    diff_env = DifferentiableYawSimEnv(dt=DT)

    log_J = nn.Parameter(torch.tensor(math.log(initial_params[0]), device=device))
    log_tau_c = nn.Parameter(torch.tensor(math.log(initial_params[1]), device=device))
    log_b = nn.Parameter(torch.tensor(math.log(initial_params[2]), device=device))
    tau_d = nn.Parameter(torch.tensor(initial_params[3], device=device))

    def get_params():
        return torch.exp(log_J), torch.exp(log_tau_c), torch.exp(log_b), tau_d

    opt = torch.optim.Adam([log_J, log_tau_c, log_b, tau_d], lr=lr)

    evaluation = fixed_evaluation_windows(samples, device)

    def evaluate():
        with torch.no_grad():
            diff_env.set_params(*get_params())
            theta, omega = diff_env.simulate(evaluation[2], evaluation[0], evaluation[1])
            delta = theta - evaluation[3]
            delta = torch.atan2(torch.sin(delta), torch.cos(delta))
            loss = torch.mean(delta ** 2) + torch.mean((omega - evaluation[4]) ** 2)
            if not torch.isfinite(loss):
                raise FloatingPointError("固定窗口评估损失非有限值")
            return loss.item()

    best_loss = evaluate()
    initial_evaluation_loss = best_loss
    best_params = [p.item() for p in get_params()]
    best_epoch = 0
    patience_loss = best_loss
    epochs_without_improvement = 0

    def save_best():
        if checkpoint_path:
            with open(checkpoint_path, 'w', encoding='utf-8') as stream:
                json.dump({'epoch': best_epoch, 'evaluation_loss': best_loss,
                           'parameters': dict(zip(['J', 'tau_c', 'b', 'tau_d'], best_params))},
                          stream, indent=2, ensure_ascii=False)

    save_best()

    loss_history, param_history, evaluation_history = [], [], []
    stopping_reason = 'max_epochs'
    for epoch in range(num_epochs):
        epoch_loss = 0.0
        n_seq = 0

        for sample_idx, (sample) in enumerate(samples):
            torque = sample['torque']
            L = len(torque)
            if L < SEG_STEPS:
                raise ValueError(f"采样长度不足：sample={sample_idx}")
            gz = sample['gz']
            yaw = sample['yaw']

            # 每次随机截取 0.1s（10 步）的同索引输入和实测片段。
            start = random.randint(0, L - SEG_STEPS)
            torque_seg = torque[start:start + SEG_STEPS]
            gz_seg = gz[start:start + SEG_STEPS]
            yaw_seg = yaw[start:start + SEG_STEPS]

            tau_t = torch.tensor(torque_seg, dtype=torch.float32, device=device)
            theta_init = torch.tensor(float(yaw[start]), dtype=torch.float32, device=device)
            omega_init = torch.tensor(float(gz[start]), dtype=torch.float32, device=device)
            theta_true = torch.tensor(yaw_seg, dtype=torch.float32, device=device)
            omega_true = torch.tensor(gz_seg, dtype=torch.float32, device=device)

            J, tc, b, td = get_params()
            diff_env.set_params(J, tc, b, td)
            theta_sim, omega_sim = diff_env.simulate(tau_t, theta_init, omega_init)

            # 每模拟一个片段就计算一个片段的损失（位置 + 速度）
            # 角度误差先归一化到 (-π, π] 再平方，避免 ±π 处绕卷跳变
            delta = theta_sim - theta_true
            delta = torch.atan2(torch.sin(delta), torch.cos(delta))
            pos_loss = torch.mean(delta ** 2)
            vel_loss = torch.mean((omega_sim - omega_true) ** 2)
            loss = pos_loss + vel_loss
            if not torch.isfinite(loss):
                raise FloatingPointError(f"拟合损失非有限值：epoch={epoch}, sample={sample_idx}")

            opt.zero_grad()
            loss.backward()
            opt.step()

            J_now, tc_now, b_now, td_now = get_params()

            epoch_loss += loss.item()
            n_seq += 1

        if n_seq == 0:
            continue
        epoch_loss /= n_seq

        loss_history.append(epoch_loss)
        J, tc, b, td = get_params()
        param_history.append([J.item(), tc.item(), b.item(), td.item()])
        evaluation_loss = evaluate()
        evaluation_history.append(evaluation_loss)
        if evaluation_loss < best_loss:
            best_loss = evaluation_loss
            best_params = param_history[-1].copy()
            best_epoch = epoch + 1
            save_best()
        # 固定窗口避免随机抽样波动影响 patience；累计改善达到阈值时重新计数。
        if evaluation_loss < patience_loss - min_delta:
            patience_loss = evaluation_loss
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1
        print(f"Epoch {epoch + 1:5d} loss={epoch_loss:.6f} eval={evaluation_loss:.6f} "
              f"best={best_loss:.6f} wait={epochs_without_improvement}/{patience} "
              f"J={J.item():.6g} tau_c={tc.item():.6g} "
              f"b={b.item():.6g} tau_d={td.item():.6g}")
        if epochs_without_improvement >= patience:
            stopping_reason = 'patience'
            print(f"Early stopping: {patience} epochs without sufficient improvement")
            break

    statistics = {
        'epochs_completed': len(loss_history), 'best_epoch': best_epoch,
        'initial_evaluation_loss': initial_evaluation_loss,
        'best_evaluation_loss': best_loss, 'evaluation_history': evaluation_history,
        'evaluation_window_count': evaluation[0].numel(),
        'patience': patience, 'min_delta': min_delta,
        'epochs_without_improvement': epochs_without_improvement,
        'stopping_reason': stopping_reason,
    }
    return best_params, loss_history, param_history, statistics

# ============================================================================
# 主流程
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="yaw 动力学参数辨识")
    parser.add_argument('--data-dir', default=SAVE_DIR, help="采样 NPZ 文件目录")
    parser.add_argument('--initial-params', type=float, nargs=4, default=INITIAL_PARAMS,
                        metavar=('J', 'TAU_C', 'B', 'TAU_D'), help="四个动力学参数的初始值")
    parser.add_argument('--epochs', type=int, default=50000, help="最大训练轮数")
    parser.add_argument('--patience', type=int, default=1000, help="固定窗口评估损失无充分改善时的等待轮数")
    parser.add_argument('--min-delta', type=float, default=1e-6, help="重置 patience 所需的最小累计改善")
    parser.add_argument('--output-dir', help="保存拟合参数、训练历史和对比图的目录")
    parser.add_argument('--no-show', action='store_true', help="不打开绘图窗口")
    args = parser.parse_args()
    if args.epochs <= 0 or args.patience <= 0:
        parser.error("训练轮数和 patience 必须为正")
    if not math.isfinite(args.min_delta) or args.min_delta < 0:
        parser.error("min_delta 必须为非负有限值")
    if not all(math.isfinite(v) for v in args.initial_params) or any(v <= 0 for v in args.initial_params[:3]):
        parser.error("初始参数必须为有限值，且 J、tau_c、b 必须为正")
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Device: {device}")
    print(f"Initial parameters (J, tau_c, b, tau_d): {args.initial_params}")

    samples = load_samples(args.data_dir)
    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
    checkpoint_path = os.path.join(args.output_dir, 'best_parameters.json') if args.output_dir else None

    print("Optimizing...")
    final_params, loss_history, param_history, statistics = optimize(
        samples, num_epochs=args.epochs, lr=3e-4, device=device,
        initial_params=args.initial_params, patience=args.patience,
        min_delta=args.min_delta, checkpoint_path=checkpoint_path)

    param_names = ['J', 'tau_c', 'b', 'tau_d']
    print("\n" + "=" * 50)
    print("Identified parameters:")
    print(f"Best epoch: {statistics['best_epoch']}, evaluation loss: {statistics['best_evaluation_loss']:.8f}")
    for name, val in zip(param_names, final_params):
        print(f"  {name} = {val:.6f}")
    print("=" * 50)

    # 画图
    param_history = np.array(param_history)
    fig, axes = plt.subplots(3, 3, figsize=(16, 10))

    axes[0, 0].plot(loss_history, label='Random training windows', alpha=0.5)
    axes[0, 0].plot(statistics['evaluation_history'], label='Fixed training windows')
    axes[0, 0].legend()
    axes[0, 0].set_yscale('log')
    axes[0, 0].set_title('Loss'); axes[0, 0].grid(True)

    for i, name in enumerate(param_names):
        ax = axes[(i+1)//3, (i+1)%3]
        ax.plot(param_history[:, i])
        ax.set_title(name); ax.grid(True)

    # 仿真对比（最后一条样本）
    diff_env = DifferentiableYawSimEnv(dt=DT)
    J, tc, b, td = [torch.tensor(v, device=device) for v in final_params]
    diff_env.set_params(J, tc, b, td)

    sample = samples[-1]
    tau_t = torch.tensor(sample['torque'], dtype=torch.float32, device=device)
    yaw_true = torch.tensor(sample['yaw'], dtype=torch.float32, device=device)
    gz_true = torch.tensor(sample['gz'], dtype=torch.float32, device=device)
    theta_init = torch.tensor(float(sample['yaw'][0]), dtype=torch.float32, device=device)
    omega_init = torch.tensor(float(sample['gz'][0]), dtype=torch.float32, device=device)
    with torch.no_grad():
        yaw_sim, gz_sim = diff_env.simulate(tau_t, theta_init, omega_init)

    # 位置（yaw）曲线对比
    axes[1, 2].plot(yaw_true.cpu()[:300], label='Real', alpha=0.7)
    axes[1, 2].plot(yaw_sim.cpu()[:300], '--', label='Sim', alpha=0.7)
    axes[1, 2].set_title('yaw compare'); axes[1, 2].legend(); axes[1, 2].grid(True)

    # 速度（gz）曲线对比
    axes[2, 0].plot(gz_true.cpu()[:300], label='Real', alpha=0.7)
    axes[2, 0].plot(gz_sim.cpu()[:300], '--', label='Sim', alpha=0.7)
    axes[2, 0].set_title('gz compare'); axes[2, 0].legend(); axes[2, 0].grid(True)

    # 删除多余的空白子图
    fig.delaxes(axes[2, 1])
    fig.delaxes(axes[2, 2])

    plt.tight_layout()
    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
        result = {
            'parameters': dict(zip(param_names, final_params)),
            'initial_parameters': dict(zip(param_names, args.initial_params)),
            'nominal_dt': DT,
            'epochs': statistics['epochs_completed'],
            'max_epochs': args.epochs,
            'patience': args.patience,
            'min_delta': args.min_delta,
            'best_epoch': statistics['best_epoch'],
            'initial_evaluation_loss': statistics['initial_evaluation_loss'],
            'best_evaluation_loss': statistics['best_evaluation_loss'],
            'evaluation_window_count': statistics['evaluation_window_count'],
            'epochs_without_improvement': statistics['epochs_without_improvement'],
            'stopping_reason': statistics['stopping_reason'],
            'sample_count': len(samples),
            'final_training_loss': loss_history[-1],
            'last_sample_rollout_position_rmse': torch.mean((yaw_sim - yaw_true) ** 2).sqrt().item(),
            'last_sample_rollout_velocity_rmse': torch.mean((gz_sim - gz_true) ** 2).sqrt().item(),
        }
        with open(os.path.join(args.output_dir, 'fit_result.json'), 'w', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2, ensure_ascii=False)
        np.savez(os.path.join(args.output_dir, 'training_history.npz'),
                 loss=np.array(loss_history), parameters=param_history,
                 evaluation_loss=np.array(statistics['evaluation_history']))
        fig.savefig(os.path.join(args.output_dir, 'fit_comparison.png'), dpi=160)
    if not args.no_show:
        plt.show()
    plt.close(fig)

if __name__ == "__main__":
    main()
