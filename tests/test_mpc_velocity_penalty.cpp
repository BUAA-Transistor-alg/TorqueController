#include "tcs/mpc/mpc_controller.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

tcs::MPCController controller(double scale, int horizon = 1) {
    return tcs::MPCController(0.1, 0.1, 1.0, 0.0, 0.0, 0.0,
                              10.0, 1000.0, horizon, 0.0, 2.0, 0.0, 100,
                              1e-6, scale);
}

void check(bool condition) {
    if (!condition) throw std::runtime_error("MPC 角速度惩罚验证失败");
}

} // namespace

int main() {
    // 单步无摩擦动力学的最优力矩有解析解，可独立验证系数平方及减速方向。
    for (double scale : {0.0, 0.7, 1.0, 1.4}) {
        for (double omega : {-1.0, 0.0, 1.0}) {
            auto mpc = controller(scale);
            const auto result = mpc.step(0.0, omega, {0.0});
            const double weight = scale * scale;
            const double expected = -weight * 0.1 * omega / (2.0 + weight * 0.01);
            check(std::isfinite(result.torque));
            check(std::abs(result.torque - expected) < 1e-5);
            check(std::abs(result.omega - (omega + result.torque * 0.1)) < 1e-12);
        }
    }

    // 验证实际二十步预测窗口及热启动连续求解的减速效果。
    auto mpc = controller(1.4, 20);
    double theta = 0.0, omega = 1.0;
    for (int step = 0; step < 20; ++step) {
        const auto result = mpc.step(theta, omega, std::vector<double>(20, 0.0));
        check(std::isfinite(result.torque) && std::isfinite(result.omega));
        check(result.torque < 0.0 && std::abs(result.torque) <= 10.0);
        check(result.omega > 0.0 && result.omega < omega);
        check(mpc.lastPred().size() == 20);
        theta = result.theta;
        omega = result.omega;
    }

    for (double scale : {-1.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            auto invalid = controller(scale);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        check(rejected);
    }
    std::puts("MPC 角速度惩罚验证通过");
}
