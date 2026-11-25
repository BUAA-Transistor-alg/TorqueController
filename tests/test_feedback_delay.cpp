#include "tcs/mpc/feedback_delay_compensation.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace {

void checkNear(double actual, double expected) {
    if (std::abs(actual - expected) > 1e-12) {
        throw std::runtime_error("反馈延迟外推验证失败");
    }
}

} // namespace

int main() {
    // 与恒速运动的解析位置比较，覆盖正反向、底盘转动和多圈位置。
    for (double delay : {0.0, 0.005, 0.010, 0.020}) {
        tcs::FeedbackDelayCompensation compensation(delay);
        for (double joint_rate : {-1.0, 0.0, 1.0}) {
            for (double chassis_rate : {-2.0, 0.0, 2.0}) {
                const double joint = 40.0;
                const double base = -30.0;
                const auto state = compensation.predict(
                    joint, joint_rate, joint + base, chassis_rate);
                checkNear(state.theta, joint + joint_rate * delay);
                checkNear(state.omega, joint_rate);
                checkNear(state.imu_yaw, joint + base +
                          (joint_rate + chassis_rate) * delay);
                // 参考转换中的底盘方向必须只随底盘角速度变化。
                checkNear(state.imu_yaw - state.theta, base + chassis_rate * delay);
            }
        }
    }

    for (double delay : {-0.001, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            tcs::FeedbackDelayCompensation compensation(delay);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected) throw std::runtime_error("无效反馈延迟未被拒绝");
    }
    std::puts("反馈延迟外推验证通过");
}
