#ifndef TCS_FEEDBACK_DELAY_COMPENSATION_H
#define TCS_FEEDBACK_DELAY_COMPENSATION_H

#include <cmath>
#include <stdexcept>

namespace tcs {

// 假设融合反馈整体滞后，以恒定角速度外推关节位置和世界系云台方向。
class FeedbackDelayCompensation {
public:
    struct State {
        double theta;
        double omega;
        double imu_yaw;
    };

    explicit FeedbackDelayCompensation(double delay_s) : delay_s_(delay_s) {
        if (!std::isfinite(delay_s_) || delay_s_ < 0.0) {
            throw std::invalid_argument("feedback_delay_s must be finite and nonnegative");
        }
    }

    State predict(double theta, double omega, double imu_yaw,
                  double chassis_omega) const {
        return {theta + omega * delay_s_, omega,
                imu_yaw + (omega + chassis_omega) * delay_s_};
    }

private:
    double delay_s_;
};

} // namespace tcs
#endif
