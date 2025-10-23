#include "pid.hpp"
#include <cmath>

namespace aim::algorithm {
    float PID::calculate(const float ref, const float set) {
        if (std::isnan(sum_error_)) {
            sum_error_ = 0.0f;
        }

        set_point_last_ = set_point_;
        set_point_ = set;
        pre_error_ = set_point_ - ref;

        if (std::fabs(pre_error_) < deadband_) {
            return 0.0f;
        }

        d_error_ = pre_error_ - last_error_;

        if (pre_error_ > -error_max_ && pre_error_ < error_max_) {
            sum_error_ += (pre_error_ + last_error_) / 2.0f;
        }

        last_error_ = pre_error_;

        sum_error_ = limit_max_min(sum_error_, i_max_, -i_max_);

        p_out_ = kp_ * pre_error_;
        i_out_ = ki_ * sum_error_;
        d_out_ = kd_ * d_error_;

        out_ = limit_max_min(p_out_ + i_out_ + d_out_, out_max_, -out_max_);

        return out_;
    }

    float PID::calculate(const float ref) {
        return calculate(ref, set_point_);
    }

    void PID::clear() {
        set_point_ = 0.0f;
        set_point_last_ = 0.0f;
        last_error_ = 0.0f;
        pre_error_ = 0.0f;
        sum_error_ = 0.0f;
        d_error_ = 0.0f;
        p_out_ = i_out_ = d_out_ = out_ = 0.0f;
    }

    void PID::set_setpoint(const float sp) {
        set_point_ = sp;
    }

    float PID::get_output() const {
        return out_;
    }
}