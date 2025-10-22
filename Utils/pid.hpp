#ifndef PID_H
#define PID_H

namespace aim::algorithms {
    class PID {
    public:
        explicit PID(const float kp = 0.0f, const float ki = 0.0f, const float kd = 0.0f,
                     const float max_out = 0.0f, const float max_iout = 0.0f) : kp_(kp), ki_(ki), kd_(kd),
                                                                    deadband_(0.0f),
                                                                    out_max_(max_out), i_max_(max_iout),
                                                                    error_max_(9999999.0f),
                                                                    set_point_(0.0f), set_point_last_(0.0f),
                                                                    last_error_(0.0f), pre_error_(0.0f),
                                                                    sum_error_(0.0f), d_error_(0.0f),
                                                                    p_out_(0.0f), i_out_(0.0f), d_out_(0.0f),
                                                                    out_(0.0f) {}

        float calculate(float ref, float set);

        float calculate(float ref);

        void clear();

        void set_setpoint(float sp);

        [[nodiscard]] float get_output() const;

        void set_kp(const float kp) { kp_ = kp; }
        void set_ki(const float ki) { ki_ = ki; }
        void set_kd(const float kd) { kd_ = kd; }

        void set_deadband(const float db) { deadband_ = db; }
        void set_output_limit(const float max_out) { out_max_ = max_out; }
        void set_integral_limit(const float max_iout) { i_max_ = max_iout; }

    private:
        float kp_;
        float ki_;
        float kd_;
        float deadband_;

        float out_max_;
        float i_max_;
        float error_max_;

        float set_point_;
        float set_point_last_;
        float last_error_;
        float pre_error_;
        float sum_error_;
        float d_error_;

        float p_out_;
        float i_out_;
        float d_out_;
        float out_;
    };

    float limit_max_min(float value, float max, float min);


}

#endif
