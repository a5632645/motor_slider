#include "pid.h"
#include "config.h"

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void Pid_Init(struct PidCtx* pid, float kp, float ki, uint16_t pwm_bias, uint16_t pwm_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->integral_ = 0.0f;

    /* 输出限幅 = ±(pwm_max - pwm_bias)，调用方加 bias 得实际 duty */
    float range = (float)(pwm_max - pwm_bias);
    pid->output_min_ = -range;
    pid->output_max_ = range;

    pid->integral_limit_ = PID_INTEGRAL_LIMIT;
    pid->pwm_bias_ = pwm_bias;
    pid->pwm_max_ = pwm_max;
}


float Pid_Update(struct PidCtx* pid, float setpoint, float feedback) {
    float error = setpoint - feedback;
    float p_term = pid->kp * error;

    pid->integral_ += error;
    if (pid->integral_ > pid->integral_limit_)
        pid->integral_ = pid->integral_limit_;
    else if (pid->integral_ < -pid->integral_limit_)
        pid->integral_ = -pid->integral_limit_;
    float i_term = pid->ki * pid->integral_;

    float output = p_term + i_term;
    if (output > pid->output_max_)
        output = pid->output_max_;
    else if (output < pid->output_min_)
        output = pid->output_min_;

    return output;
}


void Pid_Reset(struct PidCtx* pid) {
    pid->integral_ = 0.0f;
}
