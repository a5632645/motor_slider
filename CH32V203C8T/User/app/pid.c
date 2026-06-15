#include "pid.h"
#include "config.h"

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void Pid_Init(struct PidCtx* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_ = 0.0f;
    pid->prev_error_ = 0.0f;
    pid->output_min_ = PID_OUTPUT_MIN;
    pid->output_max_ = PID_OUTPUT_MAX;
    pid->integral_limit_ = PID_INTEGRAL_LIMIT;
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

    float d_term = pid->kd * (error - pid->prev_error_);
    pid->prev_error_ = error;

    float output = p_term + i_term + d_term;
    if (output > pid->output_max_)
        output = pid->output_max_;
    else if (output < pid->output_min_)
        output = pid->output_min_;

    return output;
}


void Pid_Reset(struct PidCtx* pid) {
    pid->integral_ = 0.0f;
    pid->prev_error_ = 0.0f;
}
