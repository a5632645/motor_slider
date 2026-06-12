#include "pid.h"
#include "config.h"

/*********************************************************************
 * @fn      Pid_Init
 *
 * @brief   初始化 PID 控制器参数
 *
 * @param   pid   PID 上下文
 * @param   kp    比例增益
 * @param   ki    积分增益
 * @param   kd    微分增益
 *
 * @return  none
 */
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

/*********************************************************************
 * @fn      Pid_Update
 *
 * @brief   执行一次 PID 运算
 *
 * @param   pid        PID 上下文
 * @param   setpoint   目标值
 * @param   feedback   反馈值
 *
 * @return  float  控制输出（范围 output_min_ ~ output_max_）
 */
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

/*********************************************************************
 * @fn      Pid_Reset
 *
 * @brief   重置 PID 积分和上次误差
 *
 * @param   pid   PID 上下文
 *
 * @return  none
 */
void Pid_Reset(struct PidCtx* pid) {
    pid->integral_ = 0.0f;
    pid->prev_error_ = 0.0f;
}
