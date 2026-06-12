#pragma once
#include <stdint.h>

/**
 * @brief PID 控制器上下文
 */
struct PidCtx {
    float kp;              /* 比例增益 */
    float ki;              /* 积分增益 */
    float kd;              /* 微分增益 */
    float integral_;       /* 积分累计值 */
    float prev_error_;     /* 上一次误差 */
    float output_min_;     /* 输出下限 */
    float output_max_;     /* 输出上限 */
    float integral_limit_; /* 积分限幅 */
};

/**
 * @brief 初始化 PID 控制器
 * @param pid   PID 上下文
 * @param kp    比例增益
 * @param ki    积分增益
 * @param kd    微分增益
 */
void Pid_Init(struct PidCtx* pid, float kp, float ki, float kd);

/**
 * @brief PID 更新运算
 * @param pid       PID 上下文
 * @param setpoint  目标值
 * @param feedback  反馈值
 * @return          float 控制输出
 */
float Pid_Update(struct PidCtx* pid, float setpoint, float feedback);

/**
 * @brief 重置 PID 内部状态（积分、上次误差清零）
 * @param pid   PID 上下文
 */
void Pid_Reset(struct PidCtx* pid);
