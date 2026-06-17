#pragma once

#include "app/pid.h"
#include "bsp/motor_hw.h"

#include <stdbool.h>
#include <stdint.h>

// ------------------------------------------------------------
// type
// ------------------------------------------------------------

/**
 * @brief 电机运动状态
 */
struct MotorState {
    uint16_t target_adc_;  /* 目标位置 (0~4095) */
    uint16_t current_adc_; /* 当前位置 (0~4095) */
    uint32_t filtered_adc_q8_; /* 控制用滤波 ADC，Q8 */
    enum MotorDir dir_;    /* 当前方向 */
    uint16_t duty_;        /* 当前占空比 */
    struct PidCtx pid_;    /* PID 控制器 */
    bool active_;          /* true=正在闭环寻找目标 */
    uint32_t start_tick_;  /* 本次运动开始时间 (ms) */
    uint32_t settle_tick_; /* 进入稳定区间的起始时间 (ms) */
    uint16_t last_adc_;    /* 上一控制周期 ADC 值 */
    bool settling_;        /* true=正在累计稳定时间 */
};

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

/**
 * @brief 初始化电机控制状态
 */
void Motor_InitControl(void);

/**
 * @brief 主控制循环：读取 ADC → PID → 更新 PWM
 *        在 DMA 完成中断中调用
 */
void Motor_RunControlLoop(void);

/**
 * @brief 软件触发 ADC 转换，DMA 自动搬运结果
 */
void Motor_StartAdcConversion(void);

/**
 * @brief 设置电机目标位置
 * @param ch          电机序号
 * @param target_adc  目标 ADC 值 (0~4095)
 */
void Motor_SetTarget(uint8_t ch, uint16_t target_adc);

/**
 * @brief 停止所有电机
 */
void Motor_StopAll(void);

/**
 * @brief 设置指定电机的 PID 参数
 * @param ch 电机序号 (0~7), 0xFF=全部
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void Motor_SetPid(uint8_t ch, float kp, float ki, float kd);

/**
 * @brief 设置 PWM 起步偏置
 * @param pwm_bias 起步偏置，占空比范围 0~当前 PWM 最大值
 */
void Motor_SetPwmBias(uint16_t pwm_bias);

/**
 * @brief 设置 PWM 最大占空比
 * @param pwm_max PWM 最大占空比
 */
void Motor_SetPwmMax(uint16_t pwm_max);

/**
 * @brief 检查 ADC 数据是否就绪
 * @return true 就绪, false 忙
 */
bool Motor_IsAdcReady(void);

/**
 * @brief 通过 HID1 上报电机状态
 */
void Motor_SendStatus(void);

/**
 * @brief 处理 HID1 OUT 命令
 */
void Motor_ProcessCommand(void);

bool Motor_IsMoving(uint8_t ch);

// ------------------------------------------------------------
// dependency
// ------------------------------------------------------------

extern void Motor_OnActiveChanged(uint8_t ch, bool active);

extern void Motor_OnAdcReady(uint16_t raw_adc[kMotorIdx_Count]);
