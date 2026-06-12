#pragma once
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>


/* 电机总数 */
#define MOTOR_COUNT 8

/* 电机通道 (0-based index, 对应 ADC 通道) */
#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define MOTOR_5 4
#define MOTOR_6 5
#define MOTOR_7 6
#define MOTOR_8 7

/**
 * @brief 电机方向 / 制动模式
 */
enum MotorDir {
    kMotorDir_Stop = 0, /* IN1=0, IN2=0 (coast) */
    kMotorDir_Forward,  /* IN1=PWM, IN2=0 */
    kMotorDir_Reverse,  /* IN1=0, IN2=PWM */
    kMotorDir_Brake,    /* IN1=1, IN2=1 */
};

/**
 * @brief 初始化所有 PWM 定时器 (TIM1~TIM4) 和电机 3 GPIO
 */
void Motor_InitPwm(void);

/**
 * @brief 设置指定电机的方向和 PWM 占空比
 * @param ch   电机序号 (MOTOR_1 ~ MOTOR_8)
 * @param dir  方向
 * @param duty 占空比 (0~999)
 */
void Motor_SetPwm(uint8_t ch, enum MotorDir dir, uint16_t duty);

/**
 * @brief 初始化 ADC1 (常规组 scan 8通道) + DMA1 通道 1
 */
void Motor_InitAdc(void);

/**
 * @brief 软件触发 ADC 转换，DMA 自动搬运结果
 */
void Motor_StartAdcConversion(void);

/**
 * @brief 电机运动状态
 */
struct MotorState {
    uint16_t target_adc_;  /* 目标位置 (0~4095) */
    uint16_t current_adc_; /* 当前位置 (0~4095) */
    enum MotorDir dir_;    /* 当前方向 */
    uint16_t duty_;        /* 当前占空比 */
    struct PidCtx pid_;    /* PID 控制器 */
    bool active_;          /* true=正在闭环寻找目标 */
    uint16_t timeout_;     /* 超时计数，递减到0停止 */
};

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
 * @brief 检查 ADC 数据是否就绪
 * @return true 就绪, false 忙
 */
bool Motor_IsAdcReady(void);

/**
 * @brief 获取 8 路电机当前状态
 * @param adc         输出缓冲区 (8), 接收当前 ADC 值
 * @param target      输出缓冲区 (8), 接收目标 ADC 值
 * @param duty        输出缓冲区 (8), 接收当前占空比
 * @param active_flags 输出, 每路 1 bit (bit0=CH1 active, ... bit7=CH8 active)
 */
void Motor_GetStatus(uint16_t adc[8], uint16_t target[8], uint16_t duty[8], uint8_t* active_flags);
