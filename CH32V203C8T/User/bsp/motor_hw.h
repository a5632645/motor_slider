#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 电机索引枚举
 */
enum MotorIdx {
    kMotorIdx_0 = 0,
    kMotorIdx_1,
    kMotorIdx_2,
    kMotorIdx_3,
    kMotorIdx_4,
    kMotorIdx_5,
    kMotorIdx_6,
    kMotorIdx_7,
    kMotorIdx_Count
};

/**
 * @brief 电机方向 / 制动模式
 */
enum MotorDir {
    kMotorDir_Stop = 0,    /* IN1=0, IN2=0 (coast) */
    kMotorDir_Forward,     /* IN1=PWM, IN2=0 */
    kMotorDir_Reverse,     /* IN1=0, IN2=PWM */
    kMotorDir_Brake,       /* IN1=1, IN2=1 */
};

/**
 * @brief 初始化电机硬件
 */
void MotorHw_Init(void);

/**
 * @brief 软件触发 ADC 转换，DMA 自动搬运结果
 */
void MotorHw_StartAdcConversion(void);

/**
 * @brief 检查 ADC 数据是否就绪
 * @return true 就绪, false 忙
 */
bool MotorHw_IsAdcReady(void);

/**
 * @brief 读取 ADC DMA 缓冲区转换结果
 * @param buffer 输出缓冲区 (kMotorIdx_Count 个 uint16)，接收 ADC 原始值 (0~4095)
 */
void MotorHw_GetAdcValue(uint16_t buffer[kMotorIdx_Count]);

/**
 * @brief 设置指定电机的方向和 PWM 占空比
 * @param ch   电机序号 (MOTOR_1 ~ MOTOR_8)
 * @param dir  方向
 * @param duty 占空比 (0~999)
 */
void MotorHw_SetPwm(uint8_t ch, enum MotorDir dir, uint16_t duty);
