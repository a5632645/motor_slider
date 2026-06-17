#pragma once

#include <stdint.h>

/**
 * @brief 初始化 SysTick 定时器，产生低频中断并提供高精度时间读取
 */
void Tick_Init(void);

/**
 * @brief 获取当前系统时间（毫秒）
 * @return 从 Init 起经过的毫秒数
 */
uint32_t Tick_GetMs(void);

/**
 * @brief 获取当前系统时间（微秒）
 * @return 从 Init 起经过的微秒数
 */
uint32_t Tick_GetUs(void);

/**
 * @brief 获取当前系统时间（毫秒），兼容旧接口
 * @return 从 Init 起经过的毫秒数
 */
uint32_t Tick_Get(void);

/**
 * @brief 处理 SysTick 周期中断，由 SysTick_Handler 调用
 */
void Tick_OnSysTick(void);
