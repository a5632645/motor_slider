/**
 * @file    tick.h
 * @brief   1ms SysTick 定时器接口
 */

#pragma once
#include <stdint.h>

/**
 * @brief 初始化 SysTick 定时器，产生 1ms 中断
 */
void Tick_Init(void);

/**
 * @brief 获取当前系统滴答计数值（毫秒）
 * @return 从 Init 起经过的毫秒数
 */
uint32_t Tick_Get(void);

/**
 * @brief 滴答计数器递增，由 SysTick_Handler 周期调用
 */
void Tick_Increment(void);
