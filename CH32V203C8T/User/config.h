/**
 * @file    config.h
 * @brief   可配置参数 — 闭环控制、PID、电机行为
 *
 * @note    修改后需重新编译固件
 */
#pragma once

/* ===================================================================
 * 闭环控制
 * =================================================================== */

/** @brief ADC 误差阈值，|adc - target| ≤ CTRL_ERROR_THRESHOLD 视为到达 */
#define CTRL_ERROR_THRESHOLD 2

/** @brief 到位稳定时间 (ms)，连续满足到位条件达到该时间才停止 */
#define CTRL_SETTLE_MS 5

/** @brief 静止判定阈值，相邻控制周期 ADC 变化量 ≤ 该值视为静止 */
#define CTRL_STILL_THRESHOLD 10

/** @brief 闭环超时 (ms)，超过此时间未到达目标则停止电机 */
#define CTRL_TIMEOUT_MS 1000

/** @brief 电机控制周期 (us)，250us = 4kHz */
#define CTRL_LOOP_US 250

/** @brief 控制用 ADC 一阶 IIR 滤波强度，1=快，2=较稳，3=更稳但延迟更大 */
#define CTRL_ADC_FILTER_SHIFT 2

/** @brief 目标变化小于等于该阈值时直接吸附，不启动电机 */
#define CTRL_TARGET_SNAP_THRESHOLD 3

/** @brief PID 输出绝对值小于该阈值时视为 0，不叠加 PWM_BIAS */
#define CTRL_OUTPUT_DEADBAND 2.0f

/** @brief 目标附近小于等于该阈值时禁止反向驱动，避免来回敲击 */
#define CTRL_REVERSE_THRESHOLD 6

/** @brief MIDI RX 后 ADC 离开目标中心超过该阈值才恢复发送 CC */
#define MIDI_RX_HOLD_RELEASE_ADC 96

/** @brief MIDI 专用 ADC 平均采样次数 */
#define MIDI_ADC_AVG_COUNT 8

/** @brief MIDI CC 候选值连续出现该次数后才发送 */
#define MIDI_CC_CONFIRM_COUNT 3

/* ===================================================================
 * PID 默认参数
 * =================================================================== */

/** @brief 默认 PID 增益 (Motor_InitControl 中使用的初始值) */
#define PID_DEFAULT_KP 0.03f
#define PID_DEFAULT_KI 0.0001f

/** @brief PID 积分限幅，防止积分饱和 */
#define PID_INTEGRAL_LIMIT 1.0e4f

/** @brief PWM 最小起步占空比，克服静摩擦 */
#define PWM_BIAS 780

/** @brief PWM 最大占空比 (0~999) */
#define PWM_MAX 999

/* ===================================================================
 * HID
 * =================================================================== */

/** @brief HID0 printf FIFO 缓冲区大小 (字节)，必须为 2 的幂 */
#define HID_FIFO_SIZE 512

#define MIDI_TX_FIFO_SIZE 128
