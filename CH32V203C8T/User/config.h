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

/** @brief 闭环超时 (ms)，超过此时间未到达目标则停止电机 */
#define CTRL_TIMEOUT_MS 1000

/* ===================================================================
 * PID 默认参数
 * =================================================================== */

/** @brief 默认 PID 增益 (Motor_InitControl 中使用的初始值) */
#define PID_DEFAULT_KP 0.5f
#define PID_DEFAULT_KI 0.01f
#define PID_DEFAULT_KD 0.1f

/** @brief PID 输出限幅 (±999 对应 PWM 占空比 0~999) */
#define PID_OUTPUT_MIN -999.0f
#define PID_OUTPUT_MAX 999.0f

/** @brief PID 积分限幅，防止积分饱和 */
#define PID_INTEGRAL_LIMIT 500.0f

/** @brief 电机最小起步占空比，克服静摩擦 (0~999) */
#define PWM_MIN_START_DUTY 800

/* ===================================================================
 * 电机 3 (GPIO 开关控制)
 * =================================================================== */

/** @brief 电机 3 开关死区，|adc - target| ≤ 此值不动作 */
#define MOTOR3_DEADBAND 20

/* ===================================================================
 * HID
 * =================================================================== */

/** @brief HID0 printf FIFO 缓冲区大小 (字节)，必须为 2 的幂 */
#define HID_FIFO_SIZE 512
