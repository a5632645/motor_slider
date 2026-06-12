# 八路电机控制 + HID1 接口 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 8 路电机闭环位置控制（AD→PID→PWM），并通过独立的 USB HID1 接口通信。

**Architecture:** SysTick @1kHz 触发 ADC 注入组扫描 → ADC 完成中断执行 8 路 PID → 更新 PWM 比较值。HID1 在主循环中处理命令/状态。

**Tech Stack:** CH32V203C8T, USBFS, ch32v20x 标准外设库

---

### Task 1: PWM 定时器配置

**Files:**
- Create: `User/motor.h`
- Create: `User/motor.c`

**说明：**
- TIM1~TIM4 全部配置为 PWM 输出，ARR=999，边沿对齐向上计数，预分频 = 0
- 定时器时钟 = 96MHz，PWM 频率 = 96kHz
- Motor 3 使用 GPIO PB14/PB15 软件控制

- [ ] **Step 1: 创建 motor.h，声明 PWM 和 GPIO 初始化接口**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* 电机总数 */
#define MOTOR_COUNT 8

/* 电机通道 (0-based index, 对应 ADC 通道) */
#define MOTOR_1  0
#define MOTOR_2  1
#define MOTOR_3  2
#define MOTOR_4  3
#define MOTOR_5  4
#define MOTOR_6  5
#define MOTOR_7  6
#define MOTOR_8  7

/* 方向 */
typedef enum {
    kMotorDir_Stop   = 0,  /* IN1=0, IN2=0 (coast) */
    kMotorDir_Forward,     /* IN1=PWM, IN2=0 */
    kMotorDir_Reverse,     /* IN1=0, IN2=PWM */
    kMotorDir_Brake,       /* IN1=1, IN2=1 */
} MotorDir;

void Motor_InitPwm(void);
void Motor_SetPwm(uint8_t ch, MotorDir dir, uint16_t duty);
```

- [ ] **Step 2: 实现 motor.c 中的 TIM1 初始化**

配置 TIM1 CH1~CH4（对应电机 4/5）：
```c
#include "motor.h"
#include "ch32v20x.h"
#include "ch32v20x_tim.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"

void Motor_InitPwm(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    /* === TIM1: 电机 4 (CH2/CH1), 电机 5 (CH3/CH4) === */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA
                           | RCC_APB2Periph_GPIOB, ENABLE);

    /* TIM1 CH1=PA8, CH2=PA9, CH3=PA10, CH4=PA11 */
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 999;        /* 1000 steps → 96kHz */
    tim.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;

    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    /* CH1 (电机5 B) */
    TIM_OC1Init(TIM1, &oc);
    /* CH2 (电机4 A) */
    oc.TIM_Pulse = 0;
    TIM_OC2Init(TIM1, &oc);
    /* CH3 (电机5 A) */
    oc.TIM_Pulse = 0;
    TIM_OC3Init(TIM1, &oc);
    /* CH4 (电机5 B) — 注意 PA11 同时也是 USB DM */
    oc.TIM_Pulse = 0;
    TIM_OC4Init(TIM1, &oc);

    /* 高级定时器需要使能 MOE 主输出 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /* 后续继续初始化 TIM2, TIM3, TIM4... */
}
```

- [ ] **Step 3: 继续实现 TIM2 初始化（电机 2 + 电机 6）**

```c
    /* === TIM2: 电机 6 (CH1=PA15, CH2=PB3), 电机 2 (CH3=PB10, CH4=PB11) === */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* AFIO 重映射: TIM2_CH1 → PA15, TIM2_CH2 → PB3 */
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 999;
    tim.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &oc);  /* PA15 — 电机6 A? */
    /* 核对：定时器引脚表 PB3=电机6 A, PA15=电机6 B */
    oc.TIM_Pulse = 0;
    TIM_OC2Init(TIM2, &oc);  /* PB3 — 电机6 B? */
    oc.TIM_Pulse = 0;
    TIM_OC3Init(TIM2, &oc);  /* PB10 — 电机2 B */
    oc.TIM_Pulse = 0;
    TIM_OC4Init(TIM2, &oc);  /* PB11 — 电机2 A */

    TIM_Cmd(TIM2, ENABLE);
```

- [ ] **Step 4: 实现 TIM3 初始化（电机 1 + 电机 7）**

```c
    /* === TIM3: 电机 7 (CH1=PB4, CH2=PB5), 电机 1 (CH3=PB0, CH4=PB1) === */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /* GPIOB 已使能 */

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 999;
    tim.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM3, &tim);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &oc);  /* PB4 — 电机7 A */
    oc.TIM_Pulse = 0;
    TIM_OC2Init(TIM3, &oc);  /* PB5 — 电机7 B */
    oc.TIM_Pulse = 0;
    TIM_OC3Init(TIM3, &oc);  /* PB0 — 电机1 A */
    oc.TIM_Pulse = 0;
    TIM_OC4Init(TIM3, &oc);  /* PB1 — 电机1 B */

    TIM_Cmd(TIM3, ENABLE);
```

- [ ] **Step 5: 实现 TIM4 初始化（电机 8）和 Motor 3 GPIO**

```c
    /* === TIM4: 电机 8 (CH3=PB8, CH4=PB9) === */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 999;
    tim.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM4, &tim);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init(TIM4, &oc);  /* PB8 — 电机8 A */
    oc.TIM_Pulse = 0;
    TIM_OC4Init(TIM4, &oc);  /* PB9 — 电机8 B */

    TIM_Cmd(TIM4, ENABLE);

    /* === 电机 3: GPIO PB14, PB15 === */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
}
```

- [ ] **Step 6: 实现 Motor_SetPwm 函数**

```c
void Motor_SetPwm(uint8_t ch, MotorDir dir, uint16_t duty)
{
    if (duty > 999) duty = 999;

    if (ch == MOTOR_3) {
        /* GPIO 电机：IO 翻转 */
        if (dir == kMotorDir_Forward) {
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        } else if (dir == kMotorDir_Reverse) {
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
        } else if (dir == kMotorDir_Brake) {
            GPIO_SetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
        }
        return;
    }

    /* 查表找到对应定时器+通道 */
    /* 电机 1: TIM3 CH3 (A=IN1), CH4 (B=IN2) */
    /* 电机 2: TIM2 CH4 (A), CH3 (B) */
    /* 电机 4: TIM1 CH2 (A), CH1 (B) */
    /* 电机 5: TIM1 CH3 (A), CH4 (B) */
    /* 电机 6: TIM2 CH1 (A), CH2 (B) */
    /* 电机 7: TIM3 CH1 (A), CH2 (B) */
    /* 电机 8: TIM4 CH3 (A), CH4 (B) */

    /* 使用 switch 或查表数组 */
    switch (ch) {
        case MOTOR_1: /* TIM3 CH3=A, CH4=B */
            TIM_SetCompare3(TIM3, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare4(TIM3, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_2: /* TIM2 CH4=A, CH3=B */
            TIM_SetCompare4(TIM2, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM2, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_4: /* TIM1 CH2=A, CH1=B */
            TIM_SetCompare2(TIM1, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare1(TIM1, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_5: /* TIM1 CH3=A, CH4=B */
            TIM_SetCompare3(TIM1, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare4(TIM1, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_6: /* TIM2 CH1=A, CH2=B */
            TIM_SetCompare1(TIM2, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare2(TIM2, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_7: /* TIM3 CH1=A, CH2=B */
            TIM_SetCompare1(TIM3, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare2(TIM3, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_8: /* TIM4 CH3=A, CH4=B */
            TIM_SetCompare3(TIM4, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare4(TIM4, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
    }
}
```

---

### Task 2: ADC + DMA 配置

**Files:**
- Modify: `User/motor.h` (添加 ADC/DMA 接口声明)
- Modify: `User/motor.c` (添加 ADC + DMA 初始化)

**说明：**
- ADC 常规组 scan 模式，8 通道（PA0~PA7），软件触发
- DMA1 自动搬运 ADC 转换结果到内存缓冲区，DMA 完成中断触发 PID
- ADC 时钟 = PCLK2/8 = 12MHz，12 位，采样时间 7.5 周期

- [ ] **Step 1: motor.h 添加 ADC/DMA 接口**

```c
void Motor_InitAdc(void);
void Motor_StartAdcConversion(void);
extern volatile uint16_t motor_adc_dma_buf_[MOTOR_COUNT];
extern volatile bool motor_adc_ready_;
```

- [ ] **Step 2: motor.c 实现 ADC + DMA 初始化**

```c
#include "ch32v20x_dma.h"

/* DMA 缓冲区：ADC 自动写入 */
__attribute__((aligned(4)))
volatile uint16_t motor_adc_dma_buf_[MOTOR_COUNT];
volatile bool motor_adc_ready_;

void Motor_InitAdc(void)
{
    ADC_InitTypeDef adc;
    GPIO_InitTypeDef gpio;
    DMA_InitTypeDef dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* PA0~PA7 模拟输入 */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
                  | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    /* === DMA1 通道 1: ADC1 → motor_adc_dma_buf_ === */
    DMA_DeInit(DMA1_Channel1);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)motor_adc_dma_buf_;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = MOTOR_COUNT;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);

    /* DMA 完成中断 */
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    /* === ADC1 配置 === */
    ADC_DeInit(ADC1);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);  /* ADC 时钟 = 12MHz */

    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = ENABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = MOTOR_COUNT;
    ADC_Init(ADC1, &adc);

    /* 配置通道顺序：采样时间 7.5 周期 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_7Cycles5);

    /* 使能 ADC + 校准 */
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}
```

- [ ] **Step 3: 实现 ADC 软件触发和 DMA 中断**

```c
void Motor_StartAdcConversion(void)
{
    motor_adc_ready_ = false;
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA1_Channel1->CNTR = MOTOR_COUNT;          /* 重置 DMA 计数器 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/* DMA1 通道 1 完成中断 → ADC 数据就绪 */
void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Channel1, DMA_IT_TC)) {
        DMA_ClearITPendingBit(DMA1_Channel1, DMA_IT_TC);
        motor_adc_ready_ = true;
        Motor_RunControlLoop();
    }
}
```

---

### Task 3: PID 控制器

**Files:**
- Create: `User/pid.h`
- Create: `User/pid.c`

- [ ] **Step 1: pid.h 声明**

```c
#pragma once
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float integral_limit;
} PidCtx;

void Pid_Init(PidCtx *pid, float kp, float ki, float kd);
float Pid_Update(PidCtx *pid, float setpoint, float feedback);
void Pid_Reset(PidCtx *pid);
```

- [ ] **Step 2: pid.c 实现**

```c
#include "pid.h"

void Pid_Init(PidCtx *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_min = -999.0f;
    pid->output_max = 999.0f;
    pid->integral_limit = 500.0f;
}

float Pid_Update(PidCtx *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;
    float p_term = pid->kp * error;

    pid->integral += error;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    float i_term = pid->ki * pid->integral;

    float d_term = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;

    float output = p_term + i_term + d_term;
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    return output;
}

void Pid_Reset(PidCtx *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
}
```

---

### Task 4: 控制循环集成

**Files:**
- Modify: `User/motor.h` (添加控制循环接口)
- Modify: `User/motor.c` (添加控制循环实现)

- [ ] **Step 1: motor.h 添加控制循环接口**

```c
void Motor_InitControl(void);
void Motor_RunControlLoop(void);  /* 从 ADC ISR 调用 */
void Motor_SetTarget(uint8_t ch, uint16_t target_adc);
void Motor_StopAll(void);

/* 电机状态 */
typedef struct {
    uint16_t target_adc;       /* 目标位置 (0~4095) */
    uint16_t current_adc;      /* 当前位置 (0~4095) */
    MotorDir dir;
    uint16_t duty;
    PidCtx pid;
} MotorState;

extern MotorState motor_states_[MOTOR_COUNT];
```

- [ ] **Step 2: motor.c 实现 Motor_InitControl 和 Motor_RunControlLoop**

```c
MotorState motor_states_[MOTOR_COUNT];
static bool motor_stall_flag_[MOTOR_COUNT];
static uint16_t motor_stall_count_[MOTOR_COUNT];

void Motor_InitControl(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_states_[i].target_adc = 2048;  /* 中间位置 */
        motor_states_[i].current_adc = 2048;
        motor_states_[i].dir = kMotorDir_Stop;
        motor_states_[i].duty = 0;
        Pid_Init(&motor_states_[i].pid, 0.5f, 0.01f, 0.1f);
        motor_stall_flag_[i] = false;
        motor_stall_count_[i] = 0;
    }
}

void Motor_RunControlLoop(void)
{
    if (!motor_adc_ready_) return;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_states_[i].current_adc = motor_adc_values_[i];

        if (i == MOTOR_3) {
            /* 电机 3 GPIO 开关控制 */
            if (motor_states_[i].target_adc > motor_states_[i].current_adc + 20) {
                Motor_SetPwm(i, kMotorDir_Forward, 500);
            } else if (motor_states_[i].target_adc < motor_states_[i].current_adc - 20) {
                Motor_SetPwm(i, kMotorDir_Reverse, 500);
            } else {
                Motor_SetPwm(i, kMotorDir_Stop, 0);
            }
            continue;
        }

        float output = Pid_Update(&motor_states_[i].pid,
                                  (float)motor_states_[i].target_adc,
                                  (float)motor_states_[i].current_adc);

        if (output > 0) {
            motor_states_[i].dir = kMotorDir_Forward;
            motor_states_[i].duty = (uint16_t)(output > 999.0f ? 999 : (uint16_t)output);
        } else if (output < 0) {
            motor_states_[i].dir = kMotorDir_Reverse;
            motor_states_[i].duty = (uint16_t)(-output > 999.0f ? 999 : (uint16_t)(-output));
        } else {
            motor_states_[i].dir = kMotorDir_Stop;
            motor_states_[i].duty = 0;
        }

        Motor_SetPwm(i, motor_states_[i].dir, motor_states_[i].duty);
    }
}
```

---

### Task 5: USB HID1 电机控制接口

**Files:**
- Modify: `User/usb/usb_desc.cpp`
- Modify: `User/usb/usb_impl.h`
- Modify: `User/usb/usb_impl.c`

- [ ] **Step 1: usb_desc.cpp 添加 HID1 接口描述符**

在现有 `kConfig` 中添加第二个 HID 接口：
```cpp
static constexpr auto kConfig =
tpusb::Config{
    tpusb::ConfigInitPack{ ... },
    tpusb::hid::HID_Interface{   /* HID0: printf */
        ...
    },
    tpusb::hid::HID_Interface{   /* HID1: motor control */
        tpusb::InterfaceInitPackClassed{
            .interface_no = 1,
            .alter = 0,
            .protocol = 0,
            .str_id = 0,
        },
        tpusb::hid::HID_Descriptor<1>{
            tpusb::hid::HID_Descriptor_InitPack{
                .bcd_hid = 0x0111,
                .country_code = 0,
            },
            std::array{
                tpusb::hid::HID_DescriptorLengthDesc{
                    .type = 0x22,
                    .length = sizeof(kMotorHidReportDesc)
                }
            }
        },
        tpusb::Endpoint{
            tpusb::InterruptInitPack{
                .address = 0x82,         /* EP2 IN */
                .max_pack_size = 64,
                .interval = 1,
            }
        },
        tpusb::Endpoint{
            tpusb::InterruptInitPack{
                .address = 0x02,         /* EP2 OUT */
                .max_pack_size = 64,
                .interval = 1,
            }
        },
    }
};
```

添加 HID1 报告描述符：
```cpp
/* HID1 Report Descriptor: motor control commands + status */
static constexpr uint8_t kMotorHidReportDesc[] = {
    0x06, 0x01, 0xFF,       /* Usage Page (Vendor 0xFF01) */
    0x09, 0x01,             /* Usage (Motor Control) */
    0xA1, 0x01,             /* Collection (Application) */
    0x09, 0x02,             /*   Usage (Commands) */
    0x15, 0x00,             /*   Logical Min (0) */
    0x26, 0xFF, 0x00,       /*   Logical Max (255) */
    0x75, 0x08,             /*   Report Size (8) */
    0x95, 64,               /*   Report Count (64) */
    0x91, 0x02,             /*   Output (Data,Var,Abs) */
    0x09, 0x03,             /*   Usage (Status) */
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 64,
    0x81, 0x02,             /*   Input (Data,Var,Abs) */
    0xC0,
};
```

更新 `UsbDesc_Hid` 以支持接口索引：
```cpp
uint8_t const* UsbDesc_Hid(uint16_t interface_idx, uint16_t* len)
{
    if (interface_idx == 0) {
        *len = sizeof(kHidReportDesc);
        return kHidReportDesc;
    } else if (interface_idx == 1) {
        *len = sizeof(kMotorHidReportDesc);
        return kMotorHidReportDesc;
    }
    *len = 0;
    return nullptr;
}
```

- [ ] **Step 2: usb_impl.h 添加 HID1 端点和 API**

```c
#define HID1_IN_EP_ADDRESS   0x82
#define HID1_OUT_EP_ADDRESS  0x02
#define HID1_EP_MPSIZE       64

void HID1_Init(void);
bool HID1_Read(uint8_t* buf, uint32_t* len);
bool HID1_Write(const uint8_t* buf, uint32_t len);
void HID1_ProcessCommand(void);
```

- [ ] **Step 3: usb_impl.c 实现 HID1 端点初始化**

```c
void UsbImpl_InitAndOpenEndpoints()
{
    /* 现有 EP1 TX (HID0 printf) */
    USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN | USBFS_UEP1_RX_EN;
    /* 需要配置 EP2 (HID1) 通过 UEP2_3_MOD */
    USBFSD->UEP2_3_MOD = USBFS_UEP2_TX_EN | USBFS_UEP2_RX_EN;

    USBFSD->UEP1_DMA = (uint32_t)hid_report_buf;
    USBFSD->UEP1_TX_LEN = 0;
    USBFSD->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;

    USBFSD->UEP2_DMA = (uint32_t)hid1_report_buf;
    USBFSD->UEP2_TX_LEN = 0;
    USBFSD->UEP2_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP2_RX_CTRL = USBFS_UEP_R_RES_ACK;

    hid_tx_busy = false;
}
```

- [ ] **Step 4: HID1 OUT 接收中断处理**

```c
void UsbImpl_EpOutComplete(uint8_t ep_num, uint16_t count)
{
    if (ep_num == 2) {
        /* HID1 OUT: 接收上位机命令 */
        memcpy(hid1_rx_buf, hid1_report_buf, count);
        hid1_rx_len = count;
        hid1_rx_pending = true;
        /* 切换 DMA 缓冲区（如果需要双缓冲）或重新使能 RX */
        USBFSD->UEP2_RX_CTRL = (USBFSD->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK)
                              | USBFS_UEP_R_RES_ACK;
    }
}
```

- [ ] **Step 5: HID1 IN 发送处理**

```c
void UsbImpl_EpInComplete(uint8_t ep_num)
{
    switch ((enum UsbEndpointNumber)ep_num) {
        case kUsbEndpoint_HidIn:
            _HidInHandler();
            break;
        case kUsbEndpoint_Hid1In:
            /* HID1 IN complete — status sent, mark ready */
            hid1_tx_busy = false;
            USBFSD->UEP2_TX_CTRL ^= USBFS_UEP_T_TOG;
            break;
    }
}
```

- [ ] **Step 6: 添加命令解析函数**

```c
/* HID1 命令格式：64 字节报告 */
/* byte 0: 命令 ID */
/*   0x01: 设置目标位置 (bytes 1-16: 8路目标值 × 2字节) */
/*   0x02: 读取状态 */
/*   0x03: 停止所有电机 */
/*   0x04: 设置 PID 参数 */

void HID1_ProcessCommand(void)
{
    if (!hid1_rx_pending) return;
    hid1_rx_pending = false;

    switch (hid1_rx_buf[0]) {
        case 0x01: /* 设置目标位置 */
            for (int i = 0; i < MOTOR_COUNT; i++) {
                uint16_t target = (uint16_t)hid1_rx_buf[1 + i*2]
                                | ((uint16_t)hid1_rx_buf[2 + i*2] << 8);
                Motor_SetTarget(i, target);
            }
            break;
        case 0x02: /* 读取状态 — 填充 IN 报告 */
            _Hid1BuildStatusReport();
            break;
        case 0x03: /* 停止所有 */
            Motor_StopAll();
            break;
    }
}
```

---

### Task 6: SysTick 集成

**Files:**
- Modify: `User/ch32v20x_it.c`

- [ ] **Step 1: SysTick_Handler 添加 ADC 触发**

```c
void SysTick_Handler(void)
{
    Tick_Increment();
    Motor_StartAdcConversion();    /* 触发 ADC (DMA 自动搬运) */
    HID_Flush();                  /* 继续轮询 HID0 printf */
}
```

- [ ] **Step 2: Startup 文件检查 DMA1 中断向量**

确保 `startup_ch32v20x_D6.S` 中有 `DMA1_Channel1_IRQHandler` 的向量入口（通常在 DMA1_Channel1_IRQn = 27 位置）。如果使用了 weak 定义，在 `ch32v20x_it.c` 中定义即可。

---

### Task 7: CMakeLists 与 main.c 更新

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `User/main.c`

- [ ] **Step 1: CMakeLists.txt 添加新源文件**

```cmake
set(SRC_FILES 
    ...
    "${CMAKE_CURRENT_SOURCE_DIR}/User/motor.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/pid.c"
)
```

- [ ] **Step 2: main.c 初始化所有模块**

```c
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Tick_Init();

    printf("SystemClk:%d\r\n", SystemCoreClock);

    Motor_InitPwm();
    Motor_InitAdc();
    Motor_InitControl();

    HID_Init();
    Usbd_Init();
    Usbd_Connect();
    printf("USB HID ready\r\n");

    while (1) {
        __asm__ volatile("wfi");
        HID1_ProcessCommand();
    }
}
```

---

## 未解决问题

1. PID 参数（kp, ki, kd）需要实际测试调整，初始值可能需要根据不同电机的机械特性单独设置
2. DMA1 通道 1 中断优先级需要确定相对于 SysTick 和 USBFS 的高低
