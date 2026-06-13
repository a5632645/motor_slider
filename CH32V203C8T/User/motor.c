#include "motor.h"
#include "ch32v20x.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_misc.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_tim.h"
#include "config.h"
#include "pid.h"
#include "usb/usb_impl.h"


/* ADC DMA 缓冲区 */
__attribute__((aligned(4))) volatile uint16_t motor_adc_dma_buf_[MOTOR_COUNT];
volatile bool motor_adc_ready_;

/* 电机状态数组 */
struct MotorState motor_states_[MOTOR_COUNT];

/*********************************************************************
 * @fn      Motor_InitPwm
 *
 * @brief   初始化 TIM1~TIM4 为 PWM 输出，电机 3 配置为 GPIO 控制
 *          所有定时器：ARR=999, 预分频=0 → 96kHz
 *
 * @return  none
 */
void Motor_InitPwm(void) {
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* === TIM1: 电机 4 (CH2=PA9/A, CH1=PA8/B), 电机 5 (CH3=PA10/A, CH4=PA11/B) === */
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 999;
    tim.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM1, &oc); /* CH1=PA8  — 电机4 B */
    TIM_OC2Init(TIM1, &oc); /* CH2=PA9  — 电机4 A */
    TIM_OC3Init(TIM1, &oc); /* CH3=PA10 — 电机5 A */
    TIM_OC4Init(TIM1, &oc); /* CH4=PA11 — 电机5 B */

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /* === TIM2: 电机 6 (CH1=PA15/A, CH2=PB3/B), 电机 2 (CH3=PB10/B, CH4=PB11/A) === */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

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

    TIM_OC1Init(TIM2, &oc); /* PA15 — 电机6 A */
    TIM_OC2Init(TIM2, &oc); /* PB3  — 电机6 B */
    TIM_OC3Init(TIM2, &oc); /* PB10 — 电机2 B */
    TIM_OC4Init(TIM2, &oc); /* PB11 — 电机2 A */

    TIM_Cmd(TIM2, ENABLE);

    /* === TIM3: 电机 7 (CH1=PB4/A, CH2=PB5/B), 电机 1 (CH3=PB0/A, CH4=PB1/B) === */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

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

    TIM_OC1Init(TIM3, &oc); /* PB4 — 电机7 A */
    TIM_OC2Init(TIM3, &oc); /* PB5 — 电机7 B */
    TIM_OC3Init(TIM3, &oc); /* PB0 — 电机1 A */
    TIM_OC4Init(TIM3, &oc); /* PB1 — 电机1 B */

    TIM_Cmd(TIM3, ENABLE);

    /* === TIM4: 电机 8 (CH3=PB8/A, CH4=PB9/B) === */
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

    TIM_OC3Init(TIM4, &oc); /* PB8 — 电机8 A */
    TIM_OC4Init(TIM4, &oc); /* PB9 — 电机8 B */

    TIM_Cmd(TIM4, ENABLE);

    /* === 电机 3: GPIO PB14, PB15 === */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
}

/*********************************************************************
 * @fn      Motor_InitAdc
 *
 * @brief   初始化 ADC1 常规组 8 通道 scan + DMA1 通道 1
 *          ADC 时钟 = PCLK2/8 = 12MHz, 12bit
 *          软件触发，DMA 自动搬运结果到 motor_adc_dma_buf_
 *
 * @return  none
 */
void Motor_InitAdc(void) {
    ADC_InitTypeDef adc;
    GPIO_InitTypeDef gpio;
    DMA_InitTypeDef dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* PA0~PA7 模拟输入 */
    gpio.GPIO_Pin =
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
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

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    /* === ADC1 配置 === */
    ADC_DeInit(ADC1);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8); /* ADC 时钟 = 12MHz */

    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = ENABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = MOTOR_COUNT;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_7Cycles5);

    ADC_DMACmd(ADC1, ENABLE);

    /* DMA NVIC 中断配置 */
    {
        NVIC_InitTypeDef nvic;
        nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
        nvic.NVIC_IRQChannelCmd = ENABLE;
        nvic.NVIC_IRQChannelPreemptionPriority = 1;
        nvic.NVIC_IRQChannelSubPriority = 0;
        NVIC_Init(&nvic);
    }

    motor_adc_ready_ = true;

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
        ;
}

/*********************************************************************
 * @fn      Motor_StartAdcConversion
 *
 * @brief   软件触发 ADC 转换，复位 DMA 计数器后使能
 *
 * @return  none
 */
void Motor_StartAdcConversion(void) {
    /* 上次转换未完成，跳过本次触发 */
    if (!motor_adc_ready_)
        return;

    motor_adc_ready_ = false;
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA1_Channel1->CNTR = MOTOR_COUNT;
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*********************************************************************
 * @fn      Motor_SetPwm
 *
 * @brief   设置指定电机的方向和 PWM 占空比
 *
 * @param   ch    电机序号 (MOTOR_1 ~ MOTOR_8)
 * @param   dir   方向
 * @param   duty  占空比 (0~999)
 *
 * @return  none
 *
 * @note    电机 1~2, 4~8 使用硬件 PWM 定时器
 *          电机 3 使用 GPIO 软件控制
 */
void Motor_SetPwm(uint8_t ch, enum MotorDir dir, uint16_t duty) {
    if (duty > 999)
        duty = 999;

    if (ch == MOTOR_3) {
        if (dir == kMotorDir_Reverse) {
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        }
        else if (dir == kMotorDir_Forward) {
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
        }
        else if (dir == kMotorDir_Brake) {
            GPIO_SetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
        }
        else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
        }
        return;
    }

    switch (ch) {
        case MOTOR_1: /* TIM3 CH3=IN1, CH4=IN2 */
            TIM_SetCompare4(TIM3, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM3, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_2: /* TIM2 CH4=IN1, CH3=IN2 */
            TIM_SetCompare4(TIM2, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM2, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_4: /* TIM1 CH2=IN1, CH1=IN2 */
            TIM_SetCompare2(TIM1, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare1(TIM1, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_5: /* TIM1 CH3=IN1, CH4=IN2 */
            TIM_SetCompare4(TIM1, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM1, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_6: /* TIM2 CH1=IN1, CH2=IN2 */
            TIM_SetCompare2(TIM2, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare1(TIM2, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_7: /* TIM3 CH1=IN1, CH2=IN2 */
            TIM_SetCompare2(TIM3, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare1(TIM3, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case MOTOR_8: /* TIM4 CH3=IN1, CH4=IN2 */
            TIM_SetCompare4(TIM4, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM4, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
    }
}

/*********************************************************************
 * @fn      DMA1_Channel1_IRQHandler
 *
 * @brief   DMA1 通道 1 传输完成中断 — ADC 数据就绪
 *          由 Motor_RunControlLoop 消费
 *
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void DMA1_Channel1_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC1)) {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        motor_adc_ready_ = true;
    }
}

/*********************************************************************
 * @fn      Motor_InitControl
 *
 * @brief   初始化所有电机状态和 PID 参数
 *
 * @return  none
 */
void Motor_InitControl(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_states_[i].target_adc_ = 2048;
        motor_states_[i].current_adc_ = 2048;
        motor_states_[i].dir_ = kMotorDir_Stop;
        motor_states_[i].duty_ = 0;
        motor_states_[i].active_ = false;
        motor_states_[i].timeout_ = 0;
        Pid_Init(&motor_states_[i].pid_, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD);
    }
}

/*********************************************************************
 * @fn      Motor_RunControlLoop
 *
 * @brief   主控制循环：读取 ADC 值 → 8路 PID → 更新 PWM
 *          在 DMA1 完成中断中执行
 *
 * @return  none
 */
void Motor_RunControlLoop(void) {
    if (!motor_adc_ready_)
        return;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        /* ADC 物理通道与电机序号反序 (PCB 布局) */
        uint8_t mi = (uint8_t)(MOTOR_COUNT - 1 - i);
        motor_states_[mi].current_adc_ = motor_adc_dma_buf_[i];

        /* 非活跃电机：跳过，PWM 保持 0 */
        if (!motor_states_[mi].active_)
            continue;

        /* 判断是否到达目标 */
        int16_t diff = (int16_t)(motor_states_[mi].target_adc_ - motor_states_[mi].current_adc_);
        int16_t abs_diff = (diff < 0) ? -diff : diff;
        if (abs_diff <= CTRL_ERROR_THRESHOLD) {
            motor_states_[mi].active_ = false;
            motor_states_[mi].duty_ = 0;
            Motor_SetPwm(mi, kMotorDir_Stop, 0);
            continue;
        }

        /* 超时判断 */
        if (motor_states_[mi].timeout_ > 0) {
            motor_states_[mi].timeout_--;
        }
        if (motor_states_[mi].timeout_ == 0) {
            motor_states_[mi].active_ = false;
            motor_states_[mi].duty_ = 0;
            Motor_SetPwm(mi, kMotorDir_Stop, 0);
            continue;
        }

        /* ── 闭环控制 ── */
        if (mi == MOTOR_3) {
            /* 电机 3: GPIO 开关控制 */
            if (abs_diff > MOTOR3_DEADBAND) {
                if (diff > 0) {
                    Motor_SetPwm(mi, kMotorDir_Forward, 500);
                }
                else {
                    Motor_SetPwm(mi, kMotorDir_Reverse, 500);
                }
            }
            else {
                Motor_SetPwm(mi, kMotorDir_Stop, 0);
            }
            continue;
        }

        /* 电机 1~2, 4~8: PID 控制 */
        float output = Pid_Update(&motor_states_[mi].pid_, (float)motor_states_[mi].target_adc_,
                                  (float)motor_states_[mi].current_adc_);

        if (output > 0) {
            motor_states_[mi].dir_ = kMotorDir_Forward;
            uint16_t raw = (uint16_t)(output > 999.0f ? 999 : (uint16_t)output);
            motor_states_[mi].duty_ = (raw < PWM_MIN_START_DUTY) ? (uint16_t)PWM_MIN_START_DUTY : raw;
        }
        else if (output < 0) {
            motor_states_[mi].dir_ = kMotorDir_Reverse;
            uint16_t raw = (uint16_t)(-output > 999.0f ? 999 : (uint16_t)(-output));
            motor_states_[mi].duty_ = (raw < PWM_MIN_START_DUTY) ? (uint16_t)PWM_MIN_START_DUTY : raw;
        }
        else {
            motor_states_[mi].dir_ = kMotorDir_Stop;
            motor_states_[mi].duty_ = 0;
        }

        Motor_SetPwm(mi, motor_states_[mi].dir_, motor_states_[mi].duty_);
    }
}

/*********************************************************************
 * @fn      Motor_SetTarget
 *
 * @brief   设置指定电机的目标位置
 *
 * @param   ch          电机序号
 * @param   target_adc  目标 ADC 值 (0~4095)
 *
 * @return  none
 */
void Motor_SetTarget(uint8_t ch, uint16_t target_adc) {
    if (ch < MOTOR_COUNT) {
        motor_states_[ch].target_adc_ = target_adc;
        motor_states_[ch].active_ = true;
        motor_states_[ch].timeout_ = CTRL_TIMEOUT_MS;
        Pid_Reset(&motor_states_[ch].pid_);
    }
}

/*********************************************************************
 * @fn      Motor_StopAll
 *
 * @brief   停止所有电机运动
 *
 * @return  none
 */
void Motor_StopAll(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_states_[i].active_ = false;
        motor_states_[i].timeout_ = 0;
        Motor_SetPwm(i, kMotorDir_Stop, 0);
    }
}

/*********************************************************************
 * @fn      Motor_SetPid
 *
 * @brief   设置指定电机的 PID 参数 (运行时更新)
 *
 * @param   ch  电机序号 (0~7), 0xFF=全部
 * @param   kp  比例增益
 * @param   ki  积分增益
 * @param   kd  微分增益
 *
 * @return  none
 */
void Motor_SetPid(uint8_t ch, float kp, float ki, float kd) {
    if (ch == 0xFF) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            Pid_Init(&motor_states_[i].pid_, kp, ki, kd);
        }
    }
    else if (ch < MOTOR_COUNT) {
        Pid_Init(&motor_states_[ch].pid_, kp, ki, kd);
    }
}

/*********************************************************************
 * @fn      Motor_IsAdcReady
 *
 * @brief   检查 ADC 数据是否就绪
 *
 * @return  true 就绪, false 忙
 */
bool Motor_IsAdcReady(void) {
    return motor_adc_ready_;
}

/*********************************************************************
 * @fn      Motor_GetStatus
 *
 * @brief   导出 8 路电机状态，供 HID1 上报使用
 *
 * @param   adc    输出当前 ADC 值
 * @param   target 输出目标 ADC 值
 * @param   duty   输出当前占空比
 *
 * @return  none
 */
void Motor_GetStatus(uint16_t adc[8], uint16_t target[8], uint16_t duty[8], uint8_t* active_flags) {
    uint8_t flags = 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        adc[i] = motor_states_[i].current_adc_;
        target[i] = motor_states_[i].target_adc_;
        duty[i] = motor_states_[i].duty_;
        if (motor_states_[i].active_)
            flags |= (uint8_t)(1u << i);
    }
    if (active_flags)
        *active_flags = flags;
}
