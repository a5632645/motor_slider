#include "motor_hw.h"

#include "ch32v20x_adc.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_tim.h"
#include "ch32v20x_misc.h"

// ------------------------------------------------------------
// variable
// ------------------------------------------------------------

__attribute__((aligned(4)))
static volatile uint16_t motor_adc_dma_buf_[kMotorIdx_Count];
static volatile bool motor_adc_ready_;

// ------------------------------------------------------------
// private
// ------------------------------------------------------------

/**
 * @brief   初始化 TIM1~TIM4 为 PWM 输出，电机 3 配置为 GPIO 控制
 *          所有定时器：ARR=999, 预分频=0 → 96kHz
 */
void _InitPwm(void) {
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

/**
 * @brief   初始化 ADC1 常规组 8 通道 scan + DMA1 通道 1
 *          ADC 时钟 = PCLK2/8 = 12MHz, 12bit
 *          软件触发，DMA 自动搬运结果到 motor_adc_dma_buf_
 */
void _InitAdc(void) {
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
    dma.DMA_BufferSize = kMotorIdx_Count;
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
    adc.ADC_NbrOfChannel = kMotorIdx_Count;
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

// ------------------------------------------------------------
// irq
// ------------------------------------------------------------

__attribute__((interrupt("WCH-Interrupt-fast")))
void DMA1_Channel1_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC1)) {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        motor_adc_ready_ = true;
    }
}

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void MotorHw_Init(void) {
    _InitAdc();
    _InitPwm();
}

void MotorHw_StartAdcConversion(void) {
    /* 上次转换未完成，跳过本次触发 */
    if (!motor_adc_ready_)
        return;

    motor_adc_ready_ = false;
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA1_Channel1->CNTR = kMotorIdx_Count;
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

bool MotorHw_IsAdcReady(void) {
    return motor_adc_ready_;
}

void MotorHw_GetAdcValue(uint16_t buffer[kMotorIdx_Count]) {
    for (int i = 0; i < kMotorIdx_Count; ++i) {
        buffer[i] = motor_adc_dma_buf_[i];
    }
}

void MotorHw_SetPwm(uint8_t ch, enum MotorDir dir, uint16_t duty) {
    if (duty > 999)
        duty = 999;

    switch (ch) {
        case kMotorIdx_0: /* TIM4 CH3=IN1, CH4=IN2 */
            TIM_SetCompare4(TIM4, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM4, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case kMotorIdx_1: /* TIM3 CH1=IN1, CH2=IN2 */
            TIM_SetCompare2(TIM3, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare1(TIM3, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case kMotorIdx_2: /* TIM2 CH1=IN1, CH2=IN2 */
            TIM_SetCompare2(TIM2, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare1(TIM2, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case kMotorIdx_3: /* TIM1 CH3=IN1, CH4=IN2 */
            TIM_SetCompare4(TIM1, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM1, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case kMotorIdx_4: /* TIM1 CH2=IN1, CH1=IN2 */
            TIM_SetCompare2(TIM1, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare1(TIM1, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case kMotorIdx_5: /* GPIO PB14=IN1, PB15=IN2 */
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
            break;
        case kMotorIdx_6: /* TIM2 CH4=IN1, CH3=IN2 */
            TIM_SetCompare4(TIM2, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM2, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
        case kMotorIdx_7: /* TIM3 CH3=IN1, CH4=IN2 */
            TIM_SetCompare4(TIM3, (dir == kMotorDir_Forward || dir == kMotorDir_Brake) ? duty : 0);
            TIM_SetCompare3(TIM3, (dir == kMotorDir_Reverse || dir == kMotorDir_Brake) ? duty : 0);
            break;
    }
}
