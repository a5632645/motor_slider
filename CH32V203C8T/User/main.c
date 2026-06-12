/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "usbd.h"
#include "usb/usb_impl.h"
#include "tick.h"
#include "motor.h"

/* 控制状态机 */
enum CtrlState {
    kCtrlState_Idle,
    kCtrlState_AdcStart,
    kCtrlState_AdcWait,
    kCtrlState_Control,
};

static enum CtrlState ctrl_state_ = kCtrlState_Idle;
static uint32_t last_ctrl_tick_ = 0;

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Tick_Init();

    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* Init motor control */
    Motor_InitPwm();
    Motor_InitAdc();
    Motor_InitControl();

    /* Init USB HID device */
    HID_Init();
    HID1_Init();
    Usbd_Init();
    Usbd_Connect();
    printf("USB HID ready\r\n");

    while (1)
    {
        /* ── 1kHz 控制状态机 ── */
        switch (ctrl_state_) {
        case kCtrlState_Idle:
            if (Tick_Get() != last_ctrl_tick_) {
                last_ctrl_tick_ = Tick_Get();
                ctrl_state_ = kCtrlState_AdcStart;
            }
            break;

        case kCtrlState_AdcStart:
            Motor_StartAdcConversion();
            ctrl_state_ = kCtrlState_AdcWait;
            break;

        case kCtrlState_AdcWait:
            if (Motor_IsAdcReady()) {
                ctrl_state_ = kCtrlState_Control;
            }
            break;

        case kCtrlState_Control:
            Motor_RunControlLoop();
            {
                uint16_t adc[8], target[8], duty[8];
                uint8_t active_flags;
                Motor_GetStatus(adc, target, duty, &active_flags);
                HID1_SendStatus(adc, target, duty, active_flags);
            }
            ctrl_state_ = kCtrlState_Idle;
            break;
        }

        /* ── HID1 命令处理（保持原位，每次迭代执行） ── */
        HID1_ProcessCommand();
    }
}
