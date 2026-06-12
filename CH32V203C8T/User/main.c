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
        if (motor_adc_ready_) {
            Motor_RunControlLoop();
        }
        HID1_ProcessCommand();
    }
}
