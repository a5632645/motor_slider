#include "bsp.h"

#include "ch32v20x_dbgmcu.h"
#include "ch32v20x_misc.h"
#include "system_ch32v20x.h"

#include "bsp/motor_hw.h"
#include "bsp/tick.h"
#include "bsp/usbd.h"

#include "usb/usb_impl.h"

#include <stdio.h>

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void Bsp_Init(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Tick_Init();

    /* Init motor control */
    MotorHw_Init();

    /* Init USB HID device */
    Usbd_Init();
    Usbd_Connect();

    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
    printf("USB HID ready\r\n");
}
