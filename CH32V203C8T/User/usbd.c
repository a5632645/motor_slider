#include "usbd.h"

#include "ch32v20x_misc.h"
#include "ch32v20x_rcc.h"
#include "debug.h"
#include "usb/usb_device.h"
#include "usb/usb_hardware.h"


void Usbd_Init() {
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    if (clocks.SYSCLK_Frequency == 144000000) {
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div3);
    }
    else if (clocks.SYSCLK_Frequency == 96000000) {
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);
    }
    else /* 48000000 or others */
    {
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
    }
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);

    UsbDevice_Init();
}

void Usbd_Connect() {
    USBFSD->BASE_CTRL |= USBFS_UC_DEV_PU_EN;

    NVIC_InitTypeDef nvic = {.NVIC_IRQChannel = USBFS_IRQn,
                             .NVIC_IRQChannelCmd = ENABLE,
                             .NVIC_IRQChannelPreemptionPriority = 0,
                             .NVIC_IRQChannelSubPriority = 0};
    NVIC_Init(&nvic);
}

void Usbd_DisConnect() {
    USBFSD->BASE_CTRL = USBFS_UC_CLR_ALL | USBFS_UC_RESET_SIE;
    Delay_Us(10);
    USBFSD->BASE_CTRL &= ~USBFS_UC_RESET_SIE;
    NVIC_DisableIRQ(USBFS_IRQn);
}
