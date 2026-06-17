/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Tick counter, HID printf backend, microsecond delay.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "debug.h"
#include "usb/usb_impl.h"

/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond Delay Time (busy-wait loop).
 *
 * @param   n - Microsecond number.
 *
 * @return  None
 */
void Delay_Us(uint32_t n)
{
    uint32_t i = n * (SystemCoreClock / 8000000);
    while (i--)
    {
        __asm__ volatile("nop");
    }
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Printf backend — writes to HID buffer. Blocks when buffer
 *          is full, waiting for SysTick to drain it.
 *
 * @param   buf  - data buffer
 *          size - number of bytes
 *
 * @return  number of bytes written
 */
__attribute__((used))
int _write(int fd, char *buf, int size)
{
    if (!HID_IsConnected())
    {
        return size;
    }

    uint8_t *p = (uint8_t *)buf;
    int remaining = size;

    while (remaining > 0)
    {
        int written = (int)HID_Write(p, (uint32_t)remaining);
        p += written;
        remaining -= written;
        HID_Flush();
    }
    return size;
}

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}
