/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/10/24
 * Description        : This file contains all the functions prototypes for UART
 *                      Printf , Delay functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#include "stdio.h"
#include "ch32v20x.h"

#ifdef __cplusplus
extern "C" {
#endif

void Delay_Us(uint32_t n);

#ifdef __cplusplus
}
#endif

#endif
