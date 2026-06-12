#pragma once

#include "usb_device.h"

void UsbImpl_InitAndOpenEndpoints();

void UsbImpl_HandleClassRequest(struct UsbDevice* device, bool* allow, bool setup_phase);
void UsbImpl_HandleVendorRequest(struct UsbDevice* device, bool* allow, bool setup_phase);
void UsbImpl_HandleSof();

void UsbImpl_SetInterfaceAlter(uint8_t interface, uint8_t alter, bool* allow);
uint8_t UsbImpl_GetInterfaceAlter(uint8_t interface, bool* allow);

void UsbImpl_StallEndpoint(uint8_t address);
void UsbImpl_ClearStallEndpoint(uint8_t address);

void UsbImpl_GetDescriptor(struct UsbDevice* device, bool* allow);

void UsbImpl_EpInComplete(uint8_t ep_num);
void UsbImpl_EpOutComplete(uint8_t ep_num, uint16_t count);

/* 端点号枚举 */
enum UsbEndpointNumber {
    kUsbEndpoint_Control = 0,
    kUsbEndpoint_HidIn,
    kUsbEndpoint_Hid1In = 2,
    kUsbEndpoint_Hid1Out = 3,
};

/* HID0/HID1 端点常量 */
enum {
    kHidEpAddr_In = 0x81,
    kHidEpMpsize = 64,
    kHidReportSize = 64,
    kHid1EpAddr_In = 0x82,
    kHid1EpAddr_Out = 0x03,
    kHid1EpMpsize = 64,
};

/* HID1 64 字节状态报告格式偏移 */
#define HID1_STATUS_FLAGS 0              /* uint8, bit0=running */
#define HID1_ACTIVE_FLAGS 1              /* uint8, 每路 1 bit active */
#define HID1_ADC(i)       (2 + (i) * 2)  /* uint16 LE, 8路 */
#define HID1_TARGET(i)    (18 + (i) * 2) /* uint16 LE, 8路 */
#define HID1_DUTY(i)      (34 + (i) * 2) /* uint16 LE, 8路 */

void HID_Init(void);
uint32_t HID_Write(const uint8_t* data, uint32_t len);
bool HID_CanWrite(void);
void HID_Flush(void);
bool HID_IsConnected(void);

void HID1_Init(void);
bool HID1_Read(uint8_t* buf, uint32_t* len);
void HID1_ProcessCommand(void);

/**
 * @brief 填充 HID1 IN 报告并触发发送
 * @param adc         8 路当前 ADC 值 (0~4095)
 * @param target      8 路目标 ADC 值 (0~4095)
 * @param duty        8 路当前占空比 (0~999)
 * @param active_flags 每路 1 bit active 标志
 */
void HID1_SendStatus(const uint16_t adc[8], const uint16_t target[8], const uint16_t duty[8], uint8_t active_flags);
