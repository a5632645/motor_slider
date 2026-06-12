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

#define HID_IN_EP_ADDRESS    0x81
#define HID_IN_EP_MPSIZE     64
#define HID_REPORT_SIZE      64

/* HID1 — 电机控制接口 */
#define HID1_IN_EP_ADDRESS   0x82
#define HID1_OUT_EP_ADDRESS  0x02
#define HID1_EP_MPSIZE       64

void HID_Init(void);
uint32_t HID_Write(const uint8_t* data, uint32_t len);
bool HID_CanWrite(void);
void HID_Flush(void);
bool HID_IsConnected(void);

void HID1_Init(void);
bool HID1_Read(uint8_t* buf, uint32_t* len);
void HID1_ProcessCommand(void);
