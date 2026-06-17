#pragma once

#include "usb_device.h"

/**
 * @brief 初始化并打开 USB 端点
 */
void UsbImpl_InitAndOpenEndpoints();

/**
 * @brief 处理 USB HID 类请求
 */
void UsbImpl_HandleClassRequest(struct UsbDevice* device, bool* allow, bool setup_phase);

/**
 * @brief 处理厂商请求
 */
void UsbImpl_HandleVendorRequest(struct UsbDevice* device, bool* allow, bool setup_phase);

/**
 * @brief 帧起始回调
 */
void UsbImpl_HandleSof();

/**
 * @brief 设置接口备选配置
 */
void UsbImpl_SetInterfaceAlter(uint8_t interface, uint8_t alter, bool* allow);

/**
 * @brief 获取接口当前备选配置
 * @return 当前备选值
 */
uint8_t UsbImpl_GetInterfaceAlter(uint8_t interface, bool* allow);

/**
 * @brief 强制 STALL 指定端点
 */
void UsbImpl_StallEndpoint(uint8_t address);

/**
 * @brief 清除指定端点的 STALL 状态
 */
void UsbImpl_ClearStallEndpoint(uint8_t address);

/**
 * @brief 获取 USB 描述符
 */
void UsbImpl_GetDescriptor(struct UsbDevice* device, bool* allow);

/**
 * @brief IN 端点发送完成中断回调
 */
void UsbImpl_EpInComplete(uint8_t ep_num);

/**
 * @brief OUT 端点接收完成中断回调
 */
void UsbImpl_EpOutComplete(uint8_t ep_num, uint16_t count);

enum {
    kHidEpAddr_In = 0x81,
    kHidEpMpsize = 64,
    kHidReportSize = 64,
    kHid1EpAddr_In = 0x82,
    kHid1EpAddr_Out = 0x03,
    kHid1EpMpsize = 64,

    kMidiEpAddr_In = 0x84,
    kMidiEpAddr_Out = 0x05,
    kMidiEpMpsize = 64,
};

enum {
    kUsbInterface_HidDebug = 0,
    kUsbInterface_HidMotor,
    kUsbInterface_AudioControl,
    kUsbInterface_MidiStreaming,
    kUsbInterface_Count
};

/* HID1 64 字节状态报告格式偏移 */
#define HID1_STATUS_FLAGS 0              /* uint8, bit0=running */
#define HID1_ACTIVE_FLAGS 1              /* uint8, 每路 1 bit active */
#define HID1_ADC(i)       (2 + (i) * 2)  /* uint16 LE, 8路 */
#define HID1_TARGET(i)    (18 + (i) * 2) /* uint16 LE, 8路 */
#define HID1_DUTY(i)      (34 + (i) * 2) /* uint16 LE, 8路 */

// ----- hid debug -----

/**
 * @brief 将数据写入 HID0 printf FIFO
 * @param data 数据指针
 * @param len  数据长度
 * @return 实际写入的字节数
 */
uint32_t HID_Write(const uint8_t* data, uint32_t len);

/**
 * @brief 检查 HID0 缓冲区是否可写入
 * @return true 可写入
 */
bool HID_CanWrite(void);

/**
 * @brief 刷新 HID0 FIFO 数据到 USB
 */
void HID_Flush(void);

/**
 * @brief 检查 USB 是否已连接
 * @return true 已连接
 */
bool HID_IsConnected(void);

// ----- hid motor -----

/**
 * @brief 读取 HID1 OUT 报告
 * @param bytes 输出缓冲区 (kHidReportSize 字节)
 * @return true 有新的 OUT 报告
 */
bool HID1_Read(uint8_t bytes[kHidReportSize]);

/**
 * @brief 检查 HID1 IN 端点是否空闲可发送
 * @return true 可发送
 */
bool HID1_IsTxReady(void);

/**
 * @brief 发送 HID1 IN 报告
 * @param bytes 待发送的 64 字节报告
 */
void HID1_Write(uint8_t bytes[kHidReportSize]);

// ----- midi -----

/**
 * @brief 轮询 MIDI USB 收发状态机
 */
void Midi_Poll(void);

/**
 * @brief 写入一个 4 字节 USB-MIDI Event Packet
 * @param pack 4 字节 MIDI 事件包
 * @return true 写入成功
 */
bool Midi_Push(uint8_t pack[4]);

/**
 * @brief 获取待处理的 MIDI OUT 数据
 * @param len 输出数据长度
 * @return 数据缓冲区指针，无数据时返回 NULL
 */
uint8_t const* Midi_GetRxBuffer(uint32_t* len);

/**
 * @brief 标记 MIDI OUT 数据已处理，并重新打开 OUT 端点
 */
void Midi_SetRxReady(void);

/**
 * @brief 获取 MIDI IN 完成计数
 * @return 完成次数
 */
uint32_t Midi_GetTxDoneCount(void);

/**
 * @brief 获取 MIDI IN 超时恢复计数
 * @return 超时次数
 */
uint32_t Midi_GetTxTimeoutCount(void);

/**
 * @brief 获取 MIDI OUT 接收计数
 * @return 接收次数
 */
uint32_t Midi_GetRxDoneCount(void);

/**
 * @brief 获取 MIDI OUT 溢出计数
 * @return 溢出次数
 */
uint32_t Midi_GetRxOverflowCount(void);
