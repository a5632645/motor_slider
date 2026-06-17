#pragma once

#include "bsp/motor_hw.h"
#include <stdbool.h>
#include <stdint.h>

#define MIDI_CIN_CC          0x0B
#define MIDI_DEFAULT_CHANNEL 0

struct MidiCcMapping {
    uint8_t channel;
    uint8_t cc_num;
};

/**
 * @brief 初始化 MIDI CC 状态
 */
void MidiCC_Init(void);

/**
 * @brief 推进 MIDI CC 发送状态机
 */
void MidiCC_TryTxCC(void);

/**
 * @brief 处理 USB MIDI RX 缓冲中的 CC 消息
 */
void MidiCC_ProcessRx(void);

/**
 * @brief 接收控制环 ADC 数据并更新 MIDI CC 采样窗口
 * @param raw_adc 控制环刚读取到的原始 ADC 数组
 */
void MidiCC_UpdateAdc(uint16_t raw_adc[kMotorIdx_Count]);
