#include "app/midi_cc.h"

#include <string.h>

#include "app/motor.h"
#include "config.h"
#include "usb/usb_hardware.h"
#include "usb/usb_impl.h"
#include "util/kfifo.h"

// ------------------------------------------------------------
// CC 映射表: 电机索引 → MIDI CC
// ------------------------------------------------------------

const struct MidiCcMapping kMidiCcMap[kMotorIdx_Count] = {
    {0, 1}, // CH1 → Ch1 CC1
    {0, 2}, // CH2 → Ch1 CC2
    {0, 3}, // CH3 → Ch1 CC3
    {0, 4}, // CH4 → Ch1 CC4
    {0, 5}, // CH5 → Ch1 CC5
    {0, 6}, // CH6 → Ch1 CC6
    {0, 7}, // CH7 → Ch1 CC7
    {0, 8}, // CH8 → Ch1 CC8
};

static uint16_t _MidiCcToTargetAdc(uint8_t cc_value) {
    uint32_t target = ((uint32_t)(cc_value * 2 + 1) * 4095) / (2 * 127);
    if (target > 4095) {
        target = 4095;
    }
    return (uint16_t)target;
}

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void MidiCC_Init(void) {
}

void MidiCC_ProcessRx(void) {
    uint32_t len;
    uint8_t const* buf = Midi_GetRxBuffer(&len);
    if (len == 0) {
        return;
    }

    // 解析 4 字节 MIDI Event Packet
    for (uint32_t i = 0; i + 4 <= len; i += 4) {
        uint8_t status = buf[i + 1];
        uint8_t cc_num = buf[i + 2];
        uint8_t cc_value = buf[i + 3];

        // 仅处理 CC 消息 (0xB0)
        if ((status & 0xF0) != 0xB0)
            continue;

        uint8_t channel = status & 0x0F;

        // 匹配映射表
        for (int j = 0; j < kMotorIdx_Count; j++) {
            if (kMidiCcMap[j].channel == channel && kMidiCcMap[j].cc_num == cc_num) {
                // CC (0-127) → ADC (0-4095), 取量化区间中心
                uint16_t target_adc = _MidiCcToTargetAdc(cc_value);
                App_OnMidiCcRx(j, cc_value, target_adc);
                break;
            }
        }
    }

    Midi_SetRxReady();
}
