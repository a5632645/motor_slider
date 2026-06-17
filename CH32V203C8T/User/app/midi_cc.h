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

void MidiCC_Init(void);
void MidiCC_ProcessRx(void);

extern void App_OnMidiCcRx(uint8_t idx, uint8_t cc_value, uint16_t target_adc);
