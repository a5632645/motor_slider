#include "app.h"

#include "app/midi_cc.h"
#include "bsp/tick.h"
#include "config.h"
#include "motor.h"
#include "usb/usb_impl.h"

#include <stdint.h>

// ------------------------------------------------------------
// motor control
// ------------------------------------------------------------

// 控制状态机
enum CtrlState {
    kCtrlState_Idle,
    kCtrlState_AdcStart,
    kCtrlState_AdcWait,
    kCtrlState_Control,
};

static enum CtrlState ctrl_state_ = kCtrlState_Idle;
static uint64_t last_ctrl_us_ = 0;

static void _MotorControl(void) {
    switch (ctrl_state_) {
        case kCtrlState_Idle: {
            uint64_t now_us = Tick_GetUs();
            if ((uint64_t)(now_us - last_ctrl_us_) >= CTRL_LOOP_US) {
                last_ctrl_us_ = now_us;
                ctrl_state_ = kCtrlState_AdcStart;
            }
        } break;

        case kCtrlState_AdcStart:
            Motor_StartAdcConversion();
            ctrl_state_ = kCtrlState_AdcWait;
            break;

        case kCtrlState_AdcWait:
            if (Motor_IsAdcReady()) {
                ctrl_state_ = kCtrlState_Control;
            }
            break;

        case kCtrlState_Control:
            Motor_RunControlLoop();
            ctrl_state_ = kCtrlState_Idle;
            break;
    }
}

// ------------------------------------------------------------
// midi
// ------------------------------------------------------------

#define MIDI_ADC_FILTER_SHIFT 3
#define MIDI_CC_HYST_ADC     8

static uint16_t midi_raw_adc_[kMotorIdx_Count];
static bool midi_raw_adc_ready_;
static uint32_t midi_adc_sum_[kMotorIdx_Count];
static uint8_t midi_adc_avg_count_;
static bool midi_filter_ready_;
static uint32_t midi_filtered_adc_q8_[kMotorIdx_Count];
static uint8_t midi_last_sent_cc_[kMotorIdx_Count];
static uint8_t midi_candidate_cc_[kMotorIdx_Count];
static uint8_t midi_candidate_count_[kMotorIdx_Count];
static uint8_t midi_pending_cc_[kMotorIdx_Count];
static uint8_t midi_pending_mask_;
static uint16_t midi_rx_hold_adc_[kMotorIdx_Count];
static uint8_t midi_rx_hold_mask_;
static uint8_t midi_send_idx_;
static uint8_t midi_packet_[4];

enum MidiState {
    kMidiState_WaitAdc,
    kMidiState_Filter,
    kMidiState_PrepareSend,
    kMidiState_Send,
};

static enum MidiState midi_state_;

static uint8_t _MidiAdcToCc(uint16_t raw_adc) {
    return (uint8_t)((uint32_t)raw_adc * 127 / 4095);
}

static bool _MidiShouldUpdateCc(uint8_t last_cc, uint8_t next_cc, uint16_t filtered_adc) {
    if (last_cc == 0xFF) {
        return true;
    }

    if (next_cc == last_cc) {
        return false;
    }

    uint8_t delta = (next_cc > last_cc) ? (next_cc - last_cc) : (last_cc - next_cc);
    if (delta > 1) {
        return true;
    }

    uint32_t lower_edge = ((uint32_t)last_cc * 4095) / 127;
    uint32_t upper_edge = ((uint32_t)(last_cc + 1) * 4095) / 127;
    uint32_t update_up_threshold = upper_edge + MIDI_CC_HYST_ADC;
    uint32_t update_down_threshold = (lower_edge > MIDI_CC_HYST_ADC) ? (lower_edge - MIDI_CC_HYST_ADC) : 0;

    if (next_cc > last_cc) {
        return filtered_adc >= update_up_threshold;
    }

    return filtered_adc <= update_down_threshold;
}

static uint16_t _MidiFilterAdc(uint8_t idx, uint16_t raw_adc) {
    uint32_t raw_q8 = (uint32_t)raw_adc << 8;

    if (!midi_filter_ready_) {
        midi_filtered_adc_q8_[idx] = raw_q8;
        return raw_adc;
    }

    uint32_t filtered_q8 = midi_filtered_adc_q8_[idx];
    if (raw_q8 >= filtered_q8) {
        uint32_t step = (raw_q8 - filtered_q8) >> MIDI_ADC_FILTER_SHIFT;
        if (step == 0 && raw_q8 != filtered_q8) {
            step = 1;
        }
        filtered_q8 += step;
    }
    else {
        uint32_t step = (filtered_q8 - raw_q8) >> MIDI_ADC_FILTER_SHIFT;
        if (step == 0) {
            step = 1;
        }
        filtered_q8 -= step;
    }

    midi_filtered_adc_q8_[idx] = filtered_q8;
    return (uint16_t)((filtered_q8 + 128) >> 8);
}

static void _MidiBuildCcPacket(uint8_t idx, uint8_t cc_value, uint8_t packet[4]) {
    packet[0] = MIDI_CIN_CC | (MIDI_DEFAULT_CHANNEL & 0x0F);
    packet[1] = 0xB0 | (MIDI_DEFAULT_CHANNEL & 0x0F);
    packet[2] = idx + 1;
    packet[3] = cc_value;
}

static bool _MidiSelectPendingChannel(void) {
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        if (midi_pending_mask_ & (1 << i)) {
            midi_send_idx_ = i;
            return true;
        }
    }
    return false;
}

static bool _MidiRxHoldReleased(uint8_t idx, uint16_t filtered_adc) {
    uint16_t hold_adc = midi_rx_hold_adc_[idx];
    uint16_t delta = (filtered_adc > hold_adc) ? (filtered_adc - hold_adc) : (hold_adc - filtered_adc);
    return delta >= MIDI_RX_HOLD_RELEASE_ADC;
}

static bool _MidiConfirmCc(uint8_t idx, uint8_t cc_value) {
    if (midi_candidate_cc_[idx] != cc_value) {
        midi_candidate_cc_[idx] = cc_value;
        midi_candidate_count_[idx] = 1;
        return MIDI_CC_CONFIRM_COUNT <= 1;
    }

    if (midi_candidate_count_[idx] < MIDI_CC_CONFIRM_COUNT) {
        midi_candidate_count_[idx]++;
    }

    return midi_candidate_count_[idx] >= MIDI_CC_CONFIRM_COUNT;
}

static void _MidiControl(void) {
    switch (midi_state_) {
        case kMidiState_WaitAdc: {
            if (midi_raw_adc_ready_) {
                midi_state_ = kMidiState_Filter;
            }
        } break;

        case kMidiState_Filter: {
            midi_pending_mask_ = 0;
            for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
                uint16_t filtered_adc = _MidiFilterAdc(i, midi_raw_adc_[i]);
                uint8_t cc_value = _MidiAdcToCc(filtered_adc);

                if (midi_rx_hold_mask_ & (1 << i)) {
                    if (!_MidiRxHoldReleased(i, filtered_adc)) {
                        continue;
                    }
                    midi_rx_hold_mask_ &= ~(1 << i);
                }

                if (!Motor_IsMoving(i) && _MidiShouldUpdateCc(midi_last_sent_cc_[i], cc_value, filtered_adc) &&
                    _MidiConfirmCc(i, cc_value)) {
                    midi_pending_cc_[i] = cc_value;
                    midi_pending_mask_ |= (1 << i);
                }
            }
            midi_filter_ready_ = true;
            midi_state_ = kMidiState_PrepareSend;
        } break;

        case kMidiState_PrepareSend: {
            if (!_MidiSelectPendingChannel()) {
                midi_raw_adc_ready_ = false;
                midi_state_ = kMidiState_WaitAdc;
                break;
            }
            _MidiBuildCcPacket(midi_send_idx_, midi_pending_cc_[midi_send_idx_], midi_packet_);
            midi_state_ = kMidiState_Send;
        } break;

        case kMidiState_Send: {
            if (!HID_IsConnected()) {
                midi_raw_adc_ready_ = false;
                midi_state_ = kMidiState_WaitAdc;
                break;
            }

            if (Midi_Push(midi_packet_)) {
                midi_last_sent_cc_[midi_send_idx_] = midi_pending_cc_[midi_send_idx_];
                midi_pending_mask_ &= ~(1 << midi_send_idx_);
                midi_state_ = kMidiState_PrepareSend;
            }
        } break;
    }
}

// ------------------------------------------------------------
// implement
// ------------------------------------------------------------

void Motor_OnActiveChanged(uint8_t ch, bool active) {
}

void App_OnMidiCcRx(uint8_t idx, uint8_t cc_value, uint16_t target_adc) {
    if (idx >= kMotorIdx_Count) {
        return;
    }

    bool is_same_sent = (cc_value == midi_last_sent_cc_[idx]);
    bool is_same_pending = ((midi_pending_mask_ & (1 << idx)) && cc_value == midi_pending_cc_[idx]);

    midi_pending_mask_ &= ~(1 << idx);
    midi_rx_hold_adc_[idx] = target_adc;
    midi_rx_hold_mask_ |= (1 << idx);
    midi_last_sent_cc_[idx] = cc_value;
    midi_candidate_cc_[idx] = cc_value;
    midi_candidate_count_[idx] = 0;

    if (is_same_sent || is_same_pending) {
        return;
    }
    Motor_SetTarget(idx, target_adc);
}

void Motor_OnAdcReady(uint16_t raw_adc[kMotorIdx_Count]) {
    if (midi_raw_adc_ready_) {
        return;
    }

    for (int i = 0; i < kMotorIdx_Count; ++i) {
        midi_adc_sum_[i] += raw_adc[i];
    }

    midi_adc_avg_count_++;
    if (midi_adc_avg_count_ < MIDI_ADC_AVG_COUNT) {
        return;
    }

    for (int i = 0; i < kMotorIdx_Count; ++i) {
        midi_raw_adc_[i] = (uint16_t)(midi_adc_sum_[i] / MIDI_ADC_AVG_COUNT);
        midi_adc_sum_[i] = 0;
    }
    midi_adc_avg_count_ = 0;
    midi_raw_adc_ready_ = true;
}

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void App_Init(void) {
    Motor_InitControl();
    MidiCC_Init();
    midi_filter_ready_ = false;
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        midi_last_sent_cc_[i] = 0xFF;
        midi_candidate_cc_[i] = 0xFF;
        midi_candidate_count_[i] = 0;
        midi_rx_hold_adc_[i] = 0;
        midi_adc_sum_[i] = 0;
        midi_filtered_adc_q8_[i] = 0;
    }
    midi_adc_avg_count_ = 0;
    midi_rx_hold_mask_ = 0;
}

void App_Loop(void) {
    while (1) {
        _MotorControl();
        Motor_SendStatus();
        Motor_ProcessCommand();

        _MidiControl();
        MidiCC_ProcessRx();
        Midi_Poll();

        HID_Flush();
    }
}
