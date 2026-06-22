#include "app/midi_cc.h"

#include <stdbool.h>

#include "app/motor.h"
#include "config.h"
#include "tick.h"
#include "usb/usb_impl.h"

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

// ------------------------------------------------------------
// state
// ------------------------------------------------------------

enum MidiState {
    kMidiState_WaitAdc,
    kMidiState_Filter,
    kMidiState_Send,
};

enum MidiRxHoldState {
    kMidiRxHold_None,
    kMidiRxHold_WaitMotorStop,
    kMidiRxHold_WaitReceiveGap,
    kMidiRxHold_CaptureStopCc,
    kMidiRxHold_WaitUserMove,
};

// ------------------------------------------------------------
// context
// ------------------------------------------------------------

struct MidiAdcSampler {
    uint16_t raw_adc_[kMotorIdx_Count];
    bool raw_adc_ready_;

    uint32_t sum_[kMotorIdx_Count];
    uint16_t min_[kMotorIdx_Count];
    uint16_t max_[kMotorIdx_Count];
    uint8_t avg_count_;
};

struct MidiQuantizer {
    bool quantized_ready_;
    uint8_t quantized_cc_[kMotorIdx_Count];
};

struct MidiTxDebounce {
    uint8_t last_sent_cc_[kMotorIdx_Count];
    uint8_t candidate_cc_[kMotorIdx_Count];
    uint8_t candidate_count_[kMotorIdx_Count];
};

struct MidiRxHold {
    uint8_t cc_[kMotorIdx_Count];
    enum MidiRxHoldState state_[kMotorIdx_Count];
};

struct MidiCcTx {
    enum MidiState state_;
    uint8_t pending_cc_[kMotorIdx_Count];
    uint8_t pending_mask_;
};

static struct MidiAdcSampler midi_adc_;
static struct MidiQuantizer midi_quantizer_;
static struct MidiTxDebounce midi_tx_debounce_;
static struct MidiRxHold midi_rx_hold_;
static struct MidiCcTx midi_tx_;

static uint32_t midi_send_gap_tick_begin_[kMotorIdx_Count];
static uint32_t midi_receive_gap_tick_begin_[kMotorIdx_Count];

// ------------------------------------------------------------
// private
// ------------------------------------------------------------

static uint16_t _MidiCcToTargetAdc(uint8_t cc_value) {
    uint32_t target = ((uint32_t)(cc_value * 2 + 1) * 4095) / (2 * 127);
    if (target > 4095) {
        target = 4095;
    }
    return (uint16_t)target;
}

static void _MidiReceiveCc(uint8_t idx, uint8_t cc_value) {
    if (Tick_GetMs() - midi_send_gap_tick_begin_[idx] <= CONFIG_MIDI_SEND_GAP_TIME) {
        return;
    }

    bool is_same_sent = (cc_value == midi_tx_debounce_.last_sent_cc_[idx]);
    bool is_same_pending = ((midi_tx_.pending_mask_ & (1 << idx)) && cc_value == midi_tx_.pending_cc_[idx]);

    midi_tx_.pending_mask_ &= ~(1 << idx);
    midi_rx_hold_.cc_[idx] = cc_value;
    midi_rx_hold_.state_[idx] = kMidiRxHold_WaitMotorStop;
    midi_tx_debounce_.last_sent_cc_[idx] = cc_value;
    midi_quantizer_.quantized_cc_[idx] = cc_value;
    midi_tx_debounce_.candidate_cc_[idx] = cc_value;
    midi_tx_debounce_.candidate_count_[idx] = 0;
    midi_receive_gap_tick_begin_[idx] = Tick_GetMs();

    if (is_same_sent || is_same_pending) {
        return;
    }

    // CC (0-127) → ADC (0-4095), 取量化区间中心
    uint16_t target_adc = _MidiCcToTargetAdc(cc_value);
    Motor_SetTarget(idx, target_adc);
}

// 将 ADC 值量化为 MIDI CC，并在 CC 边界保留死区。
// 每个通道保存上一次量化出的 CC，避免边界抖动。
static uint8_t _MidiAdcToCc(uint8_t idx, uint16_t adc) {
    uint8_t cc = midi_quantizer_.quantized_cc_[idx];

    if (!midi_quantizer_.quantized_ready_) {
        cc = (uint8_t)((uint32_t)adc * 127 / 4095);
        midi_quantizer_.quantized_cc_[idx] = cc;
        return cc;
    }

    while (cc < 127) {
        uint32_t up_boundary = ((uint32_t)(cc + 1) * 4095) / 127;
        if (adc < up_boundary + MIDI_CC_BOUNDARY_GAP_ADC) {
            break;
        }
        cc++;
    }

    while (cc > 0) {
        uint32_t down_boundary = ((uint32_t)cc * 4095) / 127;
        uint32_t down_threshold =
            (down_boundary > MIDI_CC_BOUNDARY_GAP_ADC) ? (down_boundary - MIDI_CC_BOUNDARY_GAP_ADC) : 0;
        if (adc > down_threshold) {
            break;
        }
        cc--;
    }

    midi_quantizer_.quantized_cc_[idx] = cc;
    return cc;
}

static bool _MidiShouldUpdateCc(uint8_t last_cc, uint8_t next_cc, uint16_t filtered_adc) {
    (void)filtered_adc;

    if (last_cc == 0xFF) {
        return true;
    }

    return next_cc != last_cc;
}

static void _MidiBuildCcPacket(uint8_t idx, uint8_t cc_value, uint8_t packet[4]) {
    packet[0] = MIDI_CIN_CC | (MIDI_DEFAULT_CHANNEL & 0x0F);
    packet[1] = 0xB0 | (MIDI_DEFAULT_CHANNEL & 0x0F);
    packet[2] = idx + 1;
    packet[3] = cc_value;
}

// 同一个候选 CC 需要连续出现 MIDI_CC_CONFIRM_COUNT 次才允许发送。
static bool _MidiConfirmCc(uint8_t idx, uint8_t cc_value) {
    if (midi_tx_debounce_.candidate_cc_[idx] != cc_value) {
        midi_tx_debounce_.candidate_cc_[idx] = cc_value;
        midi_tx_debounce_.candidate_count_[idx] = 1;
        return MIDI_CC_CONFIRM_COUNT <= 1;
    }

    if (midi_tx_debounce_.candidate_count_[idx] < MIDI_CC_CONFIRM_COUNT) {
        midi_tx_debounce_.candidate_count_[idx]++;
    }

    return midi_tx_debounce_.candidate_count_[idx] >= MIDI_CC_CONFIRM_COUNT;
}

// 执行 MIDI RX 后的发送抑制状态机。
// 主机设置 CC 后，等待电机停止；停止后的第一个 CC 被捕获为静止基准。
static bool _MidiRxHoldAllowsSend(uint8_t idx, uint8_t cc_value) {
    switch (midi_rx_hold_.state_[idx]) {
        case kMidiRxHold_None:
            return true;

        case kMidiRxHold_WaitMotorStop:
            if (!Motor_IsMoving(idx)) {
                midi_rx_hold_.state_[idx] = kMidiRxHold_WaitReceiveGap;
            }
            return false;

        case kMidiRxHold_WaitReceiveGap:
            if (Tick_GetMs() - midi_receive_gap_tick_begin_[idx] > CONFIG_MIDI_RECEIVE_GAP_TIME) {
                midi_rx_hold_.state_[idx] = kMidiRxHold_CaptureStopCc;
            }
            return false;

        case kMidiRxHold_CaptureStopCc:
            midi_rx_hold_.cc_[idx] = cc_value;
            midi_tx_debounce_.candidate_cc_[idx] = cc_value;
            midi_tx_debounce_.candidate_count_[idx] = 0;
            midi_rx_hold_.state_[idx] = kMidiRxHold_WaitUserMove;
            return false;

        case kMidiRxHold_WaitUserMove:
            if (cc_value == midi_rx_hold_.cc_[idx]) {
                midi_tx_debounce_.candidate_cc_[idx] = cc_value;
                midi_tx_debounce_.candidate_count_[idx] = 0;
                return false;
            }

            if (!_MidiConfirmCc(idx, cc_value)) {
                return false;
            }

            midi_rx_hold_.state_[idx] = kMidiRxHold_None;
            return true;
    }

    return false;
}

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void MidiCC_Init(void) {
    // ---- TX 状态机 ----
    midi_tx_.state_ = kMidiState_WaitAdc;
    midi_tx_.pending_mask_ = 0;

    // ---- MidiTxDebounce ----
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        midi_tx_debounce_.last_sent_cc_[i] = 0xFF;
        midi_tx_debounce_.candidate_cc_[i] = 0xFF;
        midi_tx_debounce_.candidate_count_[i] = 0;
    }

    // ---- MidiQuantizer ----
    midi_quantizer_.quantized_ready_ = false;
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        midi_quantizer_.quantized_cc_[i] = 0;
    }

    // ---- MidiRxHold ----
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        midi_rx_hold_.cc_[i] = 0xFF;
        midi_rx_hold_.state_[i] = kMidiRxHold_None;
    }

    // ---- MidiAdcSampler ----
    midi_adc_.raw_adc_ready_ = false;
    midi_adc_.avg_count_ = 0;
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        midi_adc_.sum_[i] = 0;
        midi_adc_.min_[i] = 0;
        midi_adc_.max_[i] = 0;
        midi_adc_.raw_adc_[i] = 0;
    }

    // ---- MidiCcTx ----
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        midi_tx_.pending_cc_[i] = 0;
        midi_send_gap_tick_begin_[i] = 0;
        midi_receive_gap_tick_begin_[i] = 0;
    }
}

void MidiCC_TryTxCC(void) {
    switch (midi_tx_.state_) {
        case kMidiState_WaitAdc: {
            if (midi_adc_.raw_adc_ready_) {
                midi_tx_.state_ = kMidiState_Filter;
            }
        } break;

        case kMidiState_Filter: {
            midi_tx_.pending_mask_ = 0;
            for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
                uint16_t adc_val = midi_adc_.raw_adc_[i];
                uint8_t cc_value = _MidiAdcToCc(i, adc_val);

                if (!_MidiRxHoldAllowsSend(i, cc_value)) {
                    continue;
                }

                if (!Motor_IsMoving(i) && _MidiShouldUpdateCc(midi_tx_debounce_.last_sent_cc_[i], cc_value, adc_val)
                    && _MidiConfirmCc(i, cc_value)) {
                    midi_tx_.pending_cc_[i] = cc_value;
                    midi_tx_.pending_mask_ |= (1 << i);
                }
            }
            midi_quantizer_.quantized_ready_ = true;
            midi_tx_.state_ = kMidiState_Send;
        } break;

        case kMidiState_Send: {
            if (midi_tx_.pending_mask_ == 0) {
                midi_adc_.raw_adc_ready_ = false;
                midi_tx_.state_ = kMidiState_WaitAdc;
                break;
            }

            if (!UsbImpl_HidDebug_IsConnected()) {
                midi_adc_.raw_adc_ready_ = false;
                midi_tx_.state_ = kMidiState_WaitAdc;
                break;
            }

            uint32_t pending_idx = __builtin_ctz(midi_tx_.pending_mask_);
            if (Tick_GetMs() - midi_receive_gap_tick_begin_[pending_idx] <= CONFIG_MIDI_RECEIVE_GAP_TIME) {
                midi_tx_.pending_mask_ &= ~(1 << pending_idx);
                break;
            }

            uint8_t packet_[4];
            _MidiBuildCcPacket(pending_idx, midi_tx_.pending_cc_[pending_idx], packet_);
            if (UsbImpl_Midi_Push(packet_)) {
                midi_tx_debounce_.last_sent_cc_[pending_idx] = midi_tx_.pending_cc_[pending_idx];
                midi_tx_.pending_mask_ &= ~(1 << pending_idx);
                midi_send_gap_tick_begin_[pending_idx] = Tick_GetMs();
            }
        } break;
    }
}

void MidiCC_ProcessRx(void) {
    uint32_t len;
    uint8_t const* buf = UsbImpl_Midi_GetRxBuffer(&len);
    if (len == 0) {
        return;
    }

    // 解析 4 字节 MIDI Event Packet
    for (uint32_t i = 0; i + 4 <= len; i += 4) {
        uint8_t status = buf[i + 1];
        uint8_t cc_num = buf[i + 2];
        uint8_t cc_value = buf[i + 3];

        // 仅处理 CC 消息 (0xB0)
        if ((status & 0xF0) != 0xB0) {
            continue;
        }

        uint8_t channel = status & 0x0F;

        // 匹配映射表
        for (uint8_t j = 0; j < kMotorIdx_Count; j++) {
            if (kMidiCcMap[j].channel == channel && kMidiCcMap[j].cc_num == cc_num) {
                _MidiReceiveCc(j, cc_value);
                break;
            }
        }
    }

    UsbImpl_Midi_SetRxReady();
}

void MidiCC_UpdateAdc(uint16_t raw_adc[kMotorIdx_Count]) {
    if (midi_adc_.raw_adc_ready_) {
        return;
    }

    for (int i = 0; i < kMotorIdx_Count; ++i) {
        if (midi_adc_.avg_count_ == 0) {
            midi_adc_.min_[i] = raw_adc[i];
            midi_adc_.max_[i] = raw_adc[i];
        }
        else {
            if (raw_adc[i] < midi_adc_.min_[i]) {
                midi_adc_.min_[i] = raw_adc[i];
            }
            if (raw_adc[i] > midi_adc_.max_[i]) {
                midi_adc_.max_[i] = raw_adc[i];
            }
        }
        midi_adc_.sum_[i] += raw_adc[i];
    }

    midi_adc_.avg_count_++;
    if (midi_adc_.avg_count_ < MIDI_ADC_AVG_COUNT) {
        return;
    }

    for (int i = 0; i < kMotorIdx_Count; ++i) {
        uint32_t sum = midi_adc_.sum_[i];
        uint32_t count = MIDI_ADC_AVG_COUNT;
        if (MIDI_ADC_AVG_COUNT > 2) {
            sum -= midi_adc_.min_[i];
            sum -= midi_adc_.max_[i];
            count -= 2;
        }
        midi_adc_.raw_adc_[i] = (uint16_t)(sum / count);
        midi_adc_.sum_[i] = 0;
    }
    midi_adc_.avg_count_ = 0;
    midi_adc_.raw_adc_ready_ = true;
}
