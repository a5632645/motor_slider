#include "app/midi_cc.h"

#include <stdbool.h>

#include "app/motor.h"
#include "config.h"
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
    kMidiState_PrepareSend,
    kMidiState_Send,
};

enum MidiRxHoldState {
    kMidiRxHold_None,
    kMidiRxHold_WaitMotorStop,
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
    bool filter_ready_;
    uint32_t filtered_adc_q8_[kMotorIdx_Count];

    bool quantized_ready_;
    uint8_t quantized_cc_[kMotorIdx_Count];
};

struct MidiTxGuard {
    uint8_t last_sent_cc_[kMotorIdx_Count];
    uint8_t candidate_cc_[kMotorIdx_Count];
    uint8_t candidate_count_[kMotorIdx_Count];

    uint8_t pending_cc_[kMotorIdx_Count];
    uint8_t pending_mask_;
};

struct MidiRxHold {
    uint8_t cc_[kMotorIdx_Count];
    enum MidiRxHoldState state_[kMotorIdx_Count];
};

struct MidiCcTx {
    enum MidiState state_;
    uint8_t send_idx_;
    uint8_t packet_[4];
};

static struct MidiAdcSampler midi_adc_;
static struct MidiQuantizer midi_quantizer_;
static struct MidiTxGuard midi_tx_guard_;
static struct MidiRxHold midi_rx_hold_;
static struct MidiCcTx midi_tx_;

// ------------------------------------------------------------
// private
// ------------------------------------------------------------

static void _MidiReceiveCc(uint8_t idx, uint8_t cc_value, uint16_t target_adc) {
    if (idx >= kMotorIdx_Count) {
        return;
    }

    bool is_same_sent = (cc_value == midi_tx_guard_.last_sent_cc_[idx]);
    bool is_same_pending = ((midi_tx_guard_.pending_mask_ & (1 << idx)) && cc_value == midi_tx_guard_.pending_cc_[idx]);

    midi_tx_guard_.pending_mask_ &= ~(1 << idx);
    midi_rx_hold_.cc_[idx] = cc_value;
    midi_rx_hold_.state_[idx] = kMidiRxHold_WaitMotorStop;
    midi_tx_guard_.last_sent_cc_[idx] = cc_value;
    midi_quantizer_.quantized_cc_[idx] = cc_value;
    midi_tx_guard_.candidate_cc_[idx] = cc_value;
    midi_tx_guard_.candidate_count_[idx] = 0;

    if (is_same_sent || is_same_pending) {
        return;
    }
    Motor_SetTarget(idx, target_adc);
}

static uint16_t _MidiCcToTargetAdc(uint8_t cc_value) {
    uint32_t target = ((uint32_t)(cc_value * 2 + 1) * 4095) / (2 * 127);
    if (target > 4095) {
        target = 4095;
    }
    return (uint16_t)target;
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

// 更新 MIDI 专用 ADC IIR 滤波器。
// 控制环使用独立的低延迟滤波，本函数只服务 MIDI CC 上报路径。
static uint16_t _MidiFilterAdc(uint8_t idx, uint16_t raw_adc) {
    uint32_t raw_q8 = (uint32_t)raw_adc << 8;

    if (!midi_quantizer_.filter_ready_) {
        midi_quantizer_.filtered_adc_q8_[idx] = raw_q8;
        return raw_adc;
    }

    uint32_t filtered_q8 = midi_quantizer_.filtered_adc_q8_[idx];
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

    midi_quantizer_.filtered_adc_q8_[idx] = filtered_q8;
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
        if (midi_tx_guard_.pending_mask_ & (1 << i)) {
            midi_tx_.send_idx_ = i;
            return true;
        }
    }
    return false;
}

// 同一个候选 CC 需要连续出现 MIDI_CC_CONFIRM_COUNT 次才允许发送。
static bool _MidiConfirmCc(uint8_t idx, uint8_t cc_value) {
    if (midi_tx_guard_.candidate_cc_[idx] != cc_value) {
        midi_tx_guard_.candidate_cc_[idx] = cc_value;
        midi_tx_guard_.candidate_count_[idx] = 1;
        return MIDI_CC_CONFIRM_COUNT <= 1;
    }

    if (midi_tx_guard_.candidate_count_[idx] < MIDI_CC_CONFIRM_COUNT) {
        midi_tx_guard_.candidate_count_[idx]++;
    }

    return midi_tx_guard_.candidate_count_[idx] >= MIDI_CC_CONFIRM_COUNT;
}

// 执行 MIDI RX 后的发送抑制状态机。
// 主机设置 CC 后，等待电机停止；停止后的第一个 CC 被捕获为静止基准。
static bool _MidiRxHoldAllowsSend(uint8_t idx, uint8_t cc_value) {
    switch (midi_rx_hold_.state_[idx]) {
        case kMidiRxHold_None:
            return true;

        case kMidiRxHold_WaitMotorStop:
            if (!Motor_IsMoving(idx)) {
                midi_rx_hold_.state_[idx] = kMidiRxHold_CaptureStopCc;
            }
            return false;

        case kMidiRxHold_CaptureStopCc:
            midi_rx_hold_.cc_[idx] = cc_value;
            midi_tx_guard_.candidate_cc_[idx] = cc_value;
            midi_tx_guard_.candidate_count_[idx] = 0;
            midi_rx_hold_.state_[idx] = kMidiRxHold_WaitUserMove;
            return false;

        case kMidiRxHold_WaitUserMove:
            if (cc_value == midi_rx_hold_.cc_[idx]) {
                midi_tx_guard_.candidate_cc_[idx] = cc_value;
                midi_tx_guard_.candidate_count_[idx] = 0;
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
    midi_quantizer_.filter_ready_ = false;
    midi_quantizer_.quantized_ready_ = false;
    midi_adc_.raw_adc_ready_ = false;
    midi_tx_guard_.pending_mask_ = 0;
    midi_tx_.send_idx_ = 0;
    midi_tx_.state_ = kMidiState_WaitAdc;
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        midi_tx_guard_.last_sent_cc_[i] = 0xFF;
        midi_quantizer_.quantized_cc_[i] = 0;
        midi_tx_guard_.candidate_cc_[i] = 0xFF;
        midi_tx_guard_.candidate_count_[i] = 0;
        midi_rx_hold_.cc_[i] = 0xFF;
        midi_rx_hold_.state_[i] = kMidiRxHold_None;
        midi_adc_.sum_[i] = 0;
        midi_adc_.min_[i] = 0;
        midi_adc_.max_[i] = 0;
        midi_quantizer_.filtered_adc_q8_[i] = 0;
        midi_tx_guard_.pending_cc_[i] = 0;
        midi_adc_.raw_adc_[i] = 0;
    }
    midi_adc_.avg_count_ = 0;
}

void MidiCC_TryTxCC(void) {
    switch (midi_tx_.state_) {
        case kMidiState_WaitAdc: {
            if (midi_adc_.raw_adc_ready_) {
                midi_tx_.state_ = kMidiState_Filter;
            }
        } break;

        case kMidiState_Filter: {
            midi_tx_guard_.pending_mask_ = 0;
            for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
                uint16_t filtered_adc = _MidiFilterAdc(i, midi_adc_.raw_adc_[i]);
                uint8_t cc_value = _MidiAdcToCc(i, filtered_adc);

                if (!_MidiRxHoldAllowsSend(i, cc_value)) {
                    continue;
                }

                if (!Motor_IsMoving(i) && _MidiShouldUpdateCc(midi_tx_guard_.last_sent_cc_[i], cc_value, filtered_adc)
                    && _MidiConfirmCc(i, cc_value)) {
                    midi_tx_guard_.pending_cc_[i] = cc_value;
                    midi_tx_guard_.pending_mask_ |= (1 << i);
                }
            }
            midi_quantizer_.filter_ready_ = true;
            midi_quantizer_.quantized_ready_ = true;
            midi_tx_.state_ = kMidiState_PrepareSend;
        } break;

        case kMidiState_PrepareSend: {
            if (!_MidiSelectPendingChannel()) {
                midi_adc_.raw_adc_ready_ = false;
                midi_tx_.state_ = kMidiState_WaitAdc;
                break;
            }
            _MidiBuildCcPacket(midi_tx_.send_idx_, midi_tx_guard_.pending_cc_[midi_tx_.send_idx_], midi_tx_.packet_);
            midi_tx_.state_ = kMidiState_Send;
        } break;

        case kMidiState_Send: {
            if (!UsbImpl_HidDebug_IsConnected()) {
                midi_adc_.raw_adc_ready_ = false;
                midi_tx_.state_ = kMidiState_WaitAdc;
                break;
            }

            if (UsbImpl_Midi_Push(midi_tx_.packet_)) {
                midi_tx_guard_.last_sent_cc_[midi_tx_.send_idx_] = midi_tx_guard_.pending_cc_[midi_tx_.send_idx_];
                midi_tx_guard_.pending_mask_ &= ~(1 << midi_tx_.send_idx_);
                midi_tx_.state_ = kMidiState_PrepareSend;
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
        if ((status & 0xF0) != 0xB0)
            continue;

        uint8_t channel = status & 0x0F;

        // 匹配映射表
        for (int j = 0; j < kMotorIdx_Count; j++) {
            if (kMidiCcMap[j].channel == channel && kMidiCcMap[j].cc_num == cc_num) {
                // CC (0-127) → ADC (0-4095), 取量化区间中心
                uint16_t target_adc = _MidiCcToTargetAdc(cc_value);
                _MidiReceiveCc(j, cc_value, target_adc);
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
