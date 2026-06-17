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
static uint32_t last_ctrl_us_ = 0;

/**
 * @brief 推进电机控制状态机
 *
 * 按 CTRL_LOOP_US 周期启动一次 ADC scan，等待 DMA 数据就绪后运行闭环控制。
 */
static void _MotorControl(void) {
    switch (ctrl_state_) {
        case kCtrlState_Idle: {
            uint32_t now_us = Tick_GetUs();
            if ((uint32_t)(now_us - last_ctrl_us_) >= CTRL_LOOP_US) {
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

static uint16_t midi_raw_adc_[kMotorIdx_Count];
static bool midi_raw_adc_ready_;
static uint32_t midi_adc_sum_[kMotorIdx_Count];
static uint16_t midi_adc_min_[kMotorIdx_Count];
static uint16_t midi_adc_max_[kMotorIdx_Count];
static uint8_t midi_adc_avg_count_;
static bool midi_filter_ready_;
static uint32_t midi_filtered_adc_q8_[kMotorIdx_Count];
static uint8_t midi_quantized_cc_[kMotorIdx_Count];
static bool midi_quantized_ready_;
static uint8_t midi_last_sent_cc_[kMotorIdx_Count];
static uint8_t midi_candidate_cc_[kMotorIdx_Count];
static uint8_t midi_candidate_count_[kMotorIdx_Count];
static uint8_t midi_pending_cc_[kMotorIdx_Count];
static uint8_t midi_pending_mask_;
static uint8_t midi_rx_hold_cc_[kMotorIdx_Count];
static uint8_t midi_send_idx_;
static uint8_t midi_packet_[4];

enum MidiState {
    kMidiState_WaitAdc,
    kMidiState_Filter,
    kMidiState_PrepareSend,
    kMidiState_Send,
};

static enum MidiState midi_state_;

enum MidiRxHoldState {
    kMidiRxHold_None,
    kMidiRxHold_WaitMotorStop,
    kMidiRxHold_CaptureStopCc,
    kMidiRxHold_WaitUserMove,
};

static enum MidiRxHoldState midi_rx_hold_state_[kMotorIdx_Count];

/**
 * @brief 将 ADC 值量化为 MIDI CC，并在 CC 边界保留死区
 *
 * 每个通道保存上一次量化出的 CC。ADC 必须越过边界加/减
 * MIDI_CC_BOUNDARY_GAP_ADC 后才切换到相邻 CC，避免边界抖动。
 *
 * @param idx 电机/推子通道索引
 * @param adc 已滤波的 ADC 值
 * @return 量化后的 MIDI CC 值，范围 0~127
 */
static uint8_t _MidiAdcToCc(uint8_t idx, uint16_t adc) {
    uint8_t cc = midi_quantized_cc_[idx];

    if (!midi_quantized_ready_) {
        cc = (uint8_t)((uint32_t)adc * 127 / 4095);
        midi_quantized_cc_[idx] = cc;
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
        uint32_t down_threshold = (down_boundary > MIDI_CC_BOUNDARY_GAP_ADC)
                                      ? (down_boundary - MIDI_CC_BOUNDARY_GAP_ADC)
                                      : 0;
        if (adc > down_threshold) {
            break;
        }
        cc--;
    }

    midi_quantized_cc_[idx] = cc;
    return cc;
}

/**
 * @brief 判断新的 CC 是否需要上报
 * @param last_cc 上一次已发送或被主机接管的 CC，0xFF 表示未初始化
 * @param next_cc 当前量化得到的 CC
 * @param filtered_adc 当前滤波 ADC，保留参数用于兼容调用路径
 * @return 需要上报返回 true，否则返回 false
 */
static bool _MidiShouldUpdateCc(uint8_t last_cc, uint8_t next_cc, uint16_t filtered_adc) {
    (void)filtered_adc;

    if (last_cc == 0xFF) {
        return true;
    }

    return next_cc != last_cc;
}

/**
 * @brief 更新 MIDI 专用 ADC IIR 滤波器
 *
 * 控制环使用独立的低延迟滤波，本函数只服务 MIDI CC 上报路径。
 *
 * @param idx 电机/推子通道索引
 * @param raw_adc 去极值平均后的 ADC 值
 * @return 滤波后的 ADC 值
 */
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

/**
 * @brief 构造 USB-MIDI CC 事件包
 * @param idx 电机/推子通道索引
 * @param cc_value MIDI CC 值
 * @param packet 输出的 4 字节 USB-MIDI Event Packet
 */
static void _MidiBuildCcPacket(uint8_t idx, uint8_t cc_value, uint8_t packet[4]) {
    packet[0] = MIDI_CIN_CC | (MIDI_DEFAULT_CHANNEL & 0x0F);
    packet[1] = 0xB0 | (MIDI_DEFAULT_CHANNEL & 0x0F);
    packet[2] = idx + 1;
    packet[3] = cc_value;
}

/**
 * @brief 从待发送位图中选择一个通道
 * @return 找到待发送通道返回 true，否则返回 false
 */
static bool _MidiSelectPendingChannel(void) {
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        if (midi_pending_mask_ & (1 << i)) {
            midi_send_idx_ = i;
            return true;
        }
    }
    return false;
}

/**
 * @brief 确认 CC 候选值是否连续稳定
 *
 * 同一个候选 CC 需要连续出现 MIDI_CC_CONFIRM_COUNT 次才允许发送。
 *
 * @param idx 电机/推子通道索引
 * @param cc_value 当前候选 CC
 * @return 候选值已确认返回 true，否则返回 false
 */
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

/**
 * @brief 执行 MIDI RX 后的发送抑制状态机
 *
 * 主机设置 CC 后，等待电机停止；停止后的第一个 CC 被捕获为静止基准，
 * 后续只有出现不同且已确认的 CC，才认为用户重新移动推子并允许发送。
 *
 * @param idx 电机/推子通道索引
 * @param cc_value 当前量化得到的 CC
 * @return 当前 CC 允许进入发送流程返回 true，否则返回 false
 */
static bool _MidiRxHoldAllowsSend(uint8_t idx, uint8_t cc_value) {
    switch (midi_rx_hold_state_[idx]) {
        case kMidiRxHold_None:
            return true;

        case kMidiRxHold_WaitMotorStop:
            if (!Motor_IsMoving(idx)) {
                midi_rx_hold_state_[idx] = kMidiRxHold_CaptureStopCc;
            }
            return false;

        case kMidiRxHold_CaptureStopCc:
            midi_rx_hold_cc_[idx] = cc_value;
            midi_candidate_cc_[idx] = cc_value;
            midi_candidate_count_[idx] = 0;
            midi_rx_hold_state_[idx] = kMidiRxHold_WaitUserMove;
            return false;

        case kMidiRxHold_WaitUserMove:
            if (cc_value == midi_rx_hold_cc_[idx]) {
                midi_candidate_cc_[idx] = cc_value;
                midi_candidate_count_[idx] = 0;
                return false;
            }

            if (!_MidiConfirmCc(idx, cc_value)) {
                return false;
            }

            midi_rx_hold_state_[idx] = kMidiRxHold_None;
            return true;
    }

    return false;
}

/**
 * @brief 推进 MIDI CC 发送状态机
 *
 * 消费平均后的 ADC 数据，经过 IIR、边界 GAP 量化、RX hold 和连续确认后，
 * 将待发送 CC 组装为 USB-MIDI 事件包并推入 MIDI TX FIFO。
 */
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
                uint8_t cc_value = _MidiAdcToCc(i, filtered_adc);

                if (!_MidiRxHoldAllowsSend(i, cc_value)) {
                    continue;
                }

                if (!Motor_IsMoving(i) && _MidiShouldUpdateCc(midi_last_sent_cc_[i], cc_value, filtered_adc) &&
                    _MidiConfirmCc(i, cc_value)) {
                    midi_pending_cc_[i] = cc_value;
                    midi_pending_mask_ |= (1 << i);
                }
            }
            midi_filter_ready_ = true;
            midi_quantized_ready_ = true;
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

/**
 * @brief 处理主机发来的 MIDI CC
 *
 * 进入 RX hold，清除该通道待发送 CC，并把量化器状态同步到主机值。
 * 如果该 CC 与已知主机值或待发送值相同，则只更新 hold 状态，不重复设目标。
 *
 * @param idx 电机/推子通道索引
 * @param cc_value 主机发送的 CC 值
 * @param target_adc CC 映射到的目标 ADC
 */
void App_OnMidiCcRx(uint8_t idx, uint8_t cc_value, uint16_t target_adc) {
    if (idx >= kMotorIdx_Count) {
        return;
    }

    bool is_same_sent = (cc_value == midi_last_sent_cc_[idx]);
    bool is_same_pending = ((midi_pending_mask_ & (1 << idx)) && cc_value == midi_pending_cc_[idx]);

    midi_pending_mask_ &= ~(1 << idx);
    midi_rx_hold_cc_[idx] = cc_value;
    midi_rx_hold_state_[idx] = kMidiRxHold_WaitMotorStop;
    midi_last_sent_cc_[idx] = cc_value;
    midi_quantized_cc_[idx] = cc_value;
    midi_candidate_cc_[idx] = cc_value;
    midi_candidate_count_[idx] = 0;

    if (is_same_sent || is_same_pending) {
        return;
    }
    Motor_SetTarget(idx, target_adc);
}

/**
 * @brief 接收一组控制环 ADC 数据并更新 MIDI 平均窗口
 *
 * MIDI 路径先累积 MIDI_ADC_AVG_COUNT 组 ADC，并在计算平均值时去掉最大值和最小值。
 * 如果上一组 MIDI ADC 尚未被状态机消费，则暂停累积，避免覆盖未处理数据。
 *
 * @param raw_adc 控制环刚读取到的原始 ADC 数组
 */
void Motor_OnAdcReady(uint16_t raw_adc[kMotorIdx_Count]) {
    if (midi_raw_adc_ready_) {
        return;
    }

    for (int i = 0; i < kMotorIdx_Count; ++i) {
        if (midi_adc_avg_count_ == 0) {
            midi_adc_min_[i] = raw_adc[i];
            midi_adc_max_[i] = raw_adc[i];
        }
        else {
            if (raw_adc[i] < midi_adc_min_[i]) {
                midi_adc_min_[i] = raw_adc[i];
            }
            if (raw_adc[i] > midi_adc_max_[i]) {
                midi_adc_max_[i] = raw_adc[i];
            }
        }
        midi_adc_sum_[i] += raw_adc[i];
    }

    midi_adc_avg_count_++;
    if (midi_adc_avg_count_ < MIDI_ADC_AVG_COUNT) {
        return;
    }

    for (int i = 0; i < kMotorIdx_Count; ++i) {
        uint32_t sum = midi_adc_sum_[i];
        uint32_t count = MIDI_ADC_AVG_COUNT;
        if (MIDI_ADC_AVG_COUNT > 2) {
            sum -= midi_adc_min_[i];
            sum -= midi_adc_max_[i];
            count -= 2;
        }
        midi_raw_adc_[i] = (uint16_t)(sum / count);
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
    midi_quantized_ready_ = false;
    for (uint8_t i = 0; i < kMotorIdx_Count; ++i) {
        midi_last_sent_cc_[i] = 0xFF;
        midi_quantized_cc_[i] = 0;
        midi_candidate_cc_[i] = 0xFF;
        midi_candidate_count_[i] = 0;
        midi_rx_hold_cc_[i] = 0xFF;
        midi_rx_hold_state_[i] = kMidiRxHold_None;
        midi_adc_sum_[i] = 0;
        midi_adc_min_[i] = 0;
        midi_adc_max_[i] = 0;
        midi_filtered_adc_q8_[i] = 0;
    }
    midi_adc_avg_count_ = 0;
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
