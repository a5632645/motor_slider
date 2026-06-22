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

// 推进电机控制状态机。
// 按 CTRL_LOOP_US 周期启动一次 ADC scan，等待 DMA 数据就绪后运行闭环控制。
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
// implement
// ------------------------------------------------------------

void Motor_OnActiveChanged(uint8_t ch, bool active) {}

void Motor_OnRawAdcReady(uint16_t raw_adc[kMotorIdx_Count]) {}

void Motor_OnFilterAdcReady(uint16_t raw_adc[kMotorIdx_Count]) {
    MidiCC_UpdateAdc(raw_adc);
}

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void App_Init(void) {
    Motor_InitControl();
    MidiCC_Init();
}

void App_Loop(void) {
    while (1) {
        _MotorControl();
        Motor_SendStatus();
        Motor_ProcessCommand();

        MidiCC_ProcessRx();
        MidiCC_TryTxCC();

        UsbImpl_Midi_Poll();
        UsbImpl_HidDebug_Flush();
    }
}
