#include "app.h"

#include "bsp/tick.h"
#include "motor.h"
#include "usb/usb_impl.h"

#include <stdint.h>

// ------------------------------------------------------------
// private
// ------------------------------------------------------------

/* 控制状态机 */
enum CtrlState {
    kCtrlState_Idle,
    kCtrlState_AdcStart,
    kCtrlState_AdcWait,
    kCtrlState_Control,
};

static enum CtrlState ctrl_state_ = kCtrlState_Idle;
static uint32_t last_ctrl_tick_ = 0;

// ------------------------------------------------------------
// public
// ------------------------------------------------------------

void App_Init(void) {
    Motor_InitControl();
}

void App_Loop(void) {
    while (1) {
        /* ── 1kHz 控制状态机 ── */
        switch (ctrl_state_) {
            case kCtrlState_Idle:
                if (Tick_Get() != last_ctrl_tick_) {
                    last_ctrl_tick_ = Tick_Get();
                    ctrl_state_ = kCtrlState_AdcStart;
                }
                break;

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

        Motor_SendStatus();
        Motor_ProcessCommand();
    }
}
