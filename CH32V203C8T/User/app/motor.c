#include "motor.h"

#include "bsp/tick.h"
#include "config.h"
#include "pid.h"
#include "usb/usb_impl.h"

#include <stdio.h>

// ------------------------------------------------------------
// variable
// ------------------------------------------------------------

/* 电机状态数组 */
static struct MotorState motor_states_[kMotorIdx_Count];
static uint16_t pwm_bias_ = PWM_BIAS;
static uint16_t pwm_max_ = PWM_MAX;

static void _Motor_ReinitPidLimits(void) {
    for (uint8_t i = 0; i < kMotorIdx_Count; i++) {
        Pid_Init(&motor_states_[i].pid_, motor_states_[i].pid_.kp, motor_states_[i].pid_.ki, pwm_bias_, pwm_max_);
    }
}

static uint16_t _Motor_FilterAdc(struct MotorState* state, uint16_t raw_adc) {
    uint32_t raw_q8 = (uint32_t)raw_adc << 8;
    uint32_t filtered_q8 = state->filtered_adc_q8_;

    if (raw_q8 >= filtered_q8) {
        uint32_t step = (raw_q8 - filtered_q8) >> CTRL_ADC_FILTER_SHIFT;
        if (step == 0 && raw_q8 != filtered_q8) {
            step = 1;
        }
        filtered_q8 += step;
    }
    else {
        uint32_t step = (filtered_q8 - raw_q8) >> CTRL_ADC_FILTER_SHIFT;
        if (step == 0) {
            step = 1;
        }
        filtered_q8 -= step;
    }

    state->filtered_adc_q8_ = filtered_q8;
    return (uint16_t)((filtered_q8 + 128) >> 8);
}

static int16_t _Motor_AbsI16(int16_t value) {
    return (value < 0) ? -value : value;
}

// ------------------------------------------------------------
// publci
// ------------------------------------------------------------

void Motor_InitControl(void) {
    for (int i = 0; i < kMotorIdx_Count; i++) {
        motor_states_[i].target_adc_ = 2048;
        motor_states_[i].current_adc_ = 2048;
        motor_states_[i].filtered_adc_q8_ = (uint32_t)2048 << 8;
        motor_states_[i].dir_ = kMotorDir_Stop;
        motor_states_[i].duty_ = 0;
        motor_states_[i].active_ = false;
        motor_states_[i].start_tick_ = 0;
        motor_states_[i].last_adc_ = 2048;
        motor_states_[i].settle_tick_ = 0;
        motor_states_[i].settling_ = false;
        Pid_Init(&motor_states_[i].pid_, PID_DEFAULT_KP, PID_DEFAULT_KI, pwm_bias_, pwm_max_);
    }
}

void Motor_StartAdcConversion(void) {
    MotorHw_StartAdcConversion();
}

bool Motor_IsAdcReady(void) {
    return MotorHw_IsAdcReady();
}

void Motor_RunControlLoop(void) {
    uint16_t raw_adc[kMotorIdx_Count];
    uint32_t now_tick = Tick_Get();

    MotorHw_GetAdcValue(raw_adc);
    Motor_OnRawAdcReady(raw_adc);
    {
        uint16_t filter_adc[kMotorIdx_Count];
        for (int i = 0; i < kMotorIdx_Count; ++i) {
            motor_states_[i].current_adc_ = _Motor_FilterAdc(&motor_states_[i], raw_adc[i]);
            filter_adc[i] = motor_states_[i].current_adc_;
        }
        Motor_OnFilterAdcReady(filter_adc);
    }

    for (int i = 0; i < kMotorIdx_Count; i++) {
        // 非活跃电机：跳过，PWM 保持 0
        if (!motor_states_[i].active_)
            continue;

        // 判断是否到达目标
        int16_t diff = (int16_t)(motor_states_[i].target_adc_ - motor_states_[i].current_adc_);
        int16_t abs_diff = _Motor_AbsI16(diff);
        int16_t adc_delta = (int16_t)(motor_states_[i].current_adc_ - motor_states_[i].last_adc_);
        int16_t abs_adc_delta = _Motor_AbsI16(adc_delta);

        if (abs_diff <= CTRL_ERROR_THRESHOLD && abs_adc_delta <= CTRL_STILL_THRESHOLD) {
            if (!motor_states_[i].settling_) {
                motor_states_[i].settle_tick_ = now_tick;
                motor_states_[i].settling_ = true;
            }
        }
        else {
            motor_states_[i].settling_ = false;
        }
        motor_states_[i].last_adc_ = motor_states_[i].current_adc_;

        if (motor_states_[i].settling_ && (uint32_t)(now_tick - motor_states_[i].settle_tick_) >= CTRL_SETTLE_MS) {
            motor_states_[i].active_ = false;
            Motor_OnActiveChanged(i, false);
            motor_states_[i].duty_ = 0;
            MotorHw_SetPwm(i, kMotorDir_Stop, 0);
            continue;
        }

        // 超时判断
        if ((uint32_t)(now_tick - motor_states_[i].start_tick_) >= CTRL_TIMEOUT_MS) {
            motor_states_[i].active_ = false;
            Motor_OnActiveChanged(i, false);
            motor_states_[i].duty_ = 0;
            MotorHw_SetPwm(i, kMotorDir_Stop, 0);
            continue;
        }

        float output = Pid_Update(&motor_states_[i].pid_, (float)motor_states_[i].target_adc_,
                                  (float)motor_states_[i].current_adc_);

        enum MotorDir next_dir = kMotorDir_Stop;
        if (output >= CTRL_OUTPUT_DEADBAND) {
            next_dir = kMotorDir_Forward;
        }
        else if (output <= -CTRL_OUTPUT_DEADBAND) {
            next_dir = kMotorDir_Reverse;
        }

        if (next_dir != kMotorDir_Stop && motor_states_[i].dir_ != kMotorDir_Stop && next_dir != motor_states_[i].dir_
            && abs_diff <= CTRL_REVERSE_THRESHOLD) {
            next_dir = kMotorDir_Stop;
            Pid_Reset(&motor_states_[i].pid_);
        }

        if (next_dir == kMotorDir_Forward) {
            motor_states_[i].dir_ = kMotorDir_Forward;
            motor_states_[i].duty_ = (uint16_t)output + pwm_bias_;
        }
        else if (next_dir == kMotorDir_Reverse) {
            motor_states_[i].dir_ = kMotorDir_Reverse;
            motor_states_[i].duty_ = (uint16_t)(-output) + pwm_bias_;
        }
        else {
            motor_states_[i].dir_ = kMotorDir_Stop;
            motor_states_[i].duty_ = 0;
        }

        MotorHw_SetPwm(i, motor_states_[i].dir_, motor_states_[i].duty_);
    }
}

void Motor_SetTarget(uint8_t ch, uint16_t target_adc) {
    int16_t diff = (int16_t)(target_adc - motor_states_[ch].current_adc_);

    motor_states_[ch].target_adc_ = target_adc;
    if (_Motor_AbsI16(diff) <= CTRL_TARGET_SNAP_THRESHOLD) {
        motor_states_[ch].active_ = false;
        motor_states_[ch].dir_ = kMotorDir_Stop;
        motor_states_[ch].duty_ = 0;
        motor_states_[ch].settling_ = false;
        Pid_Reset(&motor_states_[ch].pid_);
        MotorHw_SetPwm(ch, kMotorDir_Stop, 0);
        Motor_OnActiveChanged(ch, false);
        return;
    }

    motor_states_[ch].active_ = true;
    motor_states_[ch].start_tick_ = Tick_Get();
    motor_states_[ch].last_adc_ = motor_states_[ch].current_adc_;
    motor_states_[ch].settle_tick_ = 0;
    motor_states_[ch].settling_ = false;
    Pid_Reset(&motor_states_[ch].pid_);
    Motor_OnActiveChanged(ch, true);
}

void Motor_StopAll(void) {
    for (int i = 0; i < kMotorIdx_Count; i++) {
        motor_states_[i].active_ = false;
        motor_states_[i].start_tick_ = 0;
        motor_states_[i].settle_tick_ = 0;
        motor_states_[i].settling_ = false;
        MotorHw_SetPwm(i, kMotorDir_Stop, 0);
        Motor_OnActiveChanged(i, false);
    }
}

void Motor_SetPid(uint8_t ch, float kp, float ki, float kd) {
    if (ch == 0xFF) {
        for (uint8_t i = 0; i < kMotorIdx_Count; i++) {
            Pid_Init(&motor_states_[i].pid_, kp, ki, pwm_bias_, pwm_max_);
        }
    }
    else if (ch < kMotorIdx_Count) {
        Pid_Init(&motor_states_[ch].pid_, kp, ki, pwm_bias_, pwm_max_);
    }
}

void Motor_SetPwmBias(uint16_t pwm_bias) {
    if (pwm_bias > pwm_max_) {
        pwm_bias = pwm_max_;
    }

    pwm_bias_ = pwm_bias;
    _Motor_ReinitPidLimits();
}

void Motor_SetPwmMax(uint16_t pwm_max) {
    if (pwm_max > PWM_MAX) {
        pwm_max = PWM_MAX;
    }

    pwm_max_ = pwm_max;
    if (pwm_bias_ > pwm_max_) {
        pwm_bias_ = pwm_max_;
    }
    _Motor_ReinitPidLimits();
}

void Motor_SendStatus(void) {
    if (!UsbImpl_HidMotor_IsTxReady())
        return;

    uint8_t flags = 0;
    uint16_t adc[kMotorIdx_Count];
    uint16_t target[kMotorIdx_Count];
    uint16_t duty[kMotorIdx_Count];
    for (int i = 0; i < kMotorIdx_Count; i++) {
        adc[i] = motor_states_[i].current_adc_;
        target[i] = motor_states_[i].target_adc_;
        duty[i] = motor_states_[i].duty_;
        if (motor_states_[i].active_)
            flags |= (uint8_t)(1u << i);
    }

    uint8_t buf[kHidReportSize];

    buf[HID1_STATUS_FLAGS] = 0x01; /* running */
    buf[HID1_ACTIVE_FLAGS] = flags;

    for (int i = 0; i < kMotorIdx_Count; i++) {
        buf[HID1_ADC(i)] = (uint8_t)(adc[i] & 0xFF);
        buf[HID1_ADC(i) + 1] = (uint8_t)((adc[i] >> 8) & 0xFF);
        buf[HID1_TARGET(i)] = (uint8_t)(target[i] & 0xFF);
        buf[HID1_TARGET(i) + 1] = (uint8_t)((target[i] >> 8) & 0xFF);
        buf[HID1_DUTY(i)] = (uint8_t)(duty[i] & 0xFF);
        buf[HID1_DUTY(i) + 1] = (uint8_t)((duty[i] >> 8) & 0xFF);
    }

    UsbImpl_HidMotor_Write(buf);
}

void Motor_ProcessCommand(void) {
    uint8_t buf[kHidReportSize];
    if (!UsbImpl_HidMotor_Read(buf))
        return;

    switch (buf[0]) {
        case 0x01: {
            uint8_t count = buf[1];
            if (count > kMotorIdx_Count)
                count = kMotorIdx_Count;

            if (count > 0) {
                printf("[HID1] 设置目标:");
                for (uint8_t j = 0; j < count; j++) {
                    uint8_t idx = buf[2 + j * 3];
                    uint16_t pos = (uint16_t)buf[3 + j * 3] | (uint16_t)(buf[4 + j * 3] << 8);
                    if (idx < kMotorIdx_Count) {
                        Motor_SetTarget(idx, pos);
                        printf(" CH%d=%d", idx + 1, pos);
                    }
                }
                printf("\r\n");
            }
        } break;

        case 0x03:
            Motor_StopAll();
            printf("[HID1] 停止所有电机\r\n");
            break;

        // 设置 PID 参数
        case 0x04: {
            uint8_t ch = buf[1];
            float kp = (float)((uint16_t)buf[2] | (uint16_t)(buf[3] << 8)) / 1000.0f;
            float ki = (float)((uint16_t)buf[4] | (uint16_t)(buf[5] << 8)) / 10000.0f;
            float kd = (float)((uint16_t)buf[6] | (uint16_t)(buf[7] << 8)) / 1000.0f;
            Motor_SetPid(ch, kp, ki, kd);
            if (ch == 0xFF) {
                printf("[HID1] PID全局: Kp=%.3f Ki=%.4f Kd=%.3f\r\n", kp, ki, kd);
            }
            else {
                printf("[HID1] PID CH%d: Kp=%.3f Ki=%.4f Kd=%.3f\r\n", ch + 1, kp, ki, kd);
            }
        } break;

        // 设置 PWM 起步偏置
        case 0x05: {
            uint16_t pwm_bias = (uint16_t)buf[1] | (uint16_t)(buf[2] << 8);
            Motor_SetPwmBias(pwm_bias);
            printf("[HID1] PWM Bias=%d\r\n", pwm_bias);
        } break;

        // 设置 PWM 最大占空比
        case 0x06: {
            uint16_t pwm_max = (uint16_t)buf[1] | (uint16_t)(buf[2] << 8);
            Motor_SetPwmMax(pwm_max);
            printf("[HID1] PWM Max=%d\r\n", pwm_max);
        } break;

        default:
            break;
    }
}

bool Motor_IsMoving(uint8_t ch) {
    return motor_states_[ch].active_;
}
