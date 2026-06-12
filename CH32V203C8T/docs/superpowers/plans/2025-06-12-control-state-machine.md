# 控制流水线状态机 + HID 数据分离 实现计划

> **日期:** 2025-06-12
> **关联文档:** `2025-06-12-motor-control.md`（原始硬件驱动实现）

---

## 目标

将当前控制流水线改为**状态机驱动**，实现：
1. ADC 启动/电机控制/HID 上报由状态机管理
2. 1kHz 固定执行频率
3. HID 命令处理和 HID0 刷新保留在原位置（不混入状态机）

---

## 当前架构（改造前）

```
SysTick ISR (1ms):
  Tick_Increment()
  Motor_StartAdcConversion()     ← ADC 在 ISR 中触发
  HID_Flush()                    ← HID0 printf 刷新（保持原位）

DMA ISR:
  motor_adc_ready_ = true

Main loop:
  if (Motor_IsAdcReady())
    Motor_RunControlLoop()       ← 包含 PID + PWM + HID1_SendStatus
  HID1_ProcessCommand()          ← HID1 命令处理（保持原位）
```

## 目标架构

```
SysTick ISR (1ms):
  Tick_Increment()
  HID_Flush()                    ← ※ 原位保留，仅移除 ADC 启动

Main loop ── 状态机:
                     Tick 变化
    IDLE ──────────────────────────→ ADC_START
     ↑                                   │
     │                              Motor_StartAdcConversion()
     │                                   ↓
     │                              ADC_WAIT ← 轮询 motor_adc_ready_
     │                                   │
     │                              ADC 就绪
     │                                   ↓
     │                              CONTROL
     │                                Motor_RunControlLoop()  ① 纯 PID+PWM
     │                                HID1_SendStatus(...)    ② 写 HID 状态
     │                                   │
     └──────────── 回到 IDLE ────────────┘

  HID1_ProcessCommand()          ← ※ 原位保留，每次主循环迭代执行
```

---

## 状态定义

```
enum CtrlState {
    kCtrlState_Idle,       // 等待 1ms tick 到达
    kCtrlState_AdcStart,   // 触发 ADC 转换
    kCtrlState_AdcWait,    // 等待 ADC+DMA 完成
    kCtrlState_Control,    // 电机 PID → PWM → HID 状态上报
};
```

### 各状态行为

| 状态 | 触发 | 执行动作 | 下一状态 |
|------|------|---------|---------|
| `IDLE` | 每次迭代 | 检查 `Tick_Get()` 是否变化 | 变化 → `ADC_START` |
| `ADC_START` | 单次 | `Motor_StartAdcConversion()` | → `ADC_WAIT` |
| `ADC_WAIT` | 每次迭代 | 检查 `Motor_IsAdcReady()` | true → `CONTROL` |
| `CONTROL` | 单次 | `Motor_RunControlLoop()` + `HID1_SendStatus()` | → `IDLE` |

### 时序

```
Tick n       Tick n+1
  │              │
  ▼              ▼
IDLE→ADC→WAIT→CTRL→IDLE→ADC→WAIT→CTRL→...
  │<── ~14μs ──>│
  │<── ~1ms ───────>│
```

- `ADC_START` → `CONTROL` 耗时约 14~50μs
- 其余大部分时间落在 `IDLE` 状态等待下一 tick

---

## 涉及改动

### 1. `ch32v20x_it.c` — SysTick_Handler

```c
void SysTick_Handler(void)
{
    SysTick->SR = 0;
    Tick_Increment();
    // Motor_StartAdcConversion();  ← 移除，由状态机触发
    HID_Flush();                     // ← 保持原位
}
```

### 2. `motor.h` / `motor.c` — 拆分职责

**Motor_RunControlLoop()** 末尾移除 HID1_SendStatus 块：

```c
void Motor_RunControlLoop(void)
{
    if (!motor_adc_ready_) return;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_states_[i].current_adc_ = motor_adc_dma_buf_[i];
        // ... PID 计算 ...
        Motor_SetPwm(i, motor_states_[i].dir_, motor_states_[i].duty_);
    }

    // ※ HID1_SendStatus 调用已移出，由 main 中的状态机调用
}
```

新增 `Motor_GetStatus()` 供 main 调用 HID1_SendStatus 时获取数据：

```c
// motor.h 声明:
void Motor_GetStatus(uint16_t adc[8], uint16_t target[8], uint16_t duty[8]);

// motor.c 实现:
void Motor_GetStatus(uint16_t adc[8], uint16_t target[8], uint16_t duty[8])
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        adc[i]    = motor_states_[i].current_adc_;
        target[i] = motor_states_[i].target_adc_;
        duty[i]   = motor_states_[i].duty_;
    }
}
```

### 3. `main.c` — 状态机 + 保留原位调用

```c
#include "motor.h"
#include "usb/usb_impl.h"
// ...

enum CtrlState {
    kCtrlState_Idle,
    kCtrlState_AdcStart,
    kCtrlState_AdcWait,
    kCtrlState_Control,
};

static enum CtrlState ctrl_state_ = kCtrlState_Idle;
static uint32_t last_ctrl_tick_ = 0;

int main(void)
{
    // ... 初始化（不变）...

    while (1)
    {
        /* ── 控制状态机（1kHz） ── */
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
            /* 非忙等待，每次主循环只检查一次 */
            break;

        case kCtrlState_Control:
            Motor_RunControlLoop();          // ① PID → PWM
            {
                uint16_t adc[8], target[8], duty[8];
                Motor_GetStatus(adc, target, duty);
                HID1_SendStatus(adc, target, duty);  // ② HID 数据写
            }
            ctrl_state_ = kCtrlState_Idle;
            break;
        }

        /* ── HID1 命令处理（保持原位，每次迭代执行） ── */
        HID1_ProcessCommand();
    }
}
```

---

## 关键注意事项

### 1. ADC_WAIT 非忙等待

`ADC_WAIT` 状态下**不阻塞**，每次主循环只检查一次 `Motor_IsAdcReady()` 就回到 switch 出口。
ADC 转换期间（~14μs）主循环可以立即进入下一迭代处理 `HID1_ProcessCommand()`。

### 2. HID TX DMA vs RX DMA

- **TX DMA**（EP2 IN）：硬件**读取**缓冲区 → CPU 可以随时安全更新 `hid1_report_buf_`
- **RX DMA**（EP2 OUT）：硬件**写入**缓冲区 → 需在 OUT 完成中断后读取

当前 `HID1_SendStatus()` 只写缓冲区、不操作 DMA 寄存器，
与状态机解耦后也无需额外保护。

### 3. 控制周期

状态机由 `Tick_Get()` 变化驱动，保证 **1kHz 固定频率**。
即使 `ADC_WAIT` 偶尔跨 tick，状态机也只在 tick 边界触发下一轮。

### 4. HID1_SendStatus 数据来源

`Motor_GetStatus()` 在 `Motor_RunControlLoop()` 之后调用，
此时 `motor_states_[i]` 已包含最新的 ADC/Target/Duty 值。
