# 闭环控制 + HID1 命令协议 设计

> **日期:** 2025-06-12
> **关联:** `2025-06-12-control-state-machine.md`, `2025-06-12-potentiometer-ui-rework.md`

## HID1 通信协议

### 64 字节状态报告 (IN, MCU → Host)

| 偏移 | 大小 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | `flags` | bit0=running, bit1=any_active |
| 1 | 1 | `active_flags` | 每路 1 bit, bit0=CH1 active, ... bit7=CH8 active |
| 2~17 | 16 | `adc[8]` | 8 × uint16 LE, 当前 ADC 值 (0~4095) |
| 18~33 | 16 | `target[8]` | 8 × uint16 LE, 目标 ADC 值 (0~4095) |
| 34~49 | 16 | `duty[8]` | 8 × uint16 LE, 当前占空比 (0~999) |
| 50~63 | 14 | 保留 | 填 0 |

### 64 字节命令报告 (OUT, Host → MCU)

| 偏移 | 大小 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | `cmd` | 命令 ID |
| 1~16 | 16 | `payload` | 命令参数 |
| 17~63 | 47 | 保留 | 填 0 |

**命令定义:**

| cmd | 名称 | payload 格式 | 行为 |
|-----|------|-------------|------|
| `0x01` | 设置目标 | bytes 1-16: 8 × uint16 LE 目标值 | 调用 `Motor_SetTarget(i, val)` 激活各电机 |
| `0x02` | 读取状态 | 无 | 强制刷新 IN 报告（已由 1kHz 状态机自动完成） |
| `0x03` | 停止所有 | 无 | 调用 `Motor_StopAll()` 停所有电机 |

---

## 电机停止条件

每路电机独立状态机，在 `Motor_RunControlLoop` 中执行：

```
active_=false ───→ 收到新目标 ───→ active_=true, timeout_=1000
                    ┌──────────────┴──────────────┐
                    │ 每 1ms 迭代:                  │
                    │  if |target-adc| ≤ 2          │
                    │    → active_=false, PWM=0     │
                    │  else if --timeout_ == 0      │
                    │    → active_=false, PWM=0     │
                    │  else                         │
                    │    → PID → PWM                │
                    └──────────────────────────────┘
```

停止后电机输出为 0（coast 模式），用户可以自由拨动电位器。

---

## 固件改动

| 文件 | 改动 |
|------|------|
| `motor.h` | `struct MotorState` 新增 `bool active_`, `uint16_t timeout_` |
| | `Motor_GetStatus()` 新增 `uint8_t* active_flags` 参数 |
| | `Motor_InitControl()` 初始化 `active_=false, timeout_=0` |
| `motor.c` | `Motor_RunControlLoop()` 加入 active/timeout 判断 |
| | `Motor_SetTarget()` 激活电机 + 重置 timeout + 重置 PID 积分 |
| | `Motor_StopAll()` 设所有 `active_=false` |
| | `Motor_GetStatus()` 填充 `active_flags` |
| `usb_impl.h` | `HID1_ACTIVE_FLAGS = 1` 取代 `HID1_RSV` |
| | `HID1_SendStatus()` 新增 `uint8_t active_flags` 参数 |
| `usb_impl.c` | `HID1_SendStatus()` 填充 `active_flags` 字节 |
| | `HID1_ProcessCommand()` 实现 `0x01` 和 `0x03` 命令 |
| `main.c` | CONTROL 状态调用 `Motor_GetStatus` + `HID1_SendStatus` 同步新增参数 |

## 上位机改动

| 改动 | 说明 |
|------|------|
| `HidWorker` 增加 `send_target(ch, value)` 方法 | 组装 OUT 报告 → `device.write()` |
| `PotWidget._on_slider_changed` | 拖动滑块后调用 `HidWorker.send_target()` |
| `StatusReport` 解析 `active_flags` | 可选显示 |

### OUT 报告发送实现

```python
def send_targets(self, targets: List[int]):
    """通过 HID1 OUT 发送 8 路目标值"""
    report = bytearray(64)
    report[0] = 0x01  # cmd: 设置目标
    for i in range(8):
        report[1 + i*2] = targets[i] & 0xFF
        report[1 + i*2 + 1] = (targets[i] >> 8) & 0xFF
    try:
        self._device.write(bytes(report))
    except Exception:
        pass  # 连接断开时静默忽略
```

---

## 可配置参数 (`User/config.h`)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `CTRL_ERROR_THRESHOLD` | 2 | ADC 误差阈值，在此范围内视为到达目标 |
| `CTRL_TIMEOUT_MS` | 1000 | 闭环超时 (ms)，超时未到达则停电机 |
| `PID_DEFAULT_KP` | 0.5 | 默认比例增益 |
| `PID_DEFAULT_KI` | 0.01 | 默认积分增益 |
| `PID_DEFAULT_KD` | 0.1 | 默认微分增益 |
| `PID_OUTPUT_MIN` | -999.0 | PID 输出下限（对应 PWM=0） |
| `PID_OUTPUT_MAX` | 999.0 | PID 输出上限（对应 PWM=999） |
| `PID_INTEGRAL_LIMIT` | 500.0 | 积分限幅，防止积分饱和 |
| `PWM_MIN_START_DUTY` | 100 | 电机起步最小占空比，克服静摩擦 (0~999) |
| `MOTOR3_DEADBAND` | 20 | 电机 3 开关死区（GPIO 控制） |
| `HID_FIFO_SIZE` | 512 | HID0 printf FIFO 大小 (字节) |

---

## 附：现有代码接入 config.h

将当前固件中散落的硬编码数值替换为 `config.h` 宏引用。

| 文件 | 行/位置 | 硬编码值 | 替换为 |
|------|---------|---------|--------|
| `motor.c` | `Motor_InitControl` | `Pid_Init(..., 0.5f, 0.01f, 0.1f)` | `PID_DEFAULT_KP`, `KI`, `KD` |
| `pid.c` | `Pid_Init` | `output_min_ = -999.0f` | `PID_OUTPUT_MIN` |
| | | `output_max_ = 999.0f` | `PID_OUTPUT_MAX` |
| | | `integral_limit_ = 500.0f` | `PID_INTEGRAL_LIMIT` |
| `motor.c` | `Motor_RunControlLoop` 电机 3 | `if (diff > 20)` | `MOTOR3_DEADBAND` |
| `usb_impl.c` | 文件顶部 | `#define HID_FIFO_SIZE 512` | 改为 `#include "config.h"` 移除重复定义 |
