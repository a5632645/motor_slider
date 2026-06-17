# USB MIDI CC 双向联动 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 USB HID 功能不变的前提下，为八路马达线性电位器固件新增 USB MIDI 1.0 CC 双向联动。

**Architecture:** 在现有两个 HID 接口之后追加 IAD + Audio Control + MIDI Streaming 接口。TX 使用 kfifo 环形缓冲区暂存 4 字节 MIDI Event Packet，由 SysTick 1ms 周期通过 Bulk IN EP4 发送。RX 在中断中 NAK 保留数据，主循环直接解析 USB DMA 缓冲区后 ACK。

**Tech Stack:** CH32V203C8T, USBFS, tpusb (USB 描述符生成), kfifo

---

## 文件结构

### 新增文件

| 文件 | 职责 |
|------|------|
| `User/usb/midi_cc.h` | MIDI CC 模块公共 API：初始化、发送、接收处理、常量、CC 映射表 |
| `User/usb/midi_cc.c` | MIDI CC 模块实现：TX FIFO 管理、Bulk 发送/接收逻辑、Event Packet 解析 |

### 修改文件

| 文件 | 改动 |
|------|------|
| `User/usb/usb_desc.cpp` | bcdUSB 0x0110→0x0200；追加 IAD + AC Interface + MIDI Streaming Interface 描述符 |
| `User/usb/usb_impl.h` | 新增 `kMidiEpAddr_In=0x84`, `kMidiEpAddr_Out=0x05` 常量 |
| `User/usb/usb_impl.c` | `UsbImpl_InitAndOpenEndpoints()` 配置 EP4/EP5；`UsbImpl_EpInComplete` 处理 EP4；`UsbImpl_EpOutComplete` 处理 EP5 |
| `User/bsp/ch32v20x_it.c` | `SysTick_Handler()` 追加 `MidiCC_Flush()` |
| `User/app/app.c` | `App_Init()` 调用 `MidiCC_Init()`；主循环调用 `MidiCC_ProcessRx()` |
| `User/app/motor.c` | `Motor_RunControlLoop()` 中 ADC 更新后调用 `MidiCC_CheckAndSend()`；电机停止时清除防回环标志 |

### 不变文件

`usb_device.c/h`, `usb_endpoint.h`, `usb_hardware.h`, `motor_hw.c/h`, `pid.c/h`

---

### Task 1: USB 描述符 — 修改 usb_desc.cpp

**Files:**
- Modify: `User/usb/usb_desc.cpp`

**说明：**
- 修改 `kDevice` 的 `bcdUSB` 从 `0x0110` 为 `0x0200`
- 在 `kConfig` 中两个 HID Interface 之后追加 IAD + Audio Control Interface + MIDI Streaming Interface
- 使用 tpusb 现有基础设施

- [ ] **Step 1: 修改 Device Descriptor 的 bcdUSB**

将：
```cpp
static constexpr auto kDevice =
tpusb::Device{
    0x0110,     // bcdUSB = 1.10
```
改为：
```cpp
static constexpr auto kDevice =
tpusb::Device{
    0x0200,     // bcdUSB = 2.00
```

- [ ] **Step 2: 添加 IAD + AC + MIDI Streaming 描述符到 kConfig**

在现有 `kConfig` 的第二个 HID Interface 闭括号之后、整个 `}` 结束之前，追加 IAD + AC Interface + MIDI Streaming Interface。

需要添加的 include（文件顶部已有 `#include "tpusb/device.hpp"` `#include "tpusb/hid.hpp"`，追加 `#include "tpusb/uac2.hpp"` 和 `#include "tpusb/midiv1.hpp"`）：

```cpp
#include "tpusb/uac2.hpp"
#include "tpusb/midiv1.hpp"
```

在 `kConfig` 的第二个 `tpusb::hid::HID_Interface{...}}` 之后添加：

```cpp
    ,
    /* IAD for Audio Control + MIDI Streaming */
    tpusb::InterfaceAssociation{
        tpusb::InterfaceAssociationInitPack{
            .function_class = 1,     // Audio
            .function_subclass = 3,  // MIDIStreaming
            .function_protocol = 0,
            .function_str_id = 0
        },
        /* Interface 2: Audio Control (空, 无端点) */
        tpusb::Interface{
            tpusb::InterfaceInitPack{
                .interface_no = 2,
                .alter = 0,
                .class_ = 1,         // Audio
                .subclass = 1,       // AudioControl
                .protocol = 0,
                .str_id = 0
            },
            tpusb::uac2::AudioFunction{
                tpusb::uac2::AudioFunctionInitPack{
                    .bcd_adc = 0x0200,
                    .catalog = tpusb::uac2::AudioFunctionCatalog::kOther,
                    .controls = 0x00
                }
            }
        },
        /* Interface 3: MIDI Streaming */
        tpusb::midiv1::MIDIStreamInterface{
            tpusb::InterfaceInitPackClassed{
                .interface_no = 3,
                .alter = 0,
                .protocol = 0,
                .str_id = 0
            },
            tpusb::midiv1::ExternalMidiInJack{1, 1},
            tpusb::midiv1::ExternalMidiOutJack{1, 1},
            tpusb::midiv1::MidiEndpoint<1>{
                tpusb::BulkInitPack{0x84, 64, 0},
                tpusb::midiv1::EndpointJackAssociation<1>{1}
            },
            tpusb::midiv1::MidiEndpoint<1>{
                tpusb::BulkInitPack{0x05, 64, 0},
                tpusb::midiv1::EndpointJackAssociation<1>{1}
            }
        }
    }
```

- [ ] **Step 3: 验证编译**

```bash
cmake --build build
```
Expected: 编译通过，无错误。

---

### Task 2: MIDI CC 模块头文件 — 创建 midi_cc.h

**Files:**
- Create: `User/usb/midi_cc.h`

- [ ] **Step 1: 创建 midi_cc.h**

```c
/**
 * @file    midi_cc.h
 * @brief   USB MIDI CC 双向联动 — 公共 API
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 电机总数 (来自 motor.h 的 kMotorIdx_Count) */
#include "bsp/motor_hw.h"

// ------------------------------------------------------------
// 常量
// ------------------------------------------------------------

/** @brief MIDI Bulk IN 端点地址 */
#define kMidiEpAddr_In   0x84

/** @brief MIDI Bulk OUT 端点地址 */
#define kMidiEpAddr_Out  0x05

/** @brief MIDI Bulk 端点最大包大小 */
#define kMidiEpMpsize    64

/** @brief TX FIFO 大小 (字节), 必须为 2 的幂 */
#define MIDI_TX_FIFO_SIZE 128

/** @brief Code Index Number: Control Change */
#define MIDI_CIN_CC 0x0B

/** @brief MIDI 通道 1 (0-based) */
#define MIDI_DEFAULT_CHANNEL 0

// ------------------------------------------------------------
// CC 映射
// ------------------------------------------------------------

/**
 * @brief MIDI CC 映射条目
 */
struct MidiCcMapping {
    uint8_t channel;  /**< MIDI 通道 (0-based, 0=Channel 1) */
    uint8_t cc_num;   /**< CC 编号 (0~127) */
};

/**
 * @brief 默认 CC 映射表
 */
extern const struct MidiCcMapping kMidiCcMap[kMotorIdx_Count];

// ------------------------------------------------------------
// 公共 API
// ------------------------------------------------------------

/**
 * @brief 初始化 MIDI CC 模块
 */
void MidiCC_Init(void);

/**
 * @brief 发送 MIDI CC 消息 (阻塞式: FIFO 满则循环等待)
 * @param idx       电位器编号
 * @param cc_value  CC 值 (0~127)
 */
void MidiCC_Send(uint8_t idx, uint8_t cc_value);

/**
 * @brief 周期性 Flush: 将 TX FIFO 数据通过 Bulk IN 发送
 *        由 SysTick_Handler 和 EP4 IN Complete 调用
 */
void MidiCC_Flush(void);

/**
 * @brief 处理接收到的 MIDI 数据
 *        由 App_Loop 主循环调用
 */
void MidiCC_ProcessRx(void);

/**
 * @brief 检查并发送 CC (如需): 映射 ADC → CC, 去重, 防回环
 * @param idx     电位器编号
 * @param raw_adc 原始 ADC 值 (0~4095)
 */
void MidiCC_CheckAndSend(uint8_t idx, uint16_t raw_adc);

/**
 * @brief 设置防回环标志
 * @param idx    电位器编号
 * @param active true=正在响应 RX 驱动
 */
void MidiCC_SetRxActive(uint8_t idx, bool active);
```

---

### Task 3: MIDI CC 模块实现 — 创建 midi_cc.c

**Files:**
- Create: `User/usb/midi_cc.c`

- [ ] **Step 1: 创建 midi_cc.c — 静态变量和映射表**

```c
/**
 * @file    midi_cc.c
 * @brief   USB MIDI CC 双向联动 — 实现
 */
#include "midi_cc.h"

#include <string.h>

#include "config.h"
#include "usb/usb_device.h"
#include "usb/usb_hardware.h"
#include "usb/usb_impl.h"
#include "util/kfifo.h"

// ------------------------------------------------------------
// 静态变量
// ------------------------------------------------------------

/* TX FIFO */
static struct {
    struct Kfifo fifo;
    uint8_t buf[MIDI_TX_FIFO_SIZE];
} midi_tx_fifo_;

/* EP4 DMA 缓冲区 */
__attribute__((aligned(4))) static uint8_t midi_ep4_tx_buf_[kMidiEpMpsize];
static volatile bool midi_tx_busy_;
static bool midi_zlp_needed_;

/* EP5 DMA 缓冲区 */
__attribute__((aligned(4))) uint8_t midi_ep5_rx_buf_[kMidiEpMpsize];
volatile uint8_t midi_rx_pending_len_;

/* 上次发送的 CC 值 (去重) */
static uint8_t last_sent_cc_[kMotorIdx_Count];

/* 防回环标志 */
static bool fader_rx_active_[kMotorIdx_Count];

// ------------------------------------------------------------
// 默认 CC 映射表
// ------------------------------------------------------------

const struct MidiCcMapping kMidiCcMap[kMotorIdx_Count] = {
    {0, 0},   /* 电位器0 → Channel 1, CC 0  */
    {0, 1},   /* 电位器1 → Channel 1, CC 1  */
    {0, 2},   /* 电位器2 → Channel 1, CC 2  */
    {0, 3},   /* 电位器3 → Channel 1, CC 3  */
    {0, 4},   /* 电位器4 → Channel 1, CC 4  */
    {0, 5},   /* 电位器5 → Channel 1, CC 5  */
    {0, 6},   /* 电位器6 → Channel 1, CC 6  */
    {0, 7},   /* 电位器7 → Channel 1, CC 7  */
};
```

- [ ] **Step 2: 实现 MidiCC_Init**

```c
void MidiCC_Init(void) {
    midi_tx_fifo_.fifo.mask = MIDI_TX_FIFO_SIZE - 1;
    midi_tx_fifo_.fifo.wpos = 0;
    midi_tx_fifo_.fifo.rpos = 0;
    midi_tx_busy_ = false;
    midi_zlp_needed_ = false;
    midi_rx_pending_len_ = 0;
    for (int i = 0; i < kMotorIdx_Count; i++) {
        last_sent_cc_[i] = 0xFF;  /* 初始化为无效值，确保首次发送 */
        fader_rx_active_[i] = false;
    }
}
```

- [ ] **Step 3: 实现 MidiCC_Send (阻塞式入队)**

```c
void MidiCC_Send(uint8_t idx, uint8_t cc_value) {
    if (!HID_IsConnected())
        return;

    /* 构造 4 字节 USB MIDI Event Packet */
    uint8_t packet[4];
    packet[0] = MIDI_CIN_CC;                                    /* CIN=0x0B, Cable=0 */
    packet[1] = (uint8_t)(0xB0 | kMidiCcMap[idx].channel);      /* Status: CC + Channel */
    packet[2] = kMidiCcMap[idx].cc_num;                         /* CC 编号 */
    packet[3] = cc_value;                                        /* CC 值 */

    /* 循环等待 FIFO 空间 (由 SysTick 中断消费腾空) */
    while (Kfifo_FreeSpace(&midi_tx_fifo_.fifo) < 4) {
        /* SysTick_Handler 中的 MidiCC_Flush() 会在此期间响应 */
    }
    Kfifo_TryPush(&midi_tx_fifo_.fifo, packet, 4);
}
```

- [ ] **Step 4: 实现 MidiCC_Flush (含 ZLP 处理)**

```c
void MidiCC_Flush(void) {
    if (midi_tx_busy_)
        return;

    uint32_t available = Kfifo_Size(&midi_tx_fifo_.fifo);

    if (available == 0) {
        /* FIFO 空: 检查是否需要发送 ZLP */
        if (midi_zlp_needed_) {
            midi_zlp_needed_ = false;
            USBFSD->UEP4_TX_LEN = 0;
            midi_tx_busy_ = true;
            USBFSD->UEP4_TX_CTRL = (USBFSD->UEP4_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
        }
        return;
    }

    uint32_t to_read = (available > kMidiEpMpsize) ? kMidiEpMpsize : available;
    uint32_t total = 0;

    /* 从 kfifo 读取 (可能跨越环形缓冲区边界) */
    while (total < to_read) {
        uint32_t chunk;
        uint8_t* src = Kfifo_ContinueReadBegin(&midi_tx_fifo_.fifo, &chunk);
        uint32_t need = to_read - total;
        if (chunk > need)
            chunk = need;
        if (chunk == 0)
            break;
        memcpy(midi_ep4_tx_buf_ + total, src, chunk);
        Kfifo_ContinueReadEnd(&midi_tx_fifo_.fifo, chunk);
        total += chunk;
    }

    midi_zlp_needed_ = (total == kMidiEpMpsize);
    midi_tx_busy_ = true;

    USBFSD->UEP4_TX_LEN = total;
    USBFSD->UEP4_TX_CTRL = (USBFSD->UEP4_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
}
```

- [ ] **Step 5: 实现 MidiCC_ProcessRx**

```c
void MidiCC_ProcessRx(void) {
    if (midi_rx_pending_len_ == 0)
        return;

    /* 直接解析 USB DMA 缓冲区中的 Event Packet */
    uint8_t len = midi_rx_pending_len_;
    for (uint8_t offset = 0; offset + 3 < len; offset += 4) {
        uint8_t status = midi_ep5_rx_buf_[offset + 1];

        /* 仅处理 Control Change 消息 */
        if ((status & 0xF0) != 0xB0)
            continue;

        uint8_t channel = status & 0x0F;
        uint8_t cc_num  = midi_ep5_rx_buf_[offset + 2];
        uint8_t cc_val  = midi_ep5_rx_buf_[offset + 3];

        /* 查映射表 */
        for (uint8_t i = 0; i < kMotorIdx_Count; i++) {
            if (kMidiCcMap[i].channel == channel && kMidiCcMap[i].cc_num == cc_num) {
                /* 反向映射: [0,127] → [0,4095] */
                uint16_t target_pos = (uint16_t)((uint32_t)cc_val * 4095 / 127);
                fader_rx_active_[i] = true;
                Motor_SetTarget(i, target_pos);
                break;
            }
        }
    }

    /* 全部消耗, ACK EP5 */
    midi_rx_pending_len_ = 0;
    USBFSD->UEP5_RX_CTRL = (USBFSD->UEP5_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
}
```

- [ ] **Step 6: 实现 MidiCC_CheckAndSend 和防回环接口**

```c
void MidiCC_CheckAndSend(uint8_t idx, uint16_t raw_adc) {
    if (idx >= kMotorIdx_Count)
        return;

    /* 防回环: RX 驱动中的通道不发送 TX */
    if (fader_rx_active_[idx])
        return;

    /* 映射 ADC 到 CC 值 [0,127] */
    uint8_t cc_value = (uint8_t)((uint32_t)raw_adc * 127 / 4095);

    /* 去重: 与上次发送值相同则跳过 */
    if (cc_value == last_sent_cc_[idx])
        return;

    last_sent_cc_[idx] = cc_value;
    MidiCC_Send(idx, cc_value);
}

void MidiCC_SetRxActive(uint8_t idx, bool active) {
    if (idx < kMotorIdx_Count)
        fader_rx_active_[idx] = active;
}
```

---

### Task 4: USB IMPL — 端点配置与中断处理

**Files:**
- Modify: `User/usb/usb_impl.h`
- Modify: `User/usb/usb_impl.c`

- [ ] **Step 1: usb_impl.h — 声明 EP4/EP5 常量**

在 `usb_impl.h` 的文件末尾或适当位置，添加 EP4/EP5 的声明。注意 `kMidiEpAddr_In` 和 `kMidiEpAddr_Out` 已在 `midi_cc.h` 中定义，但 `usb_impl.h` 需要知道端点模式配置用的常量。

实际不需要在 `usb_impl.h` 添加额外内容，因为 MIDI 端点配置将在 `UsbImpl_InitAndOpenEndpoints()` 中通过直接写寄存器完成。

- [ ] **Step 2: usb_impl.c — 修改 UsbImpl_InitAndOpenEndpoints()**

追加 EP4/EP5 的初始化：

```c
    /* EP4: MIDI Bulk IN (TX) */
    USBFSD->UEP4_1_MOD |= USBFS_UEP4_TX_EN;
    USBFSD->UEP4_DMA = (uint32_t)midi_ep4_tx_buf_;
    USBFSD->UEP4_TX_LEN = 0;
    USBFSD->UEP4_TX_CTRL = USBFS_UEP_T_RES_NAK;

    /* EP5: MIDI Bulk OUT (RX) */
    USBFSD->UEP5_6_MOD |= USBFS_UEP5_RX_EN;
    USBFSD->UEP5_DMA = (uint32_t)midi_ep5_rx_buf_;
    USBFSD->UEP5_RX_CTRL = USBFS_UEP_R_RES_ACK;

    /* 初始化 MIDI CC 模块 */
    MidiCC_Init();
```

注意：需要在文件顶部添加 `#include "usb/midi_cc.h"`。

- [ ] **Step 3: usb_impl.c — 修改 UsbImpl_EpInComplete**

在 EP2 的 case 之后添加 EP4 处理：

```c
        case kMidiEpAddr_In & 0xf:
            midi_tx_busy_ = false;
            USBFSD->UEP4_TX_CTRL ^= USBFS_UEP_T_TOG;
            USBFSD->UEP4_TX_CTRL = (USBFSD->UEP4_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
            MidiCC_Flush();
            break;
```

- [ ] **Step 4: usb_impl.c — 修改 UsbImpl_EpOutComplete**

在 EP3 的 case 之后添加 EP5 处理：

```c
        case kMidiEpAddr_Out & 0xf:
            if (midi_rx_pending_len_ == 0) {
                uint16_t rx_count = USBFSD->RX_LEN;
                if (rx_count > kMidiEpMpsize)
                    rx_count = kMidiEpMpsize;
                if (rx_count == 0) {
                    /* ZLP: 直接 ACK */
                    USBFSD->UEP5_RX_CTRL = (USBFSD->UEP5_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
                } else {
                    midi_rx_pending_len_ = (uint8_t)rx_count;
                    USBFSD->UEP5_RX_CTRL = (USBFSD->UEP5_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
                }
            }
            break;
```

`midi_ep5_rx_buf_` 和 `midi_rx_pending_len_` 需要在 `usb_impl.c` 中声明为外部变量或在 `midi_cc.c` 中定义为非 static。方案：在 `midi_cc.c` 中将这两个变量改为非 static（去掉 static），并在 `midi_cc.h` 中声明为 extern。

在 `midi_cc.h` 中添加：
```c
/* USB DMA 缓冲区 (供 usb_impl.c 中断处理访问) */
extern __attribute__((aligned(4))) uint8_t midi_ep5_rx_buf_[kMidiEpMpsize];
extern volatile uint8_t midi_rx_pending_len_;
```

- [ ] **Step 5: 验证编译**

```bash
cmake --build build
```
Expected: 编译通过。

---

### Task 5: SysTick 集成 — 修改 ch32v20x_it.c

**Files:**
- Modify: `User/bsp/ch32v20x_it.c`

- [ ] **Step 1: 在 SysTick_Handler 中添加 MidiCC_Flush()**

```c
#include "usb/midi_cc.h"

void SysTick_Handler(void) {
    SysTick->SR = 0;
    Tick_Increment();
    HID_Flush();
    MidiCC_Flush();    /* 新增: MIDI TX Flush */
}
```

---

### Task 6: App 层集成 — 修改 app.c

**Files:**
- Modify: `User/app/app.c`

- [ ] **Step 1: 在 App_Init 和 App_Loop 中集成 MIDI CC**

```c
#include "usb/midi_cc.h"

void App_Init(void) {
    Motor_InitControl();
    MidiCC_Init();
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
        MidiCC_ProcessRx();    /* 新增: 处理接收到的 MIDI CC */
    }
}
```

---

### Task 7: 电机控制集成 — 修改 motor.c

**Files:**
- Modify: `User/app/motor.c`

- [ ] **Step 1: 包含 midi_cc.h**

```c
#include "usb/midi_cc.h"
```

- [ ] **Step 2: 在 Motor_RunControlLoop 中添加 CC 发送和防回环清除**

在 `Motor_RunControlLoop` 中，每路电机处理完成后（PWM 设置之后或 active 变为 false 时），添加：

在 `if (!motor_states_[i].active_)` 分支中，跳过后续处理；但我们需要在 ADC 更新后、active 检查之前调用 CC 检查。修改后的循环体：

```c
for (int i = 0; i < kMotorIdx_Count; i++) {
    motor_states_[i].current_adc_ = raw_adc[i];

    /* MIDI CC 检查 (防回环: 会自动跳过 fader_rx_active_ 的通道) */
    MidiCC_CheckAndSend(i, raw_adc[i]);

    /* 非活跃电机：跳过，PWM 保持 0 */
    if (!motor_states_[i].active_)
        continue;

    /* ... 原有停止/超时/PID逻辑 ... */

    /* 到达目标或超时停止后, 清除防回环标志 */
    if (!motor_states_[i].active_) {
        MidiCC_SetRxActive(i, false);
    }
}
```

注意：需要在 `Motor_RunControlLoop` 中现有的 `if (!motor_states_[i].active_) continue;` 之前添加 `MidiCC_CheckAndSend(i, raw_adc[i]);`。

并且在每路 `active_` 从 true→false 的位置（到达目标停止处和超时停止处）清除防回环标志。

---

### Task 8: 最终验证

- [ ] **Step 1: 完整编译**

```bash
cmake --build build
```
Expected: 编译通过，无警告。

- [ ] **Step 2: 检查二进制大小**

确认固件大小在合理范围内（不应显著增加，MIDI 功能代码量很小）。

- [ ] **Step 3: 代码审查**

确认所有改动符合 `AGENTS.md` 中的编码约定：
- 全局变量以 `_` 结尾 ✓ (`midi_ep5_rx_buf_`, `midi_rx_pending_len_`)
- 头文件函数声明 `模块_动作` 格式 ✓ (`MidiCC_Init`, `MidiCC_Send`, `MidiCC_Flush`)
- 枚举值前缀 `k` ✓ (`kMidiEpAddr_In`, `kMidiEpAddr_Out`, `kMidiCcMap`, `MIDI_CIN_CC`)
- 无 `extern` 全局变量替代方案：`midi_ep5_rx_buf_` 和 `midi_rx_pending_len_` 在 `midi_cc.c` 中定义，`midi_cc.h` 中 `extern` 声明供 `usb_impl.c` 访问
- 无 typedef 对结构体/枚举
