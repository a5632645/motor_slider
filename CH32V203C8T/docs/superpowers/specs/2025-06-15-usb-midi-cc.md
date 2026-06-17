# USB MIDI CC 双向联动 设计

> **日期:** 2025-06-15
> **关联:** `midi_cc.md`, `2025-06-12-closed-loop-control.md`, `2025-06-12-motor-control.md`

## 1. 概述

在现有 USB HID 功能不变的前提下，扩展为 USB 复合设备，新增 USB MIDI 1.0 类接口，实现八路马达线性电位器位置与 MIDI CC 消息的双向联动。

### 约束

- 不修改现有 HID 描述符、端点配置、轮询行为
- 不引入动态内存分配，所有缓冲区静态分配

---

## 2. USB 设备结构

### 2.1 复合设备拓扑

```
USB Device (bcdUSB=0x0200, Class=0xEF/0x02/0x01 Composite)
└── Configuration Descriptor (1 config)
    ├── Interface 0: HID0 (printf 调试输出, IN only) — 不变
    ├── Interface 1: HID1 (电机控制, IN + OUT) — 不变
    ├── Interface Association Descriptor (IAD)
    │   ├── bFunctionClass=1 (Audio)
    │   ├── bFunctionSubClass=3 (MIDIStreaming)
    │   └── bFunctionProtocol=0
    │   │
    │   ├── Interface 2: Audio Control (bClass=1, bSubClass=1)
    │   │   └── 无端点
    │   │
    │   └── Interface 3: MIDI Streaming (bClass=1, bSubClass=3)
    │       ├── Bulk IN  EP (0x84, 64 bytes) — MCU → Host
    │       └── Bulk OUT EP (0x05, 64 bytes) — Host → MCU
```

### 2.2 端点分配

| 端点 | 方向 | 用途 | 类型 | 包大小 |
|------|------|------|------|--------|
| EP0 | 双向 | 控制传输 | Control | 64 |
| EP1 | IN (0x81) | HID0 printf | Interrupt | 64 |
| EP2 | IN (0x82) | HID1 状态上报 | Interrupt | 64 |
| EP3 | OUT (0x03) | HID1 命令接收 | Interrupt | 64 |
| **EP4** | **IN (0x84)** | **MIDI Bulk TX** | **Bulk** | **64** |
| **EP5** | **OUT (0x05)** | **MIDI Bulk RX** | **Bulk** | **64** |

### 2.3 描述符变更

**Device Descriptor (`usb_desc.cpp`)**
- `bcdUSB`: `0x0110` → `0x0200`
- `bDeviceClass`/`bDeviceSubClass`/`bDeviceProtocol`: 保持 `0xEF`/`0x02`/`0x01`（`.Composite()` 调用）

**Configuration Descriptor (`usb_desc.cpp`)**
- 在现有两个 HID Interface 之后追加 IAD + AC Interface + MIDI Streaming Interface
- 使用 tpusb 基础设施：`InterfaceAssociation`, `uac2::AudioFunction`, `midiv1::MIDIStreamInterface`, `midiv1::MidiEndpoint`

**字符串描述符**
- 保持不变

---

## 3. MIDI 数据格式

### 3.1 USB MIDI Event Packet (4 字节)

```
Byte 0: Cable Number (高4位) | Code Index Number (低4位)
Byte 1: MIDI Status Byte  (e.g. 0xB0 | channel)
Byte 2: MIDI Data Byte 1  (CC 编号)
Byte 3: MIDI Data Byte 2  (CC 值)
```

本设计仅处理 **Control Change (CC)** 消息，CIN = `0x0B`，Cable Number = `0x00`。

### 3.2 数值映射

```
电位器原始值 (0~4095) → MIDI CC (0~127)
  cc_value = raw_adc * 127 / 4095

MIDI CC (0~127) → 目标位置 (0~4095)
  target_pos = cc_value * 4095 / 127
```

---

## 4. TX 方向：电位器 → MIDI CC 发送

### 4.1 触发条件

- 在 1kHz 控制循环中，每路 ADC 更新后检查位置变化
- 仅当映射后的 CC 值与 `last_sent_cc[i]` **不同**时，才发送

### 4.2 阻塞接口模式

FIFO 满时**循环等待**而非丢弃（MIDI CC 消息不允许丢包），Flush 全部由 SysTick 驱动：

```c
// 检查连接 → 等待空间 → 入队（不触发 Flush）
void MidiCC_Send(uint8_t idx, uint8_t cc_value) {
    if (!HID_IsConnected())       // USB 未连接时跳过
        return;
    // 构造 4 字节 Event Packet
    uint8_t packet[4] = {
        0x0B,                     // CIN=0x0B (CC), Cable=0
        (uint8_t)(0xB0 | kMidiCcMap[idx].channel),
        kMidiCcMap[idx].cc_num,
        cc_value
    };
    // 等待 FIFO 有足够空间（循环等待，由 SysTick 消费腾空）
    while (Kfifo_FreeSpace(&midi_tx_fifo_.fifo) < 4) {
        // SysTick_Handler 中的 MidiCC_Flush() 会在此循环期间被响应
    }
    Kfifo_TryPush(&midi_tx_fifo_.fifo, packet, 4);
}
```

- `MidiCC_Send()` 为**阻塞式**：入队前循环等待 FIFO 空间，入队后立即返回
- **不调用** `MidiCC_Flush()` — 全部由 `SysTick_Handler` 周期性发送
- USB 未连接时 (`using_configuration == 0`)，静默丢弃，不浪费 CPU
- 等待发生在 **主循环上下文**（`Motor_RunControlLoop` 内），SysTick 中断仍正常响应并消费 FIFO

### 4.3 TX 数据流

```
Motor_RunControlLoop()  — 1kHz 控制上下文
  → 读取 ADC, 计算映射 cc_value
  → if !fader_rx_active[i] && cc_value != last_sent_cc[i]:
      → MidiCC_Send(i, cc_value)          // 仅入队，不 Flush
      → last_sent_cc[i] = cc_value

SysTick_Handler (1ms)    — 周期性发送
  → MidiCC_Flush()
    → if midi_tx_busy: return
    → 从 midi_tx_fifo_ 读取最多 64 字节 (16 packets)
    → 填充 midi_ep4_tx_buf_
    → midi_tx_busy = true
    → USBFSD->UEP4_TX_LEN = to_read
    → 触发 Bulk IN 传输

EP4 IN Complete 中断      — 流水线递送
  → midi_tx_busy = false
  → 递归 MidiCC_Flush() 直到 FIFO 空
```

### 4.4 Bulk 零包 (ZLP) 处理

Bulk 传输要求任何 **恰好 64 字节**（wMaxPacketSize 整数倍）的数据包后，需发送零长度包 (ZLP) 以标记传输结束，否则主机会等待更多数据。

```c
// Flush 内部逻辑：
uint32_t to_read = Kfifo_Size(&midi_tx_fifo_);
if (to_read == 0) {
    if (midi_zlp_needed_) {
        midi_zlp_needed_ = false;
        USBFSD->UEP4_TX_LEN = 0;          // 发送 ZLP
        midi_tx_busy = true;
    }
    return;
}
if (to_read > 64)
    to_read = 64;
// 从 FIFO 读取到 midi_ep4_tx_buf_
midi_zlp_needed_ = (to_read == 64);       // 若恰好 64 字节，标记需要 ZLP
USBFSD->UEP4_TX_LEN = to_read;
midi_tx_busy = true;
```

- `midi_zlp_needed_` 在发送 64 字节时置位
- 下次 `MidiCC_Flush()` 在 FIFO 空时检测到此标志，发送 ZLP
- ZLP 发送后清除标志，恢复正常 NAK 状态

### 4.5 Flush 调用链

```
SysTick_Handler (1ms)
  → Tick_Increment()
  → HID_Flush()              // HID0 printf 原样保留
  → MidiCC_Flush()           // MIDI TX flush（新增）

EP4 IN Complete 中断
  → midi_tx_busy = false
  → MidiCC_Flush()           // 递归发送下一批
```

- **SysTick** 提供周期性发送机会，确保 FIFO 中的数据不会长期滞留
- **EP4 IN Complete** 实现流水线：一批发送完成后立即发送下一批，最大化吞吐

### 4.6 缓冲区

```c
#define MIDI_TX_FIFO_SIZE 128   // 32 个 MIDI Event Packet
static struct {
    struct Kfifo fifo;
    uint8_t buf[MIDI_TX_FIFO_SIZE];
} midi_tx_fifo_;

__attribute__((aligned(4))) static uint8_t midi_ep4_tx_buf_[64];

static volatile bool midi_tx_busy_;
static bool midi_zlp_needed_;   // 非 volatile，仅主循环/SysTick 访问
```

---

## 5. RX 方向：MIDI CC → 电位器驱动

### 5.1 数据流

RX 采用**中断仅记录 + NAK，主线程消费**的简单设计：

```
EP5 OUT Complete (中断)
  → if midi_rx_pending_len_ > 0:
      return                                    // 上一包尚未消费，NAK 已设置
  → len = USBFSD->RX_LEN
  → if len == 0:
      USBFSD->UEP5_RX_CTRL = ACK                // ZLP 直接 ACK，无需处理
      return
  → midi_rx_pending_len_ = len                  // 记录数据长度
  → USBFSD->UEP5_RX_CTRL = NAK                  // NAK，数据留在 DMA 缓冲区等待主线程

App_Loop() 主循环
  → MidiCC_ProcessRx()
    → if midi_rx_pending_len_ == 0:
        return
    → 从 midi_ep5_rx_buf_ 中以 4 字节为单位解析 Event Packet
    → for offset = 0; offset + 3 < midi_rx_pending_len_; offset += 4:
      → 解析 CIN/Status/Data1/Data2
      → if (status & 0xF0) == 0xB0:   // CC 消息
          → 查 CC 映射表 → 定位电位器编号
          → 反向映射: target_pos = cc_value * 4095 / 127
          → fader_rx_active[idx] = true
          → Motor_SetTarget(idx, target_pos)
      → 非 CC 消息静默丢弃
    → midi_rx_pending_len_ = 0
    → USBFSD->UEP5_RX_CTRL = ACK                // 全部消耗，重新武装 EP5
```

**原理说明：**

- 中断中**不操作 FIFO**，仅记录长度 + NAK，最小化中断耗时
- 主机收到 NAK 后自动重试同一 OUT 事务，硬件将相同数据重新写入 DMA 缓冲区，数据安全
- ZLP（零长度包）无需处理，直接 ACK
- 主循环直接解析 USB DMA 缓冲区，全部消耗完后 ACK
- NAK 期间后续 OUT 中断因 `midi_rx_pending_len_ > 0` 而直接返回

### 5.2 缓冲区

```c
__attribute__((aligned(4))) static uint8_t midi_ep5_rx_buf_[64];
static volatile uint8_t midi_rx_pending_len_;    // 0 = 无待处理数据
```

---

## 6. 防回环

### 6.1 原理

当 MCU 正在响应 RX 命令驱动某路电位器时，该路电位器的位置变化不得触发 TX 发送。

### 6.2 实现

```c
static bool fader_rx_active_[kMotorIdx_Count];

// RX 驱动时置位
void MidiCC_SetRxActive(uint8_t idx, bool active) {
    if (idx < kMotorIdx_Count)
        fader_rx_active_[idx] = active;
}

// TX 检测时跳过被标记通道
// (在 MidiCC_CheckAndSend 中实现)

// 清除时机：
// 1. Motor_RunControlLoop 中 active_ 从 true → false 时清除
// 2. 超时 1000ms (CTRL_TIMEOUT_MS) 强制清除
```

### 6.3 清除逻辑

在 `Motor_RunControlLoop()` 中，每路电机到达目标或超时停止后（`active_` 从 `true` → `false`），调用 `MidiCC_SetRxActive(i, false)`。

若电机在 `CTRL_TIMEOUT_MS` (1000ms) 内未到达目标，控制超时机制会停止电机并清除 `active_`，防回环标志随之清除。

---

## 7. CC 映射配置

### 7.1 默认映射表

| 电位器编号 | MIDI 通道 (1-based) | CC 编号 |
|:---:|:---:|:---:|
| 0 | 1 | 0 |
| 1 | 1 | 1 |
| 2 | 1 | 2 |
| 3 | 1 | 3 |
| 4 | 1 | 4 |
| 5 | 1 | 5 |
| 6 | 1 | 6 |
| 7 | 1 | 7 |

### 7.2 代码定义

```c
struct MidiCcMapping {
    uint8_t channel;  // 0-based (0=Channel 1)
    uint8_t cc_num;   // CC 编号 (0~127)
};

static const struct MidiCcMapping kMidiCcMap[kMotorIdx_Count] = {
    {0, 0},   // 电位器0 → Channel 1, CC 0
    {0, 1},   // 电位器1 → Channel 1, CC 1
    {0, 2},   // 电位器2 → Channel 1, CC 2
    {0, 3},   // 电位器3 → Channel 1, CC 3
    {0, 4},   // 电位器4 → Channel 1, CC 4
    {0, 5},   // 电位器5 → Channel 1, CC 5
    {0, 6},   // 电位器6 → Channel 1, CC 6
    {0, 7},   // 电位器7 → Channel 1, CC 7
};
```

---

## 8. 文件变更清单

### 8.1 新增文件

| 文件 | 内容 |
|------|------|
| `User/usb/midi_cc.h` | MIDI CC 模块公共 API 声明 + 常量 + 映射表 |
| `User/usb/midi_cc.c` | MIDI CC 模块实现：TX/RX FIFO 管理、包解析、发送逻辑 |

### 8.2 修改文件

| 文件 | 改动 |
|------|------|
| `User/usb/usb_desc.cpp` | bcdUSB 0x0110→0x0200；追加 IAD + AC + MIDI Streaming 描述符 |
| `User/usb/usb_impl.h` | 新增 MIDI 端点地址常量 (`kMidiEpAddr_In=0x84`, `kMidiEpAddr_Out=0x05`) |
| `User/usb/usb_impl.c` | `UsbImpl_InitAndOpenEndpoints()` 配置 EP4/EP5；`UsbImpl_EpInComplete` 处理 EP4；`UsbImpl_EpOutComplete` 处理 EP5 |
| `User/bsp/ch32v20x_it.c` | `SysTick_Handler()` 追加 `MidiCC_Flush()` 调用 |
| `User/app/app.c` | `App_Init()` 调用 `MidiCC_Init()`；主循环中调用 `MidiCC_ProcessRx()` |
| `User/app/motor.c` | `Motor_RunControlLoop()` 中 ADC 更新后调用 `MidiCC_CheckAndSend()`；电机停止时清除防回环标志 |
| `User/config.h` | 可选：新增 `MIDI_TX_FIFO_SIZE` / `MIDI_RX_FIFO_SIZE` 配置宏 |

### 8.3 不变文件

| 文件 | 说明 |
|------|------|
| `User/usb/usb_device.c/h` | USB 设备核心状态机，无改动 |
| `User/usb/usb_endpoint.h` | 端点类型枚举，无改动 |
| `User/usb/usb_hardware.h` | USB 寄存器别名，无改动 |
| `User/bsp/motor_hw.c/h` | 硬件 PWM/ADC 层，无改动 |
| `User/app/pid.c/h` | PID 控制器，无改动 |

---

## 9. 非功能需求

| 编号 | 描述 | 实现保证 |
|------|------|---------|
| NF1 | 电位器变化到 MIDI CC 发出，端到端延迟 < 10 ms | 1kHz 控制循环中入队，SysTick 1ms 周期发送，最大延迟 < 2ms |
| NF2 | MIDI 接口不影响 HID 接口的轮询周期与响应时间 | MIDI Bulk 传输使用独立端点，中断分离处理 |
| NF3 | Windows 下无需额外驱动 | Class-compliant USB MIDI 1.0，AC + MS 接口标准描述符 |
| NF4 | 不引入动态内存分配 | 所有缓冲区 `static` 静态分配 |

---

## 10. 验收标准

1. **枚举验证**：设备插入后设备管理器同时出现 HID 设备与 "USB 音频设备" 条目
2. **TX 验证**：推动电位器，MIDI-OX 收到对应 CC 消息，值随位置线性变化
3. **RX 验证**：通过 MIDI 工具发送 CC，对应电位器自动移动到目标位置
4. **防回环验证**：RX 驱动期间，MIDI 监视器不出现该路 TX CC 消息
5. **HID 兼容性验证**：MIDI 功能开启后，原有 HID 读写功能行为不变
