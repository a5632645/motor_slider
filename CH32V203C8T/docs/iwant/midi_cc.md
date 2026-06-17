# 需求文档：八路马达线性电位器 MCU 固件 — USB MIDI CC 双向联动

## 背景与范围

现有固件已通过 USB HID 实现八路马达线性电位器的位置读取与设置。本需求在**不改动现有 HID 功能**的前提下，扩展为 USB 复合设备，新增 USB MIDI 类接口，实现电位器位置与 MIDI CC 消息的双向联动。

- **MCU**：CH32V 系列
- **USB 拓扑**：单一 USB 复合设备（Composite Device），同时枚举 HID 接口 + MIDI 接口
- **USB MIDI 实现**：使用tpusb生成 USB 描述符

---

## 1. USB 描述符扩展

### 1.1 复合设备结构

在现有 HID 接口描述符之后，追加以下接口：

```
Configuration Descriptor
├── Interface 0：现有 HID 接口（保持不变）
├── Interface 1：Audio Control（USB MIDI 规范要求的空 AC 接口）
└── Interface 2：MIDIStreaming（实际收发 MIDI 数据的接口）
    ├── Endpoint IN  (Bulk) — MCU → Host，发送 MIDI 事件包
    └── Endpoint OUT (Bulk) — Host → MCU，接收 MIDI 事件包
```

### 1.2 描述符要求

| 字段 | 值 |
|---|---|
| bDeviceClass | 0x00（由接口决定） |
| 设备级 IAD | 为 Audio/MIDI 接口对添加 Interface Association Descriptor |
| MIDI IN Jack | 1 个 Embedded Jack（jackId = 1） |
| MIDI OUT Jack | 1 个 Embedded Jack（jackId = 1），关联到 Bulk IN 端点 |
| Bulk 端点包大小 | 64 字节 |

### 1.3 兼容性约束

- `bcdUSB` 设为 `0x0200`（USB 2.0）
- 端点地址不得与现有 HID 端点冲突，编译期静态分配
- Windows 下无需额外驱动（Class-compliant USB MIDI 1.0）

---

## 2. MIDI 数据格式

所有 MIDI 数据通过 **USB MIDI Event Packet**（4 字节/包）传输，格式如下：

```
Byte 0: Cable Number (高4位) | Code Index Number (低4位)
Byte 1: MIDI Status Byte  (e.g. 0xB0 | channel)
Byte 2: MIDI Data Byte 1  (CC 编号)
Byte 3: MIDI Data Byte 2  (CC 值)
```

本项目仅处理 **Control Change（CC）** 消息，CIN = `0x0B`。

---

## 3. TX 方向：电位器 → MIDI CC 发送

### 3.1 触发条件

- 在现有位置轮询循环中，检测每路电位器的位置变化。
- 仅当某路的映射位置值（CC 值）与上次发送值**不同**时，才构造并发送 CC 包。

### 3.2 数值映射

```c
// 电位器原始值已归一化为 [0, 4095]（12-bit ADC）或 [0, 255]，以实际为准
// 映射到 MIDI CC [0, 127]
uint8_t cc_value = (uint8_t)(fader_raw * 127 / FADER_MAX);
```

### 3.3 发送行为

- 每路电位器各维护一个 `last_sent_cc[8]` 数组，用于去抖/去重。
- 位置变化超过最小阈值（建议 1 个 CC 步进单位）才触发发送，避免抖动造成消息洪泛。
- 多路同时变化时，在同一 USB 传输批次中依次打包发送。

---

## 4. RX 方向：MIDI CC → 电位器驱动

### 4.1 接收处理

- 在 USB OUT 端点中断或轮询回调中解析收到的 MIDI Event Packet。
- 仅处理 CC 消息（`Status & 0xF0 == 0xB0`），其余消息类型静默忽略。

### 4.2 路由与映射

- 根据收到的 `(MIDI 通道, CC 编号)` 查找映射表，定位目标电位器编号。
- 将 CC 值 `[0, 127]` 反向映射为电位器目标位置值：

```c
uint16_t target_pos = (uint16_t)(cc_value * FADER_MAX / 127);
```

- 调用**现有**的电位器位置设置接口驱动对应马达。

### 4.3 未匹配消息

- 未在映射表中命中的 CC 消息直接丢弃，不产生任何副作用。

---

## 5. 防回环

当 MCU 正在响应 RX 命令驱动某路电位器时，该路电位器的位置变化**不得**触发 TX 发送，避免 MIDI 消息回环。

### 实现方式

```c
// 每路维护一个标志位
volatile bool fader_rx_active[8] = {false};

// RX 驱动开始时置位
fader_rx_active[i] = true;
set_fader_position(i, target_pos);

// TX 检测时跳过被标记的通道
if (!fader_rx_active[i] && cc_value != last_sent_cc[i]) {
    send_midi_cc(i, cc_value);
}

// 电位器到达目标（或超时后）清除标志
fader_rx_active[i] = false;
```

超时保护建议：若马达在 **500 ms** 内未到达目标位置，强制清除标志，防止通道永久静默。

---

## 6. CC 映射配置

默认映射表（硬编码为 `const` 数组，可后续扩展为可配置）：

| 电位器编号 | MIDI 通道 | CC 编号 |
|:---:|:---:|:---:|
| 0 | 1 | 0 |
| 1 | 1 | 1 |
| 2 | 1 | 2 |
| 3 | 1 | 3 |
| 4 | 1 | 4 |
| 5 | 1 | 5 |
| 6 | 1 | 6 |
| 7 | 1 | 7 |

---

## 7. 现有功能约束（不得改动）

| 约束 | 说明 |
|---|---|
| HID 描述符 | 不得修改现有 HID Report Descriptor 及端点配置 |
| HID 轮询行为 | 现有 HID 位置上报逻辑保持不变 |
| 位置读取接口 | `read_fader_position(uint8_t index)` 保持原签名 |
| 位置设置接口 | `set_fader_position(uint8_t index, uint16_t pos)` 保持原签名 |

---

## 8. 非功能需求

| 编号 | 描述 |
|---|---|
| NF1 | 电位器变化到 MIDI CC 发出，端到端延迟 < 10 ms |
| NF2 | MIDI 接口不得影响 HID 接口的轮询周期与响应时间 |
| NF3 | USB 枚举成功后，HID 与 MIDI 接口均应在 Windows 设备管理器中正常识别，无需安装驱动 |
| NF4 | 固件中不引入动态内存分配（`malloc`/`free`），所有缓冲区静态分配 |

---

## 9. 验收标准

1. **枚举验证**：设备插入 Windows PC 后，设备管理器同时出现 HID 设备与"USB 音频设备 / MIDI"条目。
2. **TX 验证**：手动推动任意电位器，MIDI 监视器（如 MIDI-OX）收到对应 CC 消息，值随位置线性变化。
3. **RX 验证**：通过 MIDI 工具向设备发送 CC 消息，对应电位器自动移动到目标位置。
4. **防回环验证**：RX 驱动电位器移动期间，MIDI 监视器中不出现该路的 TX CC 消息。
5. **HID 兼容性验证**：MIDI 功能开启后，原有 HID 读写功能行为不变。
