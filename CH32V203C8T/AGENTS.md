# CH32V203C8T 固件项目 — AI 代理指南

## 项目概览

CH32V203C8T (RISC-V RV32IMACXW) 微控制器固件项目。
- **USB HID 调试输出** — 替代 UART 的 printf 通道
- **8 路电机闭环位置控制** — PWM + ADC + PID

硬件：8 路电动线性电位器测试板，DRV8833 驱动，4 个定时器 14 路 PWM，8 路 ADC 位置反馈。
详细硬件设计见 [`project_promt.md`](project_promt.md)。

## 快速开始

### 构建

```bash
cmake -B build -G "Ninja Multi-Config"
cmake --build build
```

工具链路径在 `CMakeLists.txt` 的 `TOOLCHAIN_FOLDER`。

### HID 调试输出监控

```bash
pip install hidapi
python hid_debug_monitor.py
```

匹配 HID Usage Page `0xFF01` 的设备。

### 硬件信息

- **MCU**: CH32V203C8T (D6 系列), RV32IMAC, 144MHz
- **Flash/RAM**: 64 KB / 20 KB
- **USB**: USBFS (Full Speed 12Mbps), 非 USBHS
- **系统时钟**: 96MHz (HSE=8MHz × PLL12)

## 项目结构

```
├── AGENTS.md                # 本文件
├── project_promt.md         # 硬件设计需求文档
├── CMakeLists.txt           # CMake 构建配置
├── Core/                    # RISC-V 核心层
├── Debug/                   # printf 后端 + Delay_Us
├── Peripheral/              # CH32V20x 标准外设库
├── Startup/                 # 启动文件 (D6/D8/D8W)
├── User/
│   ├── main.c               # 主程序入口
│   ├── motor.c/h            # 电机控制 (PWM/ADC/DMA/PID)
│   ├── pid.c/h              # PID 控制器
│   ├── tick.c/h             # SysTick 1ms 定时器
│   ├── kfifo.h              # 字节型无锁环形缓冲区
│   ├── usbd.c/h             # USB 设备初始化/连接
│   ├── ch32v20x_it.c/h      # 中断处理
│   ├── system_ch32v20x.c/h  # 系统时钟 (96MHz)
│   └── usb/
│       ├── usb_desc.cpp/h   # USB 描述符 (tpusb 模板生成)
│       ├── usb_device.c/h   # USB 设备状态机 + EP0
│       ├── usb_impl.c/h     # HID0 printf + HID1 电机控制
│       ├── usb_hardware.h   # USBFS 寄存器别名
│       ├── usb_endpoint.h   # 端点抽象 + 端点号枚举
│       ├── usb_setup_request.h  # Setup 请求解析
│       └── tpusb/           # C++ constexpr 描述符生成库
```

## 系统架构

### 控制流水线 (1kHz)

```
SysTick (1ms)
  └→ Motor_StartAdcConversion()  [软件触发 ADC]
      └→ ADC 常规组 scan 8 通道 + DMA1 CH1 自动搬运
          └→ DMA 完成中断 → motor_adc_ready_ = true
              └→ main 循环: Motor_RunControlLoop()
                  └→ PID → Motor_SetPwm() → TIM 比较寄存器
```

### USB 双 HID 接口

| 接口 | 端点 | 用途 |
|------|------|------|
| HID0 | EP1 IN (0x81) | printf 调试输出 |
| HID1 | EP2 IN (0x82) + EP2 OUT (0x02) | 电机控制命令/状态 |

### 主要数据流

```
printf() → _write() → HID_Write() → kfifo[512] → SysTick → HID_Flush() → EP1 IN
上位机命令 → HID1 OUT → EP2 OUT → HID1_ProcessCommand() → Motor_SetTarget()
```

## 编码约定

### 命名规则
- **全局变量** 以 `_` 结尾（如 `motor_adc_dma_buf_`, `tick_`）
- **源文件 `static` 函数** 以 `_` 开头（如 `_HidInHandler`）
- **头文件函数声明** `模块_动作`（如 `Motor_InitPwm`, `HID_Write`）
- **枚举值** 前缀 `k` + 模块名（如 `kUsbEndpoint_HidIn`, `kMotorDir_Forward`）
- **禁止 `extern` 全局变量** — 用头文件声明的函数访问
- **禁止对结构体/枚举使用typedef** — 显式 `struct 结构体` / `enum 枚举名`
- **中断函数** `__attribute__((interrupt("WCH-Interrupt-fast")))`
- **注释/todo** 不能自主删除，todo 用 `#warning todo`
- **头文件** 必须使用 Doxygen 注释

### 语言
- C (gnu99)，`usb_desc.cpp` 用 C++11（tpusb 模板）

### printf
- 通过 HID 调试接口输出，非 UART

## 电机引脚映射

| 电机 | ADC | A 相 (IN1) | B 相 (IN2) |
|------|-----|-----------|-----------|
| 1 | PA0 | TIM3 CH3 PB0 | TIM3 CH4 PB1 |
| 2 | PA1 | TIM2 CH4 PB11 | TIM2 CH3 PB10 |
| 3 | PA2 | GPIO PB14 | GPIO PB15 |
| 4 | PA3 | TIM1 CH2 PA9 | TIM1 CH1 PA8 |
| 5 | PA4 | TIM1 CH3 PA10 | TIM1 CH4 PA11 |
| 6 | PA5 | TIM2 CH1 PA15 | TIM2 CH2 PB3 |
| 7 | PA6 | TIM3 CH1 PB4 | TIM3 CH2 PB5 |
| 8 | PA7 | TIM4 CH3 PB8 | TIM4 CH4 PB9 |

注意：TIM2 使用 `GPIO_FullRemap_TIM2`，TIM4 CH1/CH2 未使用。

### DRV8833 控制
- 每路 IN1/IN2 独立 PWM：正转 IN1=PWM,IN2=0；反转 IN1=0,IN2=PWM；刹车 IN1=1,IN2=1
- nSLEEP 固定高电平

## USBFS 寄存器差异（vs USBHS）

| USBHS (旧) | USBFS (当前) |
|------------|-------------|
| `USBHSD` | `USBFSD` |
| `ENDP_CONFIG` | `UEP4_1_MOD` / `UEP2_3_MOD` |
| `ENDP_TYPE` | 不存在 |
| `HOST_CTRL` | 不存在 |
| `DEV_AD` | `DEV_ADDR` |
| `UIE_SETUP_ACT` | 无，SETUP 合入 `UIF_TRANSFER` |
| `USBHS_IRQHandler` | `USBFS_IRQHandler` |

## 潜在陷阱

1. **工具链路径** 硬编码在 CMakeLists.txt，换机器需改 `TOOLCHAIN_FOLDER`
2. **路径不能含中文/空格** — MRS 工具链限制
3. **USB 时钟必须 48MHz** — 当前 96MHz→Div2，如改系统频率需同步调整
4. **SysTick 配置** 使用 bit5=STCLK、bit1=TICKINT、bit0=ENABLE，不是 bit2
5. **PA11 可能被 USB DM 占用** — 按硬件设计确认是否可用 TIM1_CH4
6. **ADC DMA 需 NVIC 使能** — `DMA1_Channel1_IRQn` 需配置优先级

## 相关文档

- [硬件设计](project_promt.md)
- [电机控制实现计划](docs/superpowers/plans/2025-06-12-motor-control.md)
- [WCH CH32V203 数据手册](http://www.wch.cn/products/CH32V203.html)
