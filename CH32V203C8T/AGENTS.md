# CH32V203C8T 固件项目 — AI 代理指南

## 快速开始

### 构建

```bash
cmake -B build -G "Ninja Multi-Config"
cmake --build build
```

> cmake --build build 可编译所有源文件（.c/.cpp → .o），但最终链接阶段会报错 `'CH32V203C8T.elf': No such file` — 这是正常的，因为当前环境缺少 MounRiver 工具链的完整链接器支持。编译 .o 文件成功即表示代码语法正确。

## 项目结构

```
├── AGENTS.md                # 本文件
├── CMakeLists.txt           # CMake 构建配置
├── .clang-format            # C 代码格式化配置
├── .clangd                  # clangd LSP 配置
├── .cproject / .project     # Eclipse/CDT 项目文件
├── Core/                    # RISC-V 核心层
├── Debug/                   # printf 后端 + Delay_Us
├── Peripheral/              # CH32V20x 标准外设库
├── Startup/                 # 启动文件 (D6/D8/D8W)
├── User/
│   ├── main.c               # 主入口: Bsp_Init → App_Init → App_Loop
│   ├── config.h             # 可配置参数 (PID/控制/定时)
│   ├── app/                 # 应用层
│   │   ├── app.c/h          # 应用主循环 + 控制状态机
│   │   ├── motor.c/h        # 电机控制 (闭环/PWM/状态管理)
│   │   ├── pid.c/h          # PID 控制器
│   │   └── midi_cc.c/h      # USB MIDI CC 接收/处理
│   ├── bsp/                 # 板级支持包 (硬件抽象)
│   │   ├── bsp.c/h          # 板级初始化总入口
│   │   ├── motor_hw.c/h     # 电机硬件抽象 (PWM/ADC/DMA)
│   │   ├── tick.c/h         # SysTick 1ms 定时器
│   │   ├── usbd.c/h         # USB 设备初始化/连接
│   │   ├── ch32v20x_it.c/h  # 中断处理
│   │   ├── system_ch32v20x.c/h  # 系统时钟 (96MHz)
│   │   └── ch32v20x_conf.h  # 外设库头文件包含
│   ├── usb/                 # USB 协议栈
│   │   ├── usb_desc.cpp/h   # USB 描述符 (tpusb 模板生成)
│   │   ├── usb_device.c/h   # USB 设备状态机 + EP0
│   │   ├── usb_impl.c/h     # USB 设备回调 + HID0/HID1/MIDI 实现
│   │   ├── usb_hardware.h   # USBFS 寄存器别名
│   │   ├── usb_endpoint.h   # 端点抽象 + 端点类型枚举
│   │   ├── usb_setup_request.h  # Setup 请求解析
│   │   └── tpusb/           # C++ constexpr 描述符生成库
│   └── util/                # 工具
│       └── kfifo.h          # 字节型无锁环形缓冲区
├── docs/
│   ├── iwant/               # 原始需求文档
│   │   ├── project_promt.md # 硬件设计需求
│   │   └── midi_cc.md       # MIDI CC 需求
│   └── superpowers/
│       ├── plans/           # 实现计划
│       └── specs/           # 设计文档
├── hid_debug_monitor.py     # HID0 printf CLI 监视器
├── hid_adc_monitor.py       # PyQt6 GUI (ADC + 调试控制台 + PID + PWM Bias/Max)
└── [build/]                 # CMake 构建输出 (gitignored)
```

## 架构分层

```
main.c
  └─ Bsp_Init()                ── 初始化 HAL、时钟、USB 设备
  └─ App_Init()                ── 初始化应用模块 (电机、PID、MIDI)
  └─ App_Loop()                ── 永不返回的主循环
       ├─ _MotorControl()       ── 控制状态机 (Idle→AdcStart→AdcWait→Control)
       │    ├─ Motor_StartAdcConversion()  ── bsp/motor_hw.c
       │    ├─ Motor_RunControlLoop()      ── app/motor.c (PID + PWM)
       │    └─ Motor_OnFilterAdcReady()          ── 回调 → MidiCC_UpdateAdc
       ├─ Motor_SendStatus()   ── USB HID1 上报 8 路 ADC/目标/占空比
       ├─ Motor_ProcessCommand()  ── USB HID1 命令接收 (目标/PID/Bias/Max)
       ├─ MidiCC_Control()     ── MIDI 发送状态机 (滤波/量化/CC 发送)
       ├─ MidiCC_ProcessRx()   ── USB MIDI CC 接收 → ADC 目标映射
       ├─ UsbImpl_Midi_Poll()  ── MIDI TX FIFO → EP4 Bulk 发送
       └─ UsbImpl_HidDebug_Flush()  ── HID0 printf FIFO → EP1 发送
```

## 编码约定

- **全局变量** 以 `_` 结尾（如 `motor_adc_dma_buf_`, `tick_`）
- **源文件 `static` 函数** 以 `_` 开头（如 `_HidInHandler`, `_MotorControl`）
- **头文件函数声明** `模块_动作`（如 `Motor_InitPwm`, `HID_Write`, `Bsp_Init`）
- **枚举值** 前缀 `k` + 模块名（如 `kUsbEndpoint_HidIn`, `kMotorDir_Forward`, `kCtrlState_Idle`）
- **禁止 `extern` 全局变量** — 用头文件声明的函数访问
- **禁止对结构体/枚举使用typedef** — 显式 `struct 结构体` / `enum 枚举名`
- **中断函数** 需要标记`__attribute__((interrupt("WCH-Interrupt-fast")))`
- **注释/todo** 不能自主删除，todo 用 `#warning todo`
- **头文件** 尽量`使用Doxygen注释函数声明`和`#pragma once`
- **源文件** 逻辑复杂的私有函数尽量使用Doxygen注释
- **普通注释** 使用 `//`
