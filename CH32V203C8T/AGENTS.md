# CH32V203C8T 固件项目 — AI 代理指南

## 快速开始

### 构建

```bash
cmake -B build -G "Ninja Multi-Config"
cmake --build build
```

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
│   ├── main.c               # 主程序入口 + 控制状态机
│   ├── motor.c/h            # 电机控制 (PWM/ADC/DMA/PID/闭环)
│   ├── pid.c/h              # PID 控制器
│   ├── config.h             # 可配置参数
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
│       ├── usb_endpoint.h   # 端点抽象 + 端点类型枚举
│       ├── usb_setup_request.h  # Setup 请求解析
│       └── tpusb/           # C++ constexpr 描述符生成库
├── docs/
│   └── superpowers/
│       ├── plans/           # 实现计划
│       └── specs/           # 设计文档
├── hid_debug_monitor.py     # HID0 printf CLI 监视器
├── hid_adc_monitor.py       # PyQt6 GUI (ADC + 调试控制台 + PID)
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

## 相关文档

- [硬件设计](project_promt.md)
- [电机控制实现计划](docs/superpowers/plans/2025-06-12-motor-control.md)
- [控制状态机设计](docs/superpowers/plans/2025-06-12-control-state-machine.md)
- [闭环控制+HID1协议](docs/superpowers/specs/2025-06-12-closed-loop-control.md)
- [电位器UI重构](docs/superpowers/specs/2025-06-12-potentiometer-ui-rework.md)
- [WCH CH32V203 数据手册](http://www.wch.cn/products/CH32V203.html)
