# CH32V203C8T 固件项目 — AI 代理指南

## 项目概览

CH32V203C8T (RISC-V RV32IMACXW) 微控制器固件项目，基于 MounRiver Studio 工具链和 CMake 构建系统。实现 USB HID 设备，通过 64 字节中断 IN 端点传输调试数据。

## 快速开始

### 构建

```bash
# 配置 (Windows + GNU Make)
cmake -B build -G "Unix Makefiles"

# 编译
cmake --build build

# 或在 MounRiver Studio IDE 中直接打开 .wvproj 文件
```

> **注意**: 工具链路径硬编码在 `CMakeLists.txt` 中（第 31 行），指向 `c:/MounRiver/MounRiver_Studio2/.../RISC-V Embedded GCC12`。如需更换路径，修改该文件中的 `TOOLCHAIN_FOLDER`。

### 调试输出监控

固件通过 USB HID 调试接口（非 UART）输出 `printf` 数据：

```bash
pip install hidapi
python hid_debug_monitor.py
```

监控工具会自动匹配 HID Usage Page `0xFF01` 的设备并实时显示 printf 输出。

### 硬件信息

- **MCU**: CH32V203C8T (RISC-V RV32IMACXW)
- **Flash**: 64 KB
- **RAM**: 20 KB
- **USB**: Full-Speed Device (USBFS 外设)
- **调试接口**: USB HID (Vendor Defined 0xFF01)

## 项目结构

```
├── AGENTS.md                # 本文件
├── CMakeLists.txt           # CMake 构建配置 + 工具链路径
├── CH32V203C8T.wvproj       # MounRiver Studio 项目文件
├── hid_debug_monitor.py     # Python HID 调试监控
├── Ld/Link.ld               # 链接脚本 (Flash 64K, RAM 20K)
├── Startup/
│   ├── startup_ch32v20x_D6.S   # 启动文件 (CH32V203C8T 对应 D6)
│   ├── startup_ch32v20x_D8.S   # D8 系列启动文件 (未使用)
│   └── startup_ch32v20x_D8W.S  # D8W 系列启动文件 (未使用)
├── Core/                    # RISC-V 内核抽象层
├── Debug/                   # 调试/延迟工具 (printf, Delay_Us)
├── Peripheral/inc & src/    # 外设驱动库 (ADC, GPIO, TIM, USART 等)
├── User/
│   ├── main.c               # 主程序入口
│   ├── usbd.c / usbd.h      # USB 设备初始化和连接管理
│   ├── tick.c / tick.h      # SysTick 1ms 定时器
│   ├── kfifo.h              # 无锁环形缓冲区 (用于 HID 数据流)
│   ├── ch32v20x_conf.h      # 外设库头文件包含
│   ├── ch32v20x_it.c/.h     # 中断处理 (SysTick, HardFault 等)
│   ├── system_ch32v20x.c/.h # 系统时钟配置
│   └── usb/
│       ├── usb_impl.c/.h    # USB HID 实现 (IN 端点、FIFO 发送)
│       ├── usb_device.c/.h  # USB 设备核心状态机
│       ├── usb_desc.cpp/.h  # USB 描述符 (设备/配置/HID/字符串)
│       ├── usb_endpoint.h   # 端点抽象
│       ├── usb_hardware.h   # 硬件寄存器便捷别名
│       └── usb_setup_request.h  # Setup 请求解析
```

## 关键架构决策

### USB HID 调试通道

- **用途**: 替代 UART，通过 USB 直接传输 `printf` 数据（无需额外串口线）
- **端点**: EP1 IN (中断传输, 0x81)
- **报告大小**: 64 字节，字节 0 为有效数据长度，字节 1-63 为负载
- **FIFO 缓冲**: 基于 `kfifo.h` 的 512 字节无锁环形缓冲区，在 `usb_impl.c` 中管理
- **刷新机制**: `HID_Flush()` 在数据可用且总线空闲时触发发送

### 内核模块

| 模块 | 文件 | 职责 |
|------|------|------|
| SysTick 定时器 | `tick.c` | 1ms 周期中断，`Tick_Get()` 返回毫秒计数 |
| USB 设备 | `usb_device.c` | 协议状态机 (SETUP/IN/OUT, 标准请求处理) |
| USB 实现 | `usb_impl.c` | 类请求 + 厂商请求处理，HID 报告发送 |
| Kfifo | `kfifo.h` | 无锁环形缓冲区，支持连续读取和写入 |

### 编码约定

- 语言: C (gnu99) + USB 描述符部分使用 C++11 (usb_desc.cpp)
- USB 枚举前缀: `kUsb` (如 `kUsbEndpoint_HidIn`, `kUsbStandardRequest_GetDescriptor`)
- 函数命名: `模块名_动作` (如 `HID_Write`, `UsbImpl_InitAndOpenEndpoints`)
- 使用 `__attribute__((interrupt("WCH-Interrupt-fast")))` 声明中断函数
- `printf` 通过 HID 调试接口输出（非标准 UART）

### 命名规则
- **禁止使用 `extern` 全局变量** — 使用头文件中声明的函数访问
- **全局变量**以 `_` 结尾（如 `curr_`, `mute_all_`, `dirty_mask_`）
- **源文件中的 `static` 函数**以 `_` 开头，头文件中的 `static` 函数不需要
- **头文件函数声明**格式：`模块_作用`（如 `DspDatabase2_Init`, `DspSet2_Set`）
- **枚举值**前缀 `k` + 模块名（如 `kDspModule_System`, `kFilterType_Peak`）
- **注释和todo标记** 未经允许不能自主删除注释和todo标记，todo标记使用 #warning todo

### 头文件
- 必须使用 [Doxygen](https://www.doxygen.nl/) 注释声明函数

## 常见任务

### 添加新功能

1. 在 `User/usb/usb_impl.c` 处理类/厂商请求
2. 在 `User/usb/usb_desc.cpp` 更新描述符（如需修改 VID/PID/报告描述符）
3. 外设初始化参考 `Peripheral/src/` 中的示例

### 修改 USB 描述符

编辑 `User/usb/usb_desc.cpp`。已定义的描述符：
- 设备描述符: VID=0x1A86 (WCH), PID=0x0002
- HID 报告描述符: Usage Page 0xFF01, 64 字节 Input
- 配置描述符: 单接口, 1 个中断 IN 端点

### 运行 Python 监控

```bash
python hid_debug_monitor.py
```

支持设备热插拔自动重连。

## 潜在陷阱

1. **工具链路径**: 硬编码在 `CMakeLists.txt`，在其他机器上需修改 `TOOLCHAIN_FOLDER`
2. **路径限制**: 工程路径不能包含中文或空格（MRS 工具链限制）
3. **Windows 构建**: 需要安装 GNU MAKE 或 MinGW，并将 bin 目录加入 PATH
4. **USB 枚举失败**: 检查 `UsbDesc_Config` 中的 wTotalLength 是否与实际描述符总长度一致
5. **HID 数据丢失**: FIFO 大小为 512 字节，数据生产速度超过 USB 发送速率时会溢出

## 相关文档

- [WCH CH32V203 数据手册](http://www.wch.cn/products/CH32V203.html)
- MounRiver Studio 文档位于安装目录下
