#!/usr/bin/env python3
"""
HID Debug Monitor
监听 HID Debug 接口的 printf 输出，插拔自动重连。

依赖: pip install hidapi
用法: python hid_debug_monitor.py
"""

import time
import sys

try:
    import hid
except ImportError:
    print("请先安装 hidapi: pip install hidapi")
    print("  pip install hidapi")
    sys.exit(1)

# ----------------------------------------
# 配置
# ----------------------------------------
VID = 0x1A86
PID = 0x0002
USAGE_PAGE = 0xFF01  # 和固件中 HID Debug 的 Usage Page 一致
REPORT_SIZE = 64


class HidDebugMonitor:
    def __init__(self):
        self.device = None
        self.running = False

    def find_device_path(self):
        """查找匹配的 HID 设备，返回设备路径"""
        devices = hid.enumerate(VID, PID)
        if not devices:
            return None

        for d in devices:
            iface = d.get("interface_number", -1)
            usage = d.get("usage_page", 0)
            path = d["path"]
            print(f"[调试] HID 设备: IF={iface}, UsagePage=0x{usage:04X}, path={path}")

        # 先按 Usage Page 精确匹配
        for d in devices:
            if d.get("usage_page") == USAGE_PAGE:
                print(f"[调试] => 选取 UsagePage=0x{USAGE_PAGE:04X} (IF={d.get('interface_number', -1)})")
                return d["path"]

        print("[调试] 未找到 Usage Page 匹配，使用第一个 HID 设备")
        return devices[0]["path"]

    def connect(self):
        """连接设备"""
        if self.device is not None:
            return True

        path = self.find_device_path()
        if path is None:
            return False

        try:
            dev = hid.device()
            dev.open_path(path)
            self.device = dev
            print(f"[连接] HID Debug 设备已连接")
            return True
        except Exception as e:
            print(f"[错误] 连接失败: {e}")
            self.device = None
            return False

    def disconnect(self):
        """断开设备"""
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
            self.device = None
            print("[断开] HID Debug 设备已断开")

    def handle_report(self, report: bytes):
        """
        处理 HID 报告。
        报告格式: [count(1B) | data(count B) | padding(0)]
        """
        if len(report) < 1:
            return

        count = report[0]
        if count == 0 or count > len(report) - 1:
            # 非法 count，丢弃整个包（已在固件修复边界）
            return

        data = report[1 : 1 + count]
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()

    def run(self):
        """主循环"""
        self.running = True
        print(f"[启动] HID Debug Monitor (VID=0x{VID:04X}, PID=0x{PID:04X})")
        print("[启动] 等待设备连接...")
        print("-" * 50)

        while self.running:
            if self.device is None:
                if self.connect():
                    print("[就绪] 开始接收数据")
                    print("----------------------------------------")
                    continue
                time.sleep(0.5)
                continue

            try:
                report = self.device.read(REPORT_SIZE, timeout_ms=100)
                if report:
                    self.handle_report(bytes(report))
            except (OSError, IOError) as e:
                print(f"[断开] 设备断开: {e}")
                self.disconnect()
                print("[等待] 设备重连...")
                continue
            except Exception as e:
                print(f"[错误] {e}")
                time.sleep(0.5)
                continue

        self.disconnect()

    def stop(self):
        self.running = False


def main():
    monitor = HidDebugMonitor()

    # 运行直到 Ctrl+C
    try:
        monitor.run()
    except KeyboardInterrupt:
        print("\n[退出] 用户中断")
        monitor.stop()


if __name__ == "__main__":
    main()
