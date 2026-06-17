#!/usr/bin/env python3
"""
HID ADC Monitor — 8 路电机 ADC + 调试控制台 (PyQt6)

通过 HID1 接口接收 CH32V203C8T 固件上报的 64 字节二进制状态报告，
以列表式 GUI 显示 8 路 ADC 值、目标值、占空比和位置百分比。
同时通过 HID0 接口接收 printf 调试输出，显示在底部控制台。

依赖:
    pip install hidapi PyQt6

用法:
    python hid_adc_monitor.py
"""

import sys
import time
import struct
from dataclasses import dataclass, field
from typing import Optional, List

try:
    import hid
except ImportError:
    print("请先安装 hidapi: pip install hidapi")
    print("  pip install hidapi")
    sys.exit(1)

try:
    from PyQt6.QtCore import (
        Qt, QThread, pyqtSignal, QTimer, QMutex, QMutexLocker, QPointF
    )
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QStatusBar, QPlainTextEdit, QPushButton, QSplitter,
        QSlider, QSizePolicy, QComboBox, QDoubleSpinBox, QSpinBox,
    )
    from PyQt6.QtGui import (
        QColor, QPalette, QFont, QPainter, QPen,
    )
except ImportError:
    print("请先安装 PyQt6: pip install PyQt6")
    print("  pip install PyQt6")
    sys.exit(1)


# ---------------------------------------------------------------------------
# 常量定义 (与固件 usb_impl.h 一致)
# ---------------------------------------------------------------------------
VID = 0x1A86
PID = 0x0002
HID0_USAGE_PAGE = 0xFF01   # HID0 printf 调试输出接口
HID1_USAGE_PAGE = 0xFF00   # HID1 电机控制接口
REPORT_SIZE = 64
MOTOR_COUNT = 8

# 报告格式偏移 (固件 HID1_ADC(i) / HID1_TARGET(i) / HID1_DUTY(i))
OFFSET_STATUS = 0
OFFSET_RSV     = 1
OFFSET_ACTIVE_FLAGS = 1   # uint8, 每路 1 bit active
OFFSET_ADC     = 2       # uint16 LE × 8,  偏移 2～17

# PID 定点数缩放 (与固件一致)
PID_SCALE_KP = 1000
PID_SCALE_KI = 10000
PID_SCALE_KD = 1000
OFFSET_TARGET  = 18      # uint16 LE × 8,  偏移 18～33
OFFSET_DUTY    = 34      # uint16 LE × 8,  偏移 34～49


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------
@dataclass
class MotorChannelData:
    """单路电机状态"""
    adc: int = 0
    target: int = 0
    duty: int = 0
    connected: bool = False


@dataclass
class StatusReport:
    """解析后的 HID1 状态报告"""
    running: bool = False
    active_flags: int = 0
    channels: List[MotorChannelData] = field(default_factory=list)

    @classmethod
    def from_bytes(cls, report: bytes) -> Optional["StatusReport"]:
        """从 64 字节 HID 报告解析"""
        if len(report) < REPORT_SIZE:
            return None

        result = cls()
        result.running = bool(report[OFFSET_STATUS] & 0x01)
        result.active_flags = report[OFFSET_ACTIVE_FLAGS]

        for i in range(MOTOR_COUNT):
            offs_adc = OFFSET_ADC + i * 2
            offs_target = OFFSET_TARGET + i * 2
            offs_duty = OFFSET_DUTY + i * 2

            ch = MotorChannelData(
                adc    = struct.unpack_from("<H", report, offs_adc)[0],
                target = struct.unpack_from("<H", report, offs_target)[0],
                duty   = struct.unpack_from("<H", report, offs_duty)[0],
                connected=True,
            )
            result.channels.append(ch)

        return result


# ---------------------------------------------------------------------------
# HID Worker 线程
# ---------------------------------------------------------------------------
class HidWorker(QThread):
    """后台线程: 持续读取 HID1 报告并解析"""

    data_ready = pyqtSignal(object)   # StatusReport
    connected = pyqtSignal()
    disconnected = pyqtSignal(str)     # 原因
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._device: Optional[hid.device] = None
        self._mutex = QMutex()

    def send_target(self, channel: int, value: int):
        """通过 HID1 OUT 发送单路目标位置 (线程安全)"""
        report = bytearray(65)
        report[0] = 0x00   # 幻影 Report ID
        report[1] = 0x01   # 命令 ID: 设置目标
        report[2] = 1      # count = 1 (单路)
        report[3] = channel
        report[4] = value & 0xFF
        report[5] = (value >> 8) & 0xFF
        with QMutexLocker(self._mutex):
            if self._device is not None:
                try:
                    self._device.write(bytes(report))
                except Exception:
                    pass

    def send_pid(self, channel: int, kp: float, ki: float, kd: float):
        """通过 HID1 OUT 发送 PID 参数"""
        report = bytearray(65)
        report[0] = 0x00   # 幻影 Report ID
        report[1] = 0x04   # 命令 ID: 设置PID
        report[2] = channel
        def _u16(v):
            return int(v) & 0xFFFF
        kp_int = _u16(kp * PID_SCALE_KP)
        ki_int = _u16(ki * PID_SCALE_KI)
        kd_int = _u16(kd * PID_SCALE_KD)
        report[3] = kp_int & 0xFF
        report[4] = (kp_int >> 8) & 0xFF
        report[5] = ki_int & 0xFF
        report[6] = (ki_int >> 8) & 0xFF
        report[7] = kd_int & 0xFF
        report[8] = (kd_int >> 8) & 0xFF
        with QMutexLocker(self._mutex):
            if self._device is not None:
                try:
                    self._device.write(bytes(report))
                except Exception:
                    pass

    def send_pwm_bias(self, value: int):
        """通过 HID1 OUT 设置 PWM 起步偏置"""
        report = bytearray(65)
        report[0] = 0x00   # 幻影 Report ID
        report[1] = 0x05   # 命令 ID: 设置 PWM Bias
        report[2] = value & 0xFF
        report[3] = (value >> 8) & 0xFF
        with QMutexLocker(self._mutex):
            if self._device is not None:
                try:
                    self._device.write(bytes(report))
                except Exception:
                    pass

    def send_pwm_max(self, value: int):
        """通过 HID1 OUT 设置 PWM 最大占空比"""
        report = bytearray(65)
        report[0] = 0x00   # 幻影 Report ID
        report[1] = 0x06   # 命令 ID: 设置 PWM Max
        report[2] = value & 0xFF
        report[3] = (value >> 8) & 0xFF
        with QMutexLocker(self._mutex):
            if self._device is not None:
                try:
                    self._device.write(bytes(report))
                except Exception:
                    pass

    def run(self):
        self._running = True
        while self._running:
            if self._device is None:
                self._try_connect()
                if self._device is None:
                    # 未找到设备，等待后重试
                    if self._running:
                        time.sleep(0.5)
                    continue

            try:
                report = self._device.read(REPORT_SIZE, timeout_ms=200)
                if report:
                    parsed = StatusReport.from_bytes(bytes(report))
                    if parsed:
                        self.data_ready.emit(parsed)
            except (OSError, IOError) as e:
                self._on_disconnect(f"设备断开: {e}")
            except Exception as e:
                self.error_occurred.emit(f"HID 读取错误: {e}")
                time.sleep(0.5)

        self._close_device()

    def stop(self):
        """安全停止线程"""
        self._running = False
        # 不等待 — 线程循环中的 sleep/read 会因 timeout 自然退出

    def _try_connect(self):
        """枚举并打开 HID1 设备"""
        try:
            devices = hid.enumerate(VID, PID)
        except Exception as e:
            self.error_occurred.emit(f"枚举设备失败: {e}")
            return

        target_path = None
        for d in devices:
            if d.get("usage_page") == HID1_USAGE_PAGE:
                target_path = d["path"]
                break

        if target_path is None:
            return  # 无匹配设备

        try:
            dev = hid.device()
            dev.open_path(target_path)
            self._device = dev
            self.connected.emit()
        except Exception as e:
            self.error_occurred.emit(f"HID1 打开失败: {e}")
            self._device = None

    def _close_device(self):
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None

    def _on_disconnect(self, reason: str):
        self._close_device()
        self.disconnected.emit(reason)


# ---------------------------------------------------------------------------
# HID Debug Worker (HID0 printf)
# ---------------------------------------------------------------------------
class HidDebugWorker(QThread):
    """后台线程: 持续读取 HID0 printf 输出, 按行发射"""

    text_ready = pyqtSignal(str)       # 完整一行文本
    connected = pyqtSignal()
    disconnected = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._device: Optional[hid.device] = None
        self._line_buf = bytearray()    # 缓存不完整的行

    def run(self):
        self._running = True
        while self._running:
            if self._device is None:
                self._try_connect()
                if self._device is None:
                    if self._running:
                        time.sleep(0.5)
                    continue

            try:
                report = self._device.read(REPORT_SIZE, timeout_ms=200)
                if report:
                    self._feed(bytes(report))
            except (OSError, IOError) as e:
                self._on_disconnect(f"设备断开: {e}")
            except Exception as e:
                self.error_occurred.emit(f"HID0 读取错误: {e}")
                time.sleep(0.5)

        self._close_device()

    def stop(self):
        self._running = False

    def _try_connect(self):
        try:
            devices = hid.enumerate(VID, PID)
        except Exception as e:
            self.error_occurred.emit(f"枚举设备失败: {e}")
            return

        target_path = None
        for d in devices:
            if d.get("usage_page") == HID0_USAGE_PAGE:
                target_path = d["path"]
                break

        if target_path is None:
            return

        try:
            dev = hid.device()
            dev.open_path(target_path)
            self._device = dev
            self.connected.emit()
        except Exception as e:
            self.error_occurred.emit(f"HID0 打开失败: {e}")
            self._device = None

    def _close_device(self):
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None

    def _on_disconnect(self, reason: str):
        self._close_device()
        self.disconnected.emit(reason)

    def _feed(self, report: bytes):
        """处理一帧 HID0 报告: [count(1B) | text(count B) | padding]"""
        if len(report) < 1:
            return
        count = report[0]
        if count == 0 or count > len(report) - 1:
            return
        payload = report[1:1 + count]
        self._line_buf.extend(payload)

        # 分割出完整行 (\r\n 或 \n)
        while True:
            idx = self._line_buf.find(b'\n')
            if idx < 0:
                break
            line = self._line_buf[:idx]
            self._line_buf = self._line_buf[idx + 1:]

            # 去除尾部 \r (CRLF 情况)
            if line.endswith(b'\r'):
                line = line[:-1]

            if line:
                try:
                    text = line.decode('utf-8', errors='replace')
                except Exception:
                    text = repr(line)
                self.text_ready.emit(text)


# ---------------------------------------------------------------------------
# ADC 指示器 (自定义绘制: 竖线 + 横线标记)
# ---------------------------------------------------------------------------
class AdcIndicator(QWidget):
    """垂直竖线 + 横线标记，表示当前 ADC 值位置"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 0
        self.setMinimumWidth(16)

    def set_value(self, value: int):
        self._value = max(0, min(4095, value))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        w = self.width()
        h = self.height()

        # 竖线 (中心)
        pen = QPen(QColor("#999"))
        pen.setWidth(2)
        painter.setPen(pen)
        cx = w // 2
        margin = 4
        painter.drawLine(cx, margin, cx, h - margin)

        # 横线标记 (ADC 位置)
        pos_ratio = self._value / 4095.0
        y = h - margin - int(pos_ratio * (h - 2 * margin))
        y = max(margin, min(h - margin, y))

        marker_pen = QPen(QColor("#4CAF50"))
        marker_pen.setWidth(3)
        painter.setPen(marker_pen)
        painter.drawLine(cx - 7, y, cx + 7, y)

        painter.end()


# ---------------------------------------------------------------------------
# 电位器部件 (Slider + ADC 指示器 + PWM 数字)
# ---------------------------------------------------------------------------
class PotWidget(QWidget):
    """单个电位器控件: 可拖动 Slider (目标) + 竖线指示器 (ADC) + PWM 数字"""

    target_changed = pyqtSignal(int, int)  # (channel, value)

    def __init__(self, channel: int, parent=None):
        super().__init__(parent)
        self._channel = channel  # 0-based
        self._adc = 0
        self._target = 2048
        self._duty = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 2, 3, 2)
        layout.setSpacing(1)

        # CH 标签
        self._ch_label = QLabel(f"CH{channel + 1}")
        self._ch_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        font = QFont("Consolas", 9)
        self._ch_label.setFont(font)
        layout.addWidget(self._ch_label)

        # Slider + ADC 指示器
        indicator_area = QWidget()
        indicator_layout = QHBoxLayout(indicator_area)
        indicator_layout.setContentsMargins(0, 0, 0, 0)
        indicator_layout.setSpacing(0)

        self._slider = QSlider(Qt.Orientation.Vertical)
        self._slider.setRange(0, 4095)
        self._slider.setValue(2048)
        self._slider.setTickPosition(QSlider.TickPosition.NoTicks)
        self._slider.valueChanged.connect(self._on_slider_changed)
        indicator_layout.addWidget(self._slider)

        self._adc_indicator = AdcIndicator()
        indicator_layout.addWidget(self._adc_indicator)

        layout.addWidget(indicator_area, stretch=1)

        # 数值标签: ADC + PWM
        self._value_label = QLabel("ADC:0  PWM:0")
        self._value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._value_label.setFont(QFont("Consolas", 8))
        layout.addWidget(self._value_label)

    def update_data(self, adc: int, target: int, duty: int):
        self._adc = adc
        self._target = target
        self._duty = duty

        # 更新 ADC 指示器 (当前值)
        self._adc_indicator.set_value(adc)

        # 更新数字
        self._value_label.setText(f"ADC:{adc}  PWM:{duty}")

    def _on_slider_changed(self, value: int):
        """Slider 拖动 -> 更新本地目标 + 通知主窗口发送 HID1"""
        self._target = value
        self.target_changed.emit(self._channel, value)


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self._worker = HidWorker()
        self._debug_worker = HidDebugWorker()
        self._report_count = 0
        self._fps = 0.0
        self._last_fps_time = time.monotonic()
        self._latest_report: Optional[StatusReport] = None
        self._pots: List[PotWidget] = []
        self._current_targets = [2048] * 8

        self._init_ui()
        self._connect_signals()

        self._fps_timer = QTimer(self)
        self._fps_timer.setInterval(500)
        self._fps_timer.timeout.connect(self._update_fps)
        self._fps_timer.start()

        self._worker.start()
        self._debug_worker.start()

    def _init_ui(self):
        self.setWindowTitle("HID ADC Monitor — 电机滑块控制")
        self.setMinimumSize(1280, 720)
        self.resize(1280, 720)

        # 中央控件
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 信息栏
        info_bar = QHBoxLayout()
        self._status_label = QLabel("状态: 等待设备...")
        self._status_label.setStyleSheet("font-weight: bold;")
        self._fps_label = QLabel("0 fps")
        self._fps_label.setStyleSheet("color: #888;")
        info_bar.addWidget(self._status_label)
        info_bar.addStretch()
        info_bar.addWidget(self._fps_label)
        layout.addLayout(info_bar)

        # 垂直分割器: 上 = 电位器, 下 = 调试控制台
        splitter = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(splitter, stretch=1)

        # === 上: 8 路电位器 ===
        pots_container = QWidget()
        pots_layout = QVBoxLayout(pots_container)
        pots_layout.setContentsMargins(0, 0, 0, 0)
        pots_layout.setSpacing(0)

        pots_row = QWidget()
        row_layout = QHBoxLayout(pots_row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(2)

        for i in range(MOTOR_COUNT):
            pot = PotWidget(i)
            self._pots.append(pot)
            row_layout.addWidget(pot, 1)  # stretch=1 强制均分宽度

        pots_layout.addWidget(pots_row)
        splitter.addWidget(pots_container)

        # === 中: PID 控制面板 ===
        pid_container = QWidget()
        pid_layout = QHBoxLayout(pid_container)
        pid_layout.setContentsMargins(0, 0, 0, 0)
        pid_layout.setSpacing(6)

        pid_layout.addWidget(QLabel("PID:"))

        self._pid_ch = QComboBox()
        self._pid_ch.addItem("ALL", 0xFF)
        for i in range(8):
            self._pid_ch.addItem(f"CH{i+1}", i)
        self._pid_ch.setFixedWidth(70)
        pid_layout.addWidget(self._pid_ch)

        pid_layout.addWidget(QLabel("Kp"))
        self._pid_kp = QDoubleSpinBox()
        self._pid_kp.setRange(0.0, 100.0)
        self._pid_kp.setDecimals(3)
        self._pid_kp.setSingleStep(0.1)
        self._pid_kp.setValue(0.5)
        self._pid_kp.setFixedWidth(80)
        pid_layout.addWidget(self._pid_kp)

        pid_layout.addWidget(QLabel("Ki"))
        self._pid_ki = QDoubleSpinBox()
        self._pid_ki.setRange(0.0, 100.0)
        self._pid_ki.setDecimals(4)
        self._pid_ki.setSingleStep(0.01)
        self._pid_ki.setValue(0.01)
        self._pid_ki.setFixedWidth(90)
        pid_layout.addWidget(self._pid_ki)

        pid_layout.addWidget(QLabel("Kd"))
        self._pid_kd = QDoubleSpinBox()
        self._pid_kd.setRange(0.0, 100.0)
        self._pid_kd.setDecimals(3)
        self._pid_kd.setSingleStep(0.1)
        self._pid_kd.setValue(0.1)
        self._pid_kd.setFixedWidth(80)
        pid_layout.addWidget(self._pid_kd)

        self._pid_btn = QPushButton("发送")
        self._pid_btn.setFixedWidth(50)
        pid_layout.addWidget(self._pid_btn)

        # --- 分隔线 ---
        sep = QLabel("|")
        sep.setStyleSheet("color: #555; padding: 0 4px;")
        pid_layout.addWidget(sep)

        # PWM Bias 控件
        pid_layout.addWidget(QLabel("PWM Bias:"))
        self._pwm_bias_spin = QSpinBox()
        self._pwm_bias_spin.setRange(0, 4095)
        self._pwm_bias_spin.setValue(0)
        self._pwm_bias_spin.setFixedWidth(80)
        pid_layout.addWidget(self._pwm_bias_spin)

        self._pwm_bias_btn = QPushButton("发送")
        self._pwm_bias_btn.setFixedWidth(50)
        pid_layout.addWidget(self._pwm_bias_btn)

        # --- 分隔线 ---
        sep2 = QLabel("|")
        sep2.setStyleSheet("color: #555; padding: 0 4px;")
        pid_layout.addWidget(sep2)

        # PWM Max 控件
        pid_layout.addWidget(QLabel("PWM Max:"))
        self._pwm_max_spin = QSpinBox()
        self._pwm_max_spin.setRange(800, 999)
        self._pwm_max_spin.setValue(999)
        self._pwm_max_spin.setFixedWidth(80)
        pid_layout.addWidget(self._pwm_max_spin)

        self._pwm_max_btn = QPushButton("发送")
        self._pwm_max_btn.setFixedWidth(50)
        pid_layout.addWidget(self._pwm_max_btn)

        pid_layout.addStretch()
        splitter.addWidget(pid_container)

        # === 下: 调试控制台 ===
        console_container = QWidget()
        console_layout = QVBoxLayout(console_container)
        console_layout.setContentsMargins(0, 0, 0, 0)
        console_layout.setSpacing(4)

        # 控制台工具栏
        console_toolbar = QHBoxLayout()
        console_toolbar.setContentsMargins(0, 0, 0, 0)

        self._hid0_label = QLabel("HID0: ○ 等待")
        self._hid0_label.setStyleSheet("color: #888; font-size: 11px;")
        console_toolbar.addWidget(self._hid0_label)

        console_toolbar.addStretch()

        self._clear_btn = QPushButton("清空")
        self._clear_btn.setFixedWidth(60)
        self._clear_btn.clicked.connect(self._clear_console)
        console_toolbar.addWidget(self._clear_btn)

        console_layout.addLayout(console_toolbar)

        self._console = QPlainTextEdit()
        self._console.setReadOnly(True)
        self._console.setFont(QFont("Consolas", 9))
        self._console.setMaximumBlockCount(1000)
        self._console.setStyleSheet("""
            QPlainTextEdit {
                background-color: #1e1e1e;
                color: #d4d4d4;
                border: 1px solid #333;
                padding: 4px;
            }
        """)
        console_layout.addWidget(self._console)

        splitter.addWidget(console_container)
        splitter.setStretchFactor(0, 3)  # 表格占 3/4
        splitter.setStretchFactor(1, 1)  # 控制台占 1/4

        # 状态栏
        status = QStatusBar()
        self.setStatusBar(status)
        self._conn_hid1_label = QLabel("HID1:○")
        self._conn_hid0_label = QLabel("HID0:○")
        self._conn_hid1_label.setStyleSheet("margin-right: 8px;")
        status.addPermanentWidget(self._conn_hid1_label)
        status.addPermanentWidget(self._conn_hid0_label)

    def _connect_signals(self):
        # HID1 (电机数据)
        self._worker.data_ready.connect(self._on_data)
        self._worker.connected.connect(self._on_hid1_connected)
        self._worker.disconnected.connect(self._on_hid1_disconnected)
        self._worker.error_occurred.connect(self._on_error)
        # HID0 (调试输出)
        self._debug_worker.text_ready.connect(self._on_debug_text)
        self._debug_worker.connected.connect(self._on_hid0_connected)
        self._debug_worker.disconnected.connect(self._on_hid0_disconnected)
        self._debug_worker.error_occurred.connect(self._on_error)
        # PotWidget Slider -> HID1 OUT
        for pot in self._pots:
            pot.target_changed.connect(self._on_pot_target_changed)
        # PID 发送按钮
        self._pid_btn.clicked.connect(self._on_pid_send)
        # PWM Bias 发送按钮
        self._pwm_bias_btn.clicked.connect(self._on_pwm_bias_send)
        # PWM Max 发送按钮
        self._pwm_max_btn.clicked.connect(self._on_pwm_max_send)

    def _on_hid1_connected(self):
        self._conn_hid1_label.setText("HID1:●")
        self._conn_hid1_label.setStyleSheet("color: #4CAF50; margin-right: 8px;")
        self._report_count = 0
        self._update_overall_status()

    def _on_hid1_disconnected(self, reason: str):
        self._conn_hid1_label.setText("HID1:○")
        self._conn_hid1_label.setStyleSheet("color: #888; margin-right: 8px;")
        # 清空所有电位器
        for pot in self._pots:
            pot.update_data(0, 2048, 0)
        self._update_overall_status()

    def _on_hid0_connected(self):
        self._hid0_label.setText("HID0: ●")
        self._hid0_label.setStyleSheet("color: #4CAF50; font-size: 11px;")
        self._update_overall_status()

    def _on_hid0_disconnected(self, reason: str):
        self._hid0_label.setText("HID0: ○ 等待")
        self._hid0_label.setStyleSheet("color: #888; font-size: 11px;")
        self._update_overall_status()

    def _update_overall_status(self):
        """根据 HID1/HID0 连接状态更新总体状态"""
        h1 = "●" in self._conn_hid1_label.text()
        h0 = "●" in self._hid0_label.text()
        if h1 and h0:
            self._status_label.setText("状态: ● 已连接 (HID1+HID0)")
            self._status_label.setStyleSheet("font-weight: bold; color: #4CAF50;")
        elif h1 or h0:
            self._status_label.setText(f"状态: ● 部分连接")
            self._status_label.setStyleSheet("font-weight: bold; color: #FF9800;")
        else:
            self._status_label.setText("状态: 等待设备...")
            self._status_label.setStyleSheet("font-weight: bold; color: #888;")

    def _on_error(self, msg: str):
        self.statusBar().showMessage(f"错误: {msg}", 5000)

    def _clear_console(self):
        self._console.clear()

    def _on_debug_text(self, text: str):
        self._console.appendPlainText(text)

    def _on_pot_target_changed(self, channel: int, value: int):
        """Slider 拖动 -> 更新目标列表 -> 发送 HID1 OUT (仅单路)"""
        if 0 <= channel < 8:
            self._current_targets[channel] = value
            self._worker.send_target(channel, value)

    def _on_pid_send(self):
        """PID 发送按钮 -> 打包参数 -> HID1 OUT"""
        ch = self._pid_ch.currentData()
        kp = self._pid_kp.value()
        ki = self._pid_ki.value()
        kd = self._pid_kd.value()
        self._worker.send_pid(ch, kp, ki, kd)

    def _on_pwm_bias_send(self):
        """PWM Bias 发送按钮 -> HID1 OUT"""
        value = self._pwm_bias_spin.value()
        self._worker.send_pwm_bias(value)
        self._console.appendPlainText(f"[HID1] PWM Bias -> {value}")

    def _on_pwm_max_send(self):
        """PWM Max 发送按钮 -> HID1 OUT"""
        value = self._pwm_max_spin.value()
        self._worker.send_pwm_max(value)
        self._console.appendPlainText(f"[HID1] PWM Max -> {value}")

    def _on_data(self, report: StatusReport):
        self._latest_report = report
        self._report_count += 1

        for i, ch in enumerate(report.channels):
            if not ch.connected:
                continue
            if i < len(self._pots):
                self._pots[i].update_data(ch.adc, ch.target, ch.duty)

    def _update_fps(self):
        now = time.monotonic()
        elapsed = now - self._last_fps_time
        if elapsed > 0:
            fps = self._report_count / elapsed
            self._fps_label.setText(f"{fps:.0f} fps")
        self._report_count = 0
        self._last_fps_time = now

    def closeEvent(self, event):
        self._worker.stop()
        self._debug_worker.stop()
        self._worker.wait(3000)
        self._debug_worker.wait(3000)
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # 全局样式
    app.setStyleSheet("""
        QStatusBar {
            font-size: 12px;
        }
        QSplitter::handle {
            background: #444;
            height: 3px;
        }
        QPushButton {
            font-size: 11px;
            padding: 2px 8px;
        }
    """)

    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
