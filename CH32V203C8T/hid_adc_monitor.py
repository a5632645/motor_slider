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
        Qt, QThread, pyqtSignal, QTimer, QMutex, QPointF
    )
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QTableWidget, QTableWidgetItem, QHeaderView, QLabel, QStatusBar,
        QStyledItemDelegate, QPlainTextEdit, QPushButton, QSplitter,
    )
    from PyQt6.QtGui import (
        QColor, QPalette, QFont, QPainter, QLinearGradient,
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
OFFSET_ADC     = 2       # uint16 LE × 8,  偏移 2～17
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
    channels: List[MotorChannelData] = field(default_factory=list)

    @classmethod
    def from_bytes(cls, report: bytes) -> Optional["StatusReport"]:
        """从 64 字节 HID 报告解析"""
        if len(report) < REPORT_SIZE:
            return None

        result = cls()
        result.running = bool(report[OFFSET_STATUS] & 0x01)

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
# 带进度条的表格项委托
# ---------------------------------------------------------------------------
class ProgressBarDelegate(QStyledItemDelegate):
    """在单元格内绘制自定义进度条"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._bar_height = 16

    def paint(self, painter, option, index):
        super().paint(painter, option, index)

        value = index.data(Qt.ItemDataRole.UserRole)
        if value is None:
            return

        percent = max(0.0, min(100.0, float(value)))

        # 计算进度条区域
        rect = option.rect.adjusted(4, 4, -4, -4)
        bar_width = int(rect.width() * percent / 100.0)

        # 背景
        painter.save()
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        bg_color = option.palette.color(QPalette.ColorRole.Window)
        painter.setBrush(bg_color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(rect, 3, 3)

        # 前景 — 渐变
        if percent > 0:
            gradient = QLinearGradient(QPointF(rect.topLeft()), QPointF(rect.topRight()))
            if percent < 30:
                gradient.setColorAt(0.0, QColor("#4CAF50"))   # 绿色
                gradient.setColorAt(1.0, QColor("#81C784"))
            elif percent < 70:
                gradient.setColorAt(0.0, QColor("#FF9800"))   # 橙色
                gradient.setColorAt(1.0, QColor("#FFB74D"))
            else:
                gradient.setColorAt(0.0, QColor("#F44336"))   # 红色
                gradient.setColorAt(1.0, QColor("#E57373"))

            bar_rect = rect.adjusted(0, 0, -(rect.width() - bar_width), 0)
            painter.setBrush(gradient)
            painter.drawRoundedRect(bar_rect, 3, 3)

            # 百分比文字
            painter.setPen(Qt.GlobalColor.white)
            font = painter.font()
            font.setPointSize(8)
            painter.setFont(font)
            text = f"{percent:.0f}%"
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)

        painter.restore()

    def sizeHint(self, option, index):
        size = super().sizeHint(option, index)
        return size


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    COL_CH    = 0
    COL_ADC   = 1
    COL_TGT   = 2
    COL_DUTY  = 3
    COL_BAR   = 4
    COL_COUNT = 5

    COL_HEADERS = ["#", "ADC", "目标", "占空比", "位置"]

    def __init__(self):
        super().__init__()
        self._worker = HidWorker()
        self._debug_worker = HidDebugWorker()
        self._report_count = 0
        self._fps = 0.0
        self._last_fps_time = time.monotonic()
        self._latest_report: Optional[StatusReport] = None

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
        self.setMinimumSize(720, 560)
        self.resize(780, 640)

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

        # 垂直分割器: 上 = 表格, 下 = 调试控制台
        splitter = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(splitter, stretch=1)

        # === 上: ADC 表格 ===
        table_container = QWidget()
        table_layout = QVBoxLayout(table_container)
        table_layout.setContentsMargins(0, 0, 0, 0)

        self._table = QTableWidget(MOTOR_COUNT, self.COL_COUNT)
        self._table.setHorizontalHeaderLabels(self.COL_HEADERS)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self._table.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # 列宽
        hdr = self._table.horizontalHeader()
        hdr.setSectionResizeMode(self.COL_CH,   QHeaderView.ResizeMode.Fixed)
        hdr.setSectionResizeMode(self.COL_ADC,  QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(self.COL_TGT,  QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(self.COL_DUTY, QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(self.COL_BAR,  QHeaderView.ResizeMode.Stretch)
        self._table.setColumnWidth(self.COL_CH, 40)

        # 进度条委托
        self._bar_delegate = ProgressBarDelegate(self._table)
        self._table.setItemDelegateForColumn(self.COL_BAR, self._bar_delegate)

        # 行高
        self._table.verticalHeader().setDefaultSectionSize(36)

        # 交替行颜色
        self._table.setAlternatingRowColors(True)

        table_layout.addWidget(self._table)
        splitter.addWidget(table_container)

        # 初始化表格内容
        font = QFont("Consolas", 10)
        for row in range(MOTOR_COUNT):
            # # 列
            item = QTableWidgetItem(str(row + 1))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            item.setFont(font)
            self._table.setItem(row, self.COL_CH, item)

            for col in (self.COL_ADC, self.COL_TGT, self.COL_DUTY):
                item = QTableWidgetItem("—")
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                item.setFont(font)
                self._table.setItem(row, col, item)

            # 进度条列
            item = QTableWidgetItem()
            item.setData(Qt.ItemDataRole.UserRole, 0.0)
            self._table.setItem(row, self.COL_BAR, item)

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

    def _on_hid1_connected(self):
        self._conn_hid1_label.setText("HID1:●")
        self._conn_hid1_label.setStyleSheet("color: #4CAF50; margin-right: 8px;")
        self._report_count = 0
        self._update_overall_status()

    def _on_hid1_disconnected(self, reason: str):
        self._conn_hid1_label.setText("HID1:○")
        self._conn_hid1_label.setStyleSheet("color: #888; margin-right: 8px;")
        # 清空 ADC 数据
        for row in range(MOTOR_COUNT):
            for col in (self.COL_ADC, self.COL_TGT, self.COL_DUTY):
                self._table.item(row, col).setText("—")
            self._table.item(row, self.COL_BAR).setData(Qt.ItemDataRole.UserRole, 0.0)
        self._table.viewport().update()
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

    def _on_data(self, report: StatusReport):
        self._latest_report = report
        self._report_count += 1

        for i, ch in enumerate(report.channels):
            if not ch.connected:
                continue
            self._table.item(i, self.COL_ADC).setText(str(ch.adc))
            self._table.item(i, self.COL_TGT).setText(str(ch.target))
            self._table.item(i, self.COL_DUTY).setText(str(ch.duty))

            percent = (ch.adc / 4095.0) * 100.0 if ch.adc <= 4095 else 0.0
            self._table.item(i, self.COL_BAR).setData(Qt.ItemDataRole.UserRole, percent)

        # 触发重绘（列委拖不会自动刷新）
        self._table.viewport().update()

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
        QTableWidget {
            font-size: 13px;
        }
        QTableWidget::item {
            padding: 2px 4px;
        }
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
