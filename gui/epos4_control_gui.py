#!/usr/bin/env python3

from __future__ import annotations

import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from PyQt5.QtCore import QObject, QRunnable, Qt, QThreadPool, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

HERE = Path(__file__).resolve().parent
if (HERE / "cooethercat").is_dir():
    sys.path.insert(0, str(HERE))

try:
    from cooethercat import EtherCATBus, EtherCATState, MaxonEPOS4, OperatingMode, PDOLoop
except Exception:
    print("Failed to import cooethercat. Install it with `pip install -e .` from the project root.")
    traceback.print_exc()
    raise

APP_NAME = "EPOS4 EtherCAT Control"
DEFAULT_QSS = HERE / "epos4_control.qss"
REFRESH_MS = 1000

COMMON_SDO_HINTS = (
    "Common objects: 0x6040 Controlword (H), 0x6041 Statusword (H), "
    "0x6060 Modes of operation (b), 0x6061 Mode display (b), "
    "0x6064 Position actual (i), 0x607A Target position (i)."
)

LED_HELP = """EPOS4 LED quick reference

NET status green LED:
  OFF = INIT
  Blink = PRE-OPERATIONAL
  Single flash = SAFE-OPERATIONAL
  ON = OPERATIONAL
  Flicker = BOOTSTRAP

NET status red LED:
  OFF = operating condition
  Double flash = application / Sync Manager watchdog timeout
  Single flash = changed COM state due to internal error
  Blink = general configuration error

Device status:
  Green slow, red off = power stage disabled: Switch On Disabled, Ready To Switch On, or Switched On
  Green on, red off = power stage enabled: Operation Enabled or Quick Stop Active
  Green off, red on = Fault
  Green on, red on = Fault Reaction Active
"""


@dataclass(frozen=True)
class MotionValues:
    position: int
    velocity: int
    acceleration: int
    absolute: bool


def load_qss(path: Path = DEFAULT_QSS) -> str:
    """Load an optional QSS stylesheet."""
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return ""


def network_interfaces() -> list[str]:
    """Return likely EtherCAT network interfaces for Linux/macOS/dev systems."""
    sys_class_net = Path("/sys/class/net")
    if sys_class_net.is_dir():
        interfaces = sorted(p.name for p in sys_class_net.iterdir() if p.name != "lo")
        if interfaces:
            return interfaces
    return ["eno1", "eno2", "eno3", "enp4s0", "enp5s0", "en0", "en1"]


def set_object_name(widget: QWidget, name: str) -> QWidget:
    widget.setObjectName(name)
    return widget


def make_text_box(*, readonly: bool = True, min_height: int | None = None) -> QTextEdit:
    box = QTextEdit()
    box.setReadOnly(readonly)
    if min_height is not None:
        box.setMinimumHeight(min_height)
    return box


class WorkerSignals(QObject):
    result = pyqtSignal(object)
    error = pyqtSignal(str)
    finished = pyqtSignal()


class Worker(QRunnable):
    """Run a blocking hardware function away from the GUI thread."""

    def __init__(self, fn: Callable[[], Any]) -> None:
        super().__init__()
        self.fn = fn
        self.signals = WorkerSignals()

    @pyqtSlot()
    def run(self) -> None:
        try:
            self.signals.result.emit(self.fn())
        except Exception:
            self.signals.error.emit(traceback.format_exc())
        finally:
            self.signals.finished.emit()


class EPOS4ControlWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1180, 760)

        self.bus: EtherCATBus | None = None
        self.drive: MaxonEPOS4 | None = None
        self.pdo_loop: PDOLoop | None = None
        self.slaves_initialized = False
        self.refresh_in_progress = False

        self.thread_pool = QThreadPool.globalInstance()
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(REFRESH_MS)
        self.refresh_timer.timeout.connect(self.refresh_status)

        self._build_ui()
        self._set_bus_controls(False)
        self._set_drive_controls(False)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.addWidget(self._build_connection_group())

        tabs = QTabWidget()
        tabs.addTab(self._build_status_tab(), "Status")
        tabs.addTab(self._build_commands_tab(), "Commands")
        tabs.addTab(self._build_sdo_tab(), "SDO Tool")
        tabs.addTab(self._build_pdo_tab(), "PDO")
        tabs.addTab(self._build_led_help_tab(), "LED Help")
        main.addWidget(tabs, 1)

        self.log_box = make_text_box(min_height=160)
        main.addWidget(self.log_box)

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QGridLayout(group)

        self.iface_combo = QComboBox()
        self.iface_combo.setEditable(True)
        self.iface_combo.addItems(network_interfaces())

        self.slave_spin = QSpinBox()
        self.slave_spin.setRange(0, 255)
        self.slave_spin.setValue(0)
        self.slave_spin.valueChanged.connect(lambda _: self.create_drive(auto=True))

        self.open_button = set_object_name(QPushButton("Open + Scan"), "openButton")
        self.scan_button = set_object_name(QPushButton("Rescan"), "scanButton")
        self.create_drive_button = QPushButton("Create EPOS4")
        self.close_button = set_object_name(QPushButton("Close"), "closeButton")
        self.auto_refresh_check = QCheckBox("Auto refresh")
        self.auto_refresh_check.setChecked(True)

        layout.addWidget(QLabel("Interface"), 0, 0)
        layout.addWidget(self.iface_combo, 0, 1)
        layout.addWidget(QLabel("Slave index"), 0, 2)
        layout.addWidget(self.slave_spin, 0, 3)
        layout.addWidget(self.open_button, 0, 4)
        layout.addWidget(self.scan_button, 0, 5)
        layout.addWidget(self.create_drive_button, 0, 6)
        layout.addWidget(self.close_button, 0, 7)
        layout.addWidget(self.auto_refresh_check, 0, 8)

        self.open_button.clicked.connect(self.open_bus)
        self.scan_button.clicked.connect(self.scan_bus)
        self.create_drive_button.clicked.connect(lambda: self.create_drive(auto=False))
        self.close_button.clicked.connect(self.close_bus)
        self.auto_refresh_check.toggled.connect(self._sync_refresh_timer)
        return group

    def _build_status_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        state_group = QGroupBox("Verified State")
        form = QFormLayout(state_group)
        self.status_labels: dict[str, QLabel] = {}
        for label, key in [
            ("Bus open", "bus_open"),
            ("NMT states", "nmt_states"),
            ("CiA 402 state", "device_state"),
            ("Statusword", "statusword"),
            ("Status bits", "status_bits"),
            ("Mode display", "mode"),
            ("Position", "position"),
            ("Velocity", "velocity"),
            ("Error", "error"),
            ("Temperature", "temperature"),
        ]:
            value = QLabel("—")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setWordWrap(True)
            self.status_labels[key] = value
            form.addRow(label, value)

        slave_group = QGroupBox("Slave Scan")
        slave_layout = QVBoxLayout(slave_group)
        self.slave_info_box = make_text_box()
        slave_layout.addWidget(self.slave_info_box)
        refresh_button = set_object_name(QPushButton("Refresh Now"), "refreshButton")
        refresh_button.clicked.connect(self.refresh_status)
        slave_layout.addWidget(refresh_button)

        layout.addWidget(state_group, 1)
        layout.addWidget(slave_group, 1)
        return tab

    def _build_commands_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        nmt_group = QGroupBox("EtherCAT NMT / ESM")
        nmt_layout = QHBoxLayout(nmt_group)
        for text, state in [
            ("PRE-OP", EtherCATState.PREOP),
            ("SAFE-OP", EtherCATState.SAFEOP),
            ("OPERATIONAL", EtherCATState.OP),
        ]:
            button = QPushButton(text)
            button.clicked.connect(lambda _checked=False, s=state: self.set_nmt(s))
            nmt_layout.addWidget(button)
            setattr(self, f"nmt_{state.name.lower()}_button", button)
        layout.addWidget(nmt_group)

        device_group = QGroupBox("CiA 402 Device Commands")
        cmd_layout = QGridLayout(device_group)
        self.fault_reset_button = set_object_name(QPushButton("Fault Reset"), "faultResetButton")
        self.shutdown_button = set_object_name(QPushButton("Shutdown / Ready"), "shutdownButton")
        self.enable_button = set_object_name(QPushButton("Enable Operation"), "enableButton")
        self.disable_op_button = QPushButton("Disable Operation")
        self.disable_voltage_button = QPushButton("Disable Voltage")
        self.quick_stop_button = set_object_name(QPushButton("Quick Stop"), "quickStopButton")

        for pos, button in enumerate([
            self.fault_reset_button,
            self.shutdown_button,
            self.enable_button,
            self.disable_op_button,
            self.disable_voltage_button,
            self.quick_stop_button,
        ]):
            cmd_layout.addWidget(button, pos // 3, pos % 3)

        self.fault_reset_button.clicked.connect(lambda: self.drive_command("Fault reset", lambda d: d.fault_reset()))
        self.shutdown_button.clicked.connect(lambda: self.drive_command("Shutdown", lambda d: d.shutdown()))
        self.enable_button.clicked.connect(self.enable_drive)
        self.disable_op_button.clicked.connect(lambda: self.drive_command("Disable operation", lambda d: d.disable_operation()))
        self.disable_voltage_button.clicked.connect(lambda: self.drive_command("Disable voltage", lambda d: d.disable_voltage()))
        self.quick_stop_button.clicked.connect(lambda: self.drive_command("Quick stop", lambda d: d.quick_stop()))
        layout.addWidget(device_group)

        layout.addWidget(self._build_mode_group())
        layout.addWidget(self._build_motion_group())
        layout.addStretch(1)
        return tab

    def _build_mode_group(self) -> QGroupBox:
        group = QGroupBox("Operating Mode")
        layout = QHBoxLayout(group)
        self.mode_combo = QComboBox()
        for mode in OperatingMode:
            self.mode_combo.addItem(f"{mode.name} ({int(mode)})", mode)
        self.set_mode_button = QPushButton("Set Mode")
        self.set_mode_button.clicked.connect(self.set_operating_mode)
        layout.addWidget(self.mode_combo)
        layout.addWidget(self.set_mode_button)
        return group

    def _build_motion_group(self) -> QGroupBox:
        group = QGroupBox("Profile Position Move via SDO")
        layout = QFormLayout(group)
        self.target_spin = QSpinBox()
        self.target_spin.setRange(-2_000_000_000, 2_000_000_000)
        self.velocity_spin = QSpinBox()
        self.velocity_spin.setRange(1, 2_000_000_000)
        self.velocity_spin.setValue(5000)
        self.accel_spin = QSpinBox()
        self.accel_spin.setRange(1, 2_000_000_000)
        self.accel_spin.setValue(10000)
        self.absolute_check = QCheckBox("Absolute move")
        self.absolute_check.setChecked(True)
        self.move_button = set_object_name(QPushButton("Move"), "moveButton")
        self.move_button.clicked.connect(self.profile_position_move)

        layout.addRow("Target position [counts]", self.target_spin)
        layout.addRow("Velocity [counts/s or drive units]", self.velocity_spin)
        layout.addRow("Acceleration", self.accel_spin)
        layout.addRow("Move type", self.absolute_check)
        layout.addRow(self.move_button)
        return group

    def _build_sdo_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("Raw SDO Read/Write")
        form = QFormLayout(group)

        self.sdo_index_edit = QLineEdit("0x6041")
        self.sdo_subindex_edit = QLineEdit("0x00")
        self.sdo_fmt_edit = QLineEdit("H")
        self.sdo_value_edit = QLineEdit("0")
        self.sdo_result_label = QLabel("—")
        self.sdo_result_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.sdo_read_button = QPushButton("Read SDO")
        self.sdo_write_button = set_object_name(QPushButton("Write SDO"), "sdoWriteButton")
        self.sdo_read_button.clicked.connect(self.read_sdo)
        self.sdo_write_button.clicked.connect(self.write_sdo)

        buttons = QHBoxLayout()
        buttons.addWidget(self.sdo_read_button)
        buttons.addWidget(self.sdo_write_button)

        form.addRow("Index", self.sdo_index_edit)
        form.addRow("Subindex", self.sdo_subindex_edit)
        form.addRow("struct fmt", self.sdo_fmt_edit)
        form.addRow("write value", self.sdo_value_edit)
        form.addRow(buttons)
        form.addRow("Result", self.sdo_result_label)
        layout.addWidget(group)

        hints = QLabel(COMMON_SDO_HINTS)
        hints.setWordWrap(True)
        layout.addWidget(hints)
        layout.addStretch(1)
        return tab

    def _build_pdo_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("Cyclic PDO Loop")
        form = QFormLayout(group)

        self.pdo_period_spin = QDoubleSpinBox()
        self.pdo_period_spin.setDecimals(4)
        self.pdo_period_spin.setRange(0.0005, 1.0)
        self.pdo_period_spin.setSingleStep(0.001)
        self.pdo_period_spin.setValue(0.002)
        self.pdo_status_label = QLabel("Stopped")
        self.pdo_start_button = QPushButton("Start PDO Loop + Request OP")
        self.pdo_stop_button = set_object_name(QPushButton("Stop PDO Loop"), "pdoStopButton")
        self.pdo_enable_button = set_object_name(QPushButton("Queue Enable"), "pdoEnableButton")
        self.pdo_quick_stop_button = set_object_name(QPushButton("Queue Quick Stop"), "pdoQuickStopButton")
        self.pdo_move_button = set_object_name(QPushButton("Queue Current Move Values"), "pdoMoveButton")

        pdo_buttons = QHBoxLayout()
        for button in [
            self.pdo_start_button,
            self.pdo_stop_button,
            self.pdo_enable_button,
            self.pdo_quick_stop_button,
            self.pdo_move_button,
        ]:
            pdo_buttons.addWidget(button)

        form.addRow("Period [s]", self.pdo_period_spin)
        form.addRow("Status", self.pdo_status_label)
        form.addRow(pdo_buttons)
        layout.addWidget(group)

        self.pdo_input_box = make_text_box()
        layout.addWidget(self.pdo_input_box, 1)

        self.pdo_start_button.clicked.connect(self.start_pdo)
        self.pdo_stop_button.clicked.connect(self.stop_pdo)
        self.pdo_enable_button.clicked.connect(lambda: self.pdo_command("Queue enable", lambda d: d.queue_enable_pdo()))
        self.pdo_quick_stop_button.clicked.connect(lambda: self.pdo_command("Queue quick stop", lambda d: d.queue_quick_stop_pdo()))
        self.pdo_move_button.clicked.connect(self.queue_pdo_move)
        return tab

    def _build_led_help_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        text = make_text_box()
        text.setPlainText(LED_HELP)
        layout.addWidget(text)
        return tab

    def log(self, message: str) -> None:
        self.log_box.append(str(message))
        self.log_box.moveCursor(QTextCursor.End)

    def run_worker(
        self,
        label: str,
        fn: Callable[[], Any],
        on_result: Callable[[Any], None] | None = None,
        *,
        refresh_after: bool = True,
    ) -> None:
        self.log(f"▶ {label}")
        worker = Worker(fn)
        worker.signals.result.connect(lambda result: self._worker_result(label, result, on_result, refresh_after))
        worker.signals.error.connect(lambda tb: self._worker_error(label, tb))
        self.thread_pool.start(worker)

    def _worker_result(
        self,
        label: str,
        result: Any,
        on_result: Callable[[Any], None] | None,
        refresh_after: bool,
    ) -> None:
        self.log(f"✓ {label}")
        if on_result is not None:
            on_result(result)
        if refresh_after and self.auto_refresh_check.isChecked():
            self.refresh_status()

    def _worker_error(self, label: str, traceback_text: str) -> None:
        self.log(f"✗ {label}\n{traceback_text}")
        QMessageBox.critical(self, label, traceback_text)

    def _require_bus(self) -> EtherCATBus:
        if self.bus is None:
            raise RuntimeError("Open the EtherCAT bus first.")
        return self.bus

    def _require_scanned_bus(self) -> EtherCATBus:
        bus = self._require_bus()
        if not self.slaves_initialized:
            raise RuntimeError("Scan the EtherCAT bus before reading states or creating drives.")
        return bus

    def _require_drive(self) -> MaxonEPOS4:
        if self.drive is None:
            raise RuntimeError("Create the EPOS4 drive object first.")
        return self.drive

    def _selected_interface(self) -> str:
        return self.iface_combo.currentText().strip()

    def _motion_values(self) -> MotionValues:
        return MotionValues(
            position=int(self.target_spin.value()),
            velocity=int(self.velocity_spin.value()),
            acceleration=int(self.accel_spin.value()),
            absolute=bool(self.absolute_check.isChecked()),
        )

    def _confirm(self, title: str, message: str) -> bool:
        return QMessageBox.question(self, title, message) == QMessageBox.Yes

    def _set_bus_controls(self, connected: bool) -> None:
        self.open_button.setEnabled(not connected)
        for widget in [self.scan_button, self.create_drive_button, self.close_button]:
            widget.setEnabled(connected)

    def _set_drive_controls(self, enabled: bool) -> None:
        controls = [
            self.nmt_preop_button,
            self.nmt_safeop_button,
            self.nmt_op_button,
            self.fault_reset_button,
            self.shutdown_button,
            self.enable_button,
            self.disable_op_button,
            self.disable_voltage_button,
            self.quick_stop_button,
            self.set_mode_button,
            self.move_button,
            self.sdo_read_button,
            self.sdo_write_button,
            self.pdo_start_button,
            self.pdo_stop_button,
            self.pdo_enable_button,
            self.pdo_quick_stop_button,
            self.pdo_move_button,
        ]
        for widget in controls:
            widget.setEnabled(enabled)

    def _sync_refresh_timer(self) -> None:
        if self.auto_refresh_check.isChecked() and self.bus is not None and self.slaves_initialized:
            self.refresh_timer.start()
        else:
            self.refresh_timer.stop()

    # Bus / drive operations
    def open_bus(self) -> None:
        iface = self._selected_interface()
        if not iface:
            QMessageBox.warning(self, "Missing interface", "Please enter an EtherCAT interface name.")
            return

        def task() -> list[Any]:
            self.bus = EtherCATBus(iface)
            self.bus.open()
            return self.bus.scan()

        def done(slaves: list[Any]) -> None:
            self.slaves_initialized = True
            self._set_bus_controls(True)
            self._set_drive_controls(True)
            self._apply_slave_scan(slaves)
            if slaves:
                self.slave_spin.setRange(0, max(0, len(slaves) - 1))
                self.create_drive(auto=True)
            self._sync_refresh_timer()
            self.log(f"Opened {iface} and found {len(slaves)} slave(s)")

        self.run_worker("Open bus + scan", task, done, refresh_after=False)

    def close_bus(self) -> None:
        self.refresh_timer.stop()

        def task() -> str:
            if self.pdo_loop is not None:
                self.pdo_loop.stop()
                self.pdo_loop = None
            if self.bus is not None:
                self.bus.close()
            self.drive = None
            self.bus = None
            self.slaves_initialized = False
            return "Closed bus"

        def done(message: str) -> None:
            self.log(message)
            self._set_bus_controls(False)
            self._set_drive_controls(False)
            self._clear_status()

        self.run_worker("Close bus", task, done, refresh_after=False)

    def scan_bus(self) -> None:
        def task() -> list[Any]:
            return self._require_bus().scan()

        def done(slaves: list[Any]) -> None:
            self.slaves_initialized = True
            self._apply_slave_scan(slaves)
            if slaves:
                self.slave_spin.setRange(0, max(0, len(slaves) - 1))
            self._sync_refresh_timer()

        self.run_worker("Scan slaves", task, done, refresh_after=False)

    def _apply_slave_scan(self, slaves: list[Any]) -> None:
        text = "\n".join(
            f"[{s.index}] {s.name} man={s.manufacturer_id} id={s.product_id} rev={s.revision} state={s.state}"
            for s in slaves
        )
        self.slave_info_box.setPlainText(text or "No slaves found")

    def create_drive(self, *, auto: bool = False) -> None:
        if self.bus is None or not self.slaves_initialized:
            if not auto:
                QMessageBox.warning(self, "Scan first", "Open and scan the EtherCAT bus before creating a drive.")
            return

        index = int(self.slave_spin.value())
        self.drive = MaxonEPOS4(self.bus, slave_index=index, name=f"epos4-{index}")
        self.log(f"Created MaxonEPOS4 for slave {index}")
        if self.auto_refresh_check.isChecked():
            self.refresh_status()

    def set_nmt(self, state: EtherCATState) -> None:
        def task() -> str:
            bus = self._require_scanned_bus()
            bus.set_master_state(state)
            ok = bus.wait_for_master_state(state, timeout_us=500_000)
            return f"Requested NMT {state.name}; reached={ok}; states={bus.read_states()}"

        self.run_worker(f"Set NMT {state.name}", task, self.log)

    # Status refresh
    def refresh_status(self) -> None:
        if self.refresh_in_progress:
            return
        if self.bus is None:
            self.status_labels["bus_open"].setText("No")
            return
        if not self.slaves_initialized:
            self.log("Skipping refresh: bus has not been scanned yet.")
            return

        self.refresh_in_progress = True

        def task() -> dict[str, Any]:
            bus = self._require_scanned_bus()
            data: dict[str, Any] = {"bus_open": bus.is_open, "nmt_states": bus.read_states()}
            if self.drive is not None:
                drive = self.drive
                statusword = drive.read_statusword()
                err = drive.read_error()
                data.update(
                    state=statusword.state.name if statusword.state else f"UNKNOWN({statusword.masked_state:#x})",
                    statusword=statusword.raw,
                    bits=", ".join(bit.name for bit in statusword.bits_set),
                    mode=drive.read_operating_mode(),
                    position=drive.read_position(),
                    velocity=drive.read_velocity(),
                    error=str(err),
                    temperature=drive.temperature_c(),
                )
                if self.pdo_loop is not None and self.pdo_loop.running:
                    data["pdo"] = drive.get_pdo_inputs()
                    data["pdo_stats"] = self.pdo_loop.last_stats
            return data

        def done(data: dict[str, Any]) -> None:
            self.refresh_in_progress = False
            self._apply_status(data)

        def on_error(traceback_text: str) -> None:
            self.refresh_in_progress = False
            self._worker_error("Refresh status", traceback_text)

        worker = Worker(task)
        worker.signals.result.connect(done)
        worker.signals.error.connect(on_error)
        self.thread_pool.start(worker)

    def _apply_status(self, data: dict[str, Any]) -> None:
        self.status_labels["bus_open"].setText("Yes" if data.get("bus_open") else "No")
        self.status_labels["nmt_states"].setText(str(data.get("nmt_states", "—")))
        self.status_labels["device_state"].setText(str(data.get("state", "—")))
        if "statusword" in data:
            self.status_labels["statusword"].setText(f"0x{data['statusword']:04X} ({data['statusword']})")
        else:
            self.status_labels["statusword"].setText("—")
        self.status_labels["status_bits"].setText(str(data.get("bits", "—")))
        self.status_labels["mode"].setText(str(data.get("mode", "—")))
        self.status_labels["position"].setText(str(data.get("position", "—")))
        self.status_labels["velocity"].setText(str(data.get("velocity", "—")))
        self.status_labels["error"].setText(str(data.get("error", "—")))
        temp = data.get("temperature")
        self.status_labels["temperature"].setText("—" if temp is None else f"{temp:.1f} °C")

        if "pdo" in data:
            self.pdo_status_label.setText("Running")
            self.pdo_input_box.setPlainText(f"Inputs: {data['pdo']}\n\nStats: {data.get('pdo_stats')}")
        elif self.pdo_loop and self.pdo_loop.running:
            self.pdo_status_label.setText("Running")
        else:
            self.pdo_status_label.setText("Stopped")

    def _clear_status(self) -> None:
        for label in self.status_labels.values():
            label.setText("—")
        self.status_labels["bus_open"].setText("No")
        self.pdo_status_label.setText("Stopped")
        self.pdo_input_box.clear()

    # Device commands
    def drive_command(self, label: str, fn: Callable[[MaxonEPOS4], Any]) -> None:
        self.run_worker(label, lambda: fn(self._require_drive()), lambda result: self.log(f"{label} result: {result}"))

    def enable_drive(self) -> None:
        if self._confirm("Enable Operation", "Enable Operation may energize the motor. Continue?"):
            self.drive_command("Enable operation", lambda d: d.enable())

    def set_operating_mode(self) -> None:
        mode = self.mode_combo.currentData()
        self.drive_command(f"Set mode {mode.name}", lambda d: d.set_operating_mode(mode))

    def profile_position_move(self) -> None:
        if not self._confirm("Move motor", "This command can move the motor. Confirm that the mechanism is safe to move."):
            return
        values = self._motion_values()

        def task() -> str:
            self._require_drive().profile_position_move_sdo(
                position=values.position,
                velocity=values.velocity,
                acceleration=values.acceleration,
                absolute=values.absolute,
            )
            return f"Started {'absolute' if values.absolute else 'relative'} move to/by {values.position}"

        self.run_worker("Profile position move", task, self.log)

    # Raw SDO tool
    def _sdo_args(self) -> tuple[int, int, str]:
        index = int(self.sdo_index_edit.text().strip(), 0)
        subindex = int(self.sdo_subindex_edit.text().strip(), 0)
        fmt = self.sdo_fmt_edit.text().strip()
        if not fmt:
            raise ValueError("struct fmt cannot be empty")
        return index, subindex, fmt

    def read_sdo(self) -> None:
        def task() -> Any:
            index, subindex, fmt = self._sdo_args()
            return self._require_scanned_bus().read_sdo(int(self.slave_spin.value()), index, subindex, fmt)

        self.run_worker("Read SDO", task, lambda value: self.sdo_result_label.setText(repr(value)))

    def write_sdo(self) -> None:
        if not self._confirm("Write SDO", "Raw SDO writes can change drive configuration or cause motion. Continue?"):
            return

        def task() -> str:
            index, subindex, fmt = self._sdo_args()
            value = int(self.sdo_value_edit.text().strip(), 0)
            self._require_scanned_bus().write_sdo(int(self.slave_spin.value()), index, subindex, fmt, value)
            return f"Wrote {value} to {index:#06x}:{subindex:#04x} fmt={fmt}"

        self.run_worker("Write SDO", task, lambda msg: self.sdo_result_label.setText(msg))

    # PDO support
    def start_pdo(self) -> None:
        if not self._confirm("Start PDO loop", "Starting PDO and requesting Operational can energize cyclic communication. Continue?"):
            return

        def task() -> str:
            bus = self._require_scanned_bus()
            drive = self._require_drive()
            if self.pdo_loop is not None and self.pdo_loop.running:
                return "PDO loop already running"
            self.pdo_loop = PDOLoop(bus, period_s=float(self.pdo_period_spin.value()))
            self.pdo_loop.add_device(drive)
            self.pdo_loop.start()
            bus.set_master_state(EtherCATState.OP)
            return "PDO loop started and OP requested"

        self.run_worker("Start PDO", task, self.log)

    def stop_pdo(self) -> None:
        def task() -> str:
            if self.pdo_loop is not None:
                self.pdo_loop.stop()
                self.pdo_loop = None
            if self.bus is not None and self.slaves_initialized:
                self.bus.set_master_state(EtherCATState.SAFEOP)
            return "PDO loop stopped; SAFE-OP requested"

        self.run_worker("Stop PDO", task, self.log)

    def pdo_command(self, label: str, fn: Callable[[MaxonEPOS4], Any]) -> None:
        self.run_worker(label, lambda: (fn(self._require_drive()), label)[1], self.log)

    def queue_pdo_move(self) -> None:
        if not self._confirm("Queue PDO move", "This command can move the motor on the next PDO cycle. Continue?"):
            return
        values = self._motion_values()

        def task() -> str:
            self._require_drive().queue_profile_position_pdo(
                position=values.position,
                velocity=values.velocity,
                acceleration=values.acceleration,
                absolute=values.absolute,
            )
            return f"Queued PDO move to/by {values.position}"

        self.run_worker("Queue PDO move", task, self.log)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.refresh_timer.stop()
        try:
            if self.pdo_loop is not None:
                self.pdo_loop.stop()
            if self.bus is not None:
                self.bus.close()
        finally:
            event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setStyle("Fusion")
    app.setStyleSheet(load_qss())
    window = EPOS4ControlWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
