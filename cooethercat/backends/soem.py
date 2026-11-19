"""
pysoem axis backend.

Python is the master here, so this module owns the CiA 402 state machine, the
profile-position set-point handshake and the cyclic PDO exchange. The axis
contract is served entirely from the process-data image, so a status read costs
no EtherCAT traffic; an SDO read happens only to fetch an error code once a
fault appears.

Motion uses profile position and homing modes, where the drive generates the
trajectory and so tolerates host scheduling jitter.

Two facts worth knowing before trusting a commanded speed. Velocity and
acceleration are converted with the axis's position scaling, which assumes the
drive's velocity unit is encoder increments per second; an EPOS4 configured in
rpm needs a different factor. And a commanded move waits for the drive to
acknowledge its operating mode before the set-point edge is asserted, because a
drive can otherwise consume the edge while still in the previous mode.
"""

from __future__ import annotations

import enum
import logging
import threading
from collections.abc import Mapping, Sequence
from typing import Any, Final

import pysoem

from ..axis import Axis, AxisSet, AxisState, CoeType, ControllerState, MoveMode
from ..bus import EtherCATBus, EtherCATState
from ..cia402 import (
    CiA402State,
    ControlwordBit,
    ControlwordCommand,
    OperatingMode,
    Statusword,
    StatuswordBit,
)
from ..config import AxisConfig, PysoemTransport, TopologyConfig, Units
from ..epos4 import MaxonEPOS4
from ..pdo import PDOLoop

_COE_FORMATS: Final[dict[CoeType, str]] = {
    CoeType.INT8: "b",
    CoeType.UINT8: "B",
    CoeType.INT16: "h",
    CoeType.UINT16: "H",
    CoeType.INT32: "i",
    CoeType.UINT32: "I",
}

# pysoem's mailbox and frame failures, none of which derive from a common base
_SDO_ERRORS: Final = (pysoem.SdoError, pysoem.MailboxError, pysoem.PacketError, pysoem.WkcError)

LOG = logging.getLogger(__name__)

# A slave exchanging both inputs and outputs contributes 3 to the working counter.
_WORKING_COUNTER_PER_SLAVE: Final = 3

# Cycles a latched set-point is held before the new-setpoint bit is cleared, so
# the drive still sees a rising edge if one exchange is lost.
_SETPOINT_HOLD_CYCLES: Final = 2

# Cycles a commanded move may leave target-reached set before it counts as a
# zero-distance move, so commanding the present position cannot hang a caller.
_SETPOINT_ACK_CYCLES: Final = 8


class _MovePhase(enum.Enum):
    """Progress of one commanded move, used to synthesise the sequence counter."""

    IDLE = enum.auto()
    ARMED = enum.auto()
    STARTING = enum.auto()
    RUNNING = enum.auto()


class _AxisDrive(MaxonEPOS4):  # pylint: disable=too-many-instance-attributes
    """An EPOS4 plus the per-cycle derived state the axis contract needs."""

    def __init__(self, bus: EtherCATBus, config: AxisConfig) -> None:
        super().__init__(bus, config.require_slave_index(), name=config.name)
        self.axis_config = config
        self._lock = threading.RLock()

        self.desired_enabled = False
        self.move_mode = MoveMode.ABSOLUTE
        self.sequence = 0
        self.error_id = 0

        self._phase = _MovePhase.IDLE
        self._phase_cycles = 0
        self._wanted_mode = OperatingMode.PROFILE_POSITION
        self._completion_bit = StatuswordBit.TARGET_REACHED
        self._setpoint_cycles = 0
        self._previous_completion = True
        self._fault_seen = False

    def arm_move(self, mode: OperatingMode) -> None:
        """
        Request an operating mode and queue a set-point edge behind it.

        The edge is asserted by the cyclic hook once the drive reports the mode,
        so a mode change and a start command never share a cycle.
        """
        with self._lock:
            self._phase = _MovePhase.ARMED
            self._phase_cycles = 0
            self._wanted_mode = mode
            self._completion_bit = (
                self.HOMING_ATTAINED_BIT
                if mode is OperatingMode.HOMING
                else StatuswordBit.TARGET_REACHED
            )
            self._previous_completion = True
        self.set_operating_mode_pdo(mode)

    def abandon_move(self) -> None:
        """Forget an in-flight move so a halt is not later counted as completion."""
        with self._lock:
            self._phase = _MovePhase.IDLE
            self._setpoint_cycles = 0

    def set_pdo_inputs(self, values: Mapping[str, Any]) -> None:
        """Take one cycle's inputs, then advance the controlword and bookkeeping."""
        super().set_pdo_inputs(values)
        statusword = Statusword(int(values["statusword"]))
        mode_display = int(values["modes_of_operation_display"])

        self._track_fault(statusword)
        if self._start_armed_move(statusword, mode_display):
            return
        self._drive_controlword(statusword)
        self._track_completion(statusword)

    def _track_fault(self, statusword: Statusword) -> None:
        faulted = StatuswordBit.FAULT in statusword
        if faulted and not self._fault_seen:
            with self._lock:
                self._phase = _MovePhase.IDLE
                self._setpoint_cycles = 0
            self.error_id = self._read_error_code_safely()
        elif not faulted and self._fault_seen:
            self.error_id = 0
        self._fault_seen = faulted

    def _read_error_code_safely(self) -> int:
        try:
            return self.read_error_code()
        except (OSError, RuntimeError) as exc:
            LOG.warning("%s: could not read error code: %s", self.name, exc)
            return 0

    def _start_armed_move(self, statusword: Statusword, mode_display: int) -> bool:
        """Assert the set-point edge once mode and device state both allow it."""
        with self._lock:
            if self._phase is not _MovePhase.ARMED:
                return False
            if StatuswordBit.FAULT in statusword:
                return False
            if not statusword.matches(CiA402State.OPERATION_ENABLED):
                return False
            if mode_display != int(self._wanted_mode):
                return False

            homing = self._wanted_mode is OperatingMode.HOMING
            relative = self.move_mode is MoveMode.RELATIVE
            self._phase = _MovePhase.STARTING
            self._phase_cycles = 0
            self._setpoint_cycles = _SETPOINT_HOLD_CYCLES

        controlword = int(ControlwordCommand.SWITCH_ON_AND_ENABLE_OPERATION)
        controlword |= 1 << int(ControlwordBit.NEW_SETPOINT)
        if not homing:
            controlword |= 1 << int(ControlwordBit.CHANGE_SET_IMMEDIATELY)
            if relative:
                controlword |= 1 << int(ControlwordBit.ABSOLUTE_RELATIVE)
        self.set_controlword_pdo(controlword)
        return True

    def _drive_controlword(self, statusword: Statusword) -> None:
        """Hold the requested enable as a level, one transition per cycle."""
        if StatuswordBit.FAULT in statusword:
            return

        state = statusword.state
        if state is None:
            return

        with self._lock:
            if self._setpoint_cycles > 0:
                self._setpoint_cycles -= 1
                return

        if not self.desired_enabled:
            if state is not CiA402State.SWITCH_ON_DISABLED:
                self.set_controlword_pdo(ControlwordCommand.DISABLE_VOLTAGE)
            return

        match state:
            case CiA402State.SWITCH_ON_DISABLED:
                self.set_controlword_pdo(ControlwordCommand.SHUTDOWN)
            case (
                CiA402State.READY_TO_SWITCH_ON
                | CiA402State.SWITCHED_ON
                | CiA402State.OPERATION_ENABLED
                | CiA402State.QUICK_STOP_ACTIVE
            ):
                self.set_controlword_pdo(ControlwordCommand.SWITCH_ON_AND_ENABLE_OPERATION)
            case _:
                LOG.debug("%s: holding in %s", self.name, state.name)

    def _track_completion(self, statusword: Statusword) -> None:
        with self._lock:
            if self._phase not in (_MovePhase.STARTING, _MovePhase.RUNNING):
                return

            reached = self._completion_bit in statusword
            self._phase_cycles += 1

            if self._phase is _MovePhase.STARTING:
                if not reached:
                    self._phase = _MovePhase.RUNNING
                    self._phase_cycles = 0
                elif self._phase_cycles >= _SETPOINT_ACK_CYCLES:
                    # Target-reached never dropped, so the move had nowhere to go.
                    self._complete()
            elif reached and not self._previous_completion:
                self._complete()

            self._previous_completion = reached

    def _complete(self) -> None:
        self.sequence += 1
        self._phase = _MovePhase.IDLE
        self._phase_cycles = 0

    def status_snapshot(self) -> dict[str, Any]:
        """Return the contract status fields from the process-data image."""
        inputs = self.get_pdo_inputs()
        statusword = Statusword(int(inputs["statusword"]))
        units = self.axis_config.units

        with self._lock:
            sequence = self.sequence
        return {
            "position": units.to_units(int(inputs["position_actual_value"])),
            "actual_velocity": units.to_units(int(inputs["velocity_actual_value"])),
            "state": int(_axis_state(statusword, int(inputs["modes_of_operation_display"]))),
            "moving": StatuswordBit.TARGET_REACHED not in statusword,
            "enabled": statusword.matches(CiA402State.OPERATION_ENABLED),
            "ready": _is_ready(statusword),
            "error": StatuswordBit.FAULT in statusword,
            "error_id": self.error_id,
            "sequence": sequence,
        }

    def settings_snapshot(self) -> dict[str, Any]:
        """Return the contract settings as last commanded."""
        outputs = self.get_pdo_outputs()
        units = self.axis_config.units
        return {
            "enable": self.desired_enabled,
            "mode": int(self.move_mode),
            "target": units.to_units(int(outputs["target_position"])),
            "velocity": units.to_units(int(outputs["profile_velocity"])),
            "acceleration": units.to_units(int(outputs["profile_acceleration"])),
            "deceleration": units.to_units(int(outputs["profile_deceleration"])),
        }


def _axis_state(  # pylint: disable=too-many-return-statements
    statusword: Statusword, mode_display: int
) -> AxisState:
    if StatuswordBit.FAULT in statusword:
        return AxisState.ERROR_STOP

    match statusword.state:
        case CiA402State.OPERATION_ENABLED:
            if StatuswordBit.TARGET_REACHED in statusword:
                return AxisState.STANDSTILL
            if mode_display == int(OperatingMode.HOMING):
                return AxisState.HOMING
            return AxisState.DISCRETE_MOTION
        case CiA402State.QUICK_STOP_ACTIVE:
            return AxisState.STOPPING
        case CiA402State.FAULT | CiA402State.FAULT_REACTION_ACTIVE:
            return AxisState.ERROR_STOP
        case _:
            return AxisState.DISABLED


def _is_ready(statusword: Statusword) -> bool:
    return statusword.state in (
        CiA402State.SWITCH_ON_DISABLED,
        CiA402State.READY_TO_SWITCH_ON,
        CiA402State.SWITCHED_ON,
        CiA402State.OPERATION_ENABLED,
    )


class SoemBackend:
    """Axis backend driving CiA 402 drives over a pysoem soft master."""

    def __init__(
        self,
        topology: TopologyConfig,
        *,
        bus: EtherCATBus | None = None,
        period_s: float = 0.002,
    ) -> None:
        if bus is None:
            transport = topology.transport
            if not isinstance(transport, PysoemTransport):
                raise TypeError(f"topology {topology.name!r} has no pysoem transport")
            bus = EtherCATBus(transport.ifname)

        self.topology = topology
        self.bus = bus
        self.loop = PDOLoop(
            bus,
            period_s=period_s,
            expected_working_counter=_WORKING_COUNTER_PER_SLAVE * len(topology.axes),
        )
        self._drives: dict[str, _AxisDrive] = {}
        self._started = False

    @property
    def drives(self) -> Mapping[str, _AxisDrive]:
        """Return the drive objects keyed by lowercased axis name."""
        return self._drives

    def open(self) -> None:
        """Bring up the bus, map process data and start cyclic exchange."""
        if self._started:
            return

        self.bus.open()
        found = self.bus.scan()
        if len(found) < len(self.topology.axes):
            raise RuntimeError(
                f"topology {self.topology.name!r} expects {len(self.topology.axes)} "
                f"slave(s) but the bus enumerated {len(found)}"
            )

        self.bus.configure()
        for config in self.topology.axes:
            drive = _AxisDrive(self.bus, config)
            self._drives[config.keyword_name] = drive
            self.loop.add_device(drive)

        self.bus.set_master_state(EtherCATState.OP)
        self.loop.cycle_once()
        self.bus.assert_master_state(EtherCATState.OP)
        self.loop.start()
        self._started = True
        LOG.info("pysoem backend serving %d axis/axes", len(self._drives))

    def close(self) -> None:
        """Stop cyclic exchange and leave every drive de-energised."""
        if not self._started:
            return

        self._started = False
        for drive in self._drives.values():
            drive.desired_enabled = False
        self.loop.stop()

        for drive in self._drives.values():
            try:
                drive.write_controlword(ControlwordCommand.DISABLE_VOLTAGE)
            except (OSError, RuntimeError) as exc:
                LOG.warning("%s: could not de-energise on close: %s", drive.name, exc)

        self.bus.set_master_state(EtherCATState.SAFEOP)
        self.bus.close()

    def read_field(self, axis: AxisConfig, field: str) -> Any:
        """Read one field from the process-data image."""
        return self.read_fields(axis, (field,))[field]

    def read_fields(self, axis: AxisConfig, fields: Sequence[str]) -> dict[str, Any]:
        """Read several fields of one axis from the process-data image."""
        drive = self._drive(axis)
        status = drive.status_snapshot()
        settings = drive.settings_snapshot()
        try:
            return {
                field: status[field] if field in status else settings[field]
                for field in fields
            }
        except KeyError:
            raise KeyError(f"{axis.name}: unreadable field in {tuple(fields)}") from None

    def read_axes_fields(
        self, axes: Sequence[AxisConfig], fields: Sequence[str]
    ) -> dict[str, dict[str, Any]]:
        """Read the fields of every axis from the process-data image."""
        return {axis.name: self.read_fields(axis, fields) for axis in axes}

    def read_object(self, axis: AxisConfig, index: int, subindex: int, kind: CoeType) -> int:
        """Read one CoE object from the axis's drive by SDO."""
        drive = self._drive(axis)
        try:
            return int(self.bus.read_sdo(drive.slave_index, index, subindex, _COE_FORMATS[kind]))
        except _SDO_ERRORS as exc:
            raise ConnectionError(
                f"{axis.name}: SDO {index:#06x}:{subindex} failed: {exc}"
            ) from exc

    def controller_state(self) -> ControllerState:
        """Return RUN while every slave is operational and the PDO loop runs."""
        if not self._started:
            return ControllerState.STOP
        states = self.bus.read_states()
        if self.loop.running and all(state == EtherCATState.OP for state in states):
            return ControllerState.RUN
        return ControllerState.CONFIG

    def write_field(self, axis: AxisConfig, field: str, value: Any) -> None:
        """Set one settable field in the drive's outputs."""
        drive = self._drive(axis)
        units = axis.units

        match field:
            case "enable":
                drive.desired_enabled = bool(value)
            case "mode":
                mode = MoveMode(int(value))
                if mode is MoveMode.VELOCITY:
                    raise NotImplementedError(
                        f"{axis.name}: this backend commands profile position and "
                        f"homing modes only"
                    )
                drive.move_mode = mode
            case "target":
                drive.set_pdo_outputs(target_position=units.to_increments(float(value)))
            case "velocity":
                drive.set_pdo_outputs(profile_velocity=_magnitude(units, value))
            case "acceleration":
                drive.set_pdo_outputs(profile_acceleration=_magnitude(units, value))
            case "deceleration":
                drive.set_pdo_outputs(profile_deceleration=_magnitude(units, value))
            case _:
                raise KeyError(f"{axis.name}: {field!r} is not a settable field")

    def fire_trigger(self, axis: AxisConfig, field: str) -> None:
        """Start a move, homing, stop or fault reset."""
        drive = self._drive(axis)

        match field:
            case "move":
                drive.arm_move(OperatingMode.PROFILE_POSITION)
            case "home":
                drive.arm_move(OperatingMode.HOMING)
            case "stop":
                drive.abandon_move()
                drive.set_controlword_pdo(ControlwordCommand.QUICK_STOP)
            case "reset":
                drive.abandon_move()
                drive.set_controlword_pdo(ControlwordCommand.FAULT_RESET)
            case _:
                raise KeyError(f"{axis.name}: {field!r} is not an action")

    def _drive(self, axis: AxisConfig) -> _AxisDrive:
        try:
            return self._drives[axis.keyword_name]
        except KeyError:
            raise KeyError(f"axis {axis.name!r} is not open on this backend") from None


def _magnitude(units: Units, value: float) -> int:
    return abs(units.to_increments(float(value)))


def open_soem(
    topology: TopologyConfig,
    *,
    bus: EtherCATBus | None = None,
    period_s: float = 0.002,
) -> AxisSet:
    """Build a started AxisSet driven by a pysoem soft master."""
    backend = SoemBackend(topology, bus=bus, period_s=period_s)
    axes = {config.keyword_name: Axis(backend, config) for config in topology.axes}
    backend.open()
    return AxisSet(backend, axes)
