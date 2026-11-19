"""
Fake pysoem master carrying simulated EPOS4 drives.

``EtherCATBus`` takes an injected master, so the whole soft-master stack can be
exercised without a NIC or a drive: the CiA 402 state machine, profile-position
moves, homing, faults and the cyclic PDO path all run against this.

The slave decodes its output PDO and encodes its input PDO using the real
layouts, so a mismatch between a layout and the code driving it shows up here
rather than on the bench. Motion advances a fixed step per exchange, which keeps
tests deterministic in cycles rather than wall-clock time.
"""

from __future__ import annotations

import struct
from typing import Any, Final

from cooethercat.bus import EtherCATState
from cooethercat.cia402 import (
    CiA402State,
    ControlwordBit,
    OperatingMode,
    StatuswordBit,
)
from cooethercat.epos4 import (
    DEFAULT_PROFILE_POSITION_RX_PDO,
    DEFAULT_PROFILE_POSITION_TX_PDO,
    EPOS4Object,
)
from cooethercat.pdo import PDOLayout

# Controlword bits that drive the state machine, ignoring the motion bits.
_STATE_COMMAND_MASK: Final = 0b1000_1111

_WORKING_COUNTER_PER_SLAVE: Final = 3
_NOMINAL_CYCLE_S: Final = 0.002


class FakeEPOS4Slave:
    """One simulated EPOS4, addressed the way pysoem exposes a slave."""

    def __init__(
        self,
        *,
        name: str = "EPOS4",
        rx_pdo: PDOLayout = DEFAULT_PROFILE_POSITION_RX_PDO,
        tx_pdo: PDOLayout = DEFAULT_PROFILE_POSITION_TX_PDO,
        cycle_s: float = _NOMINAL_CYCLE_S,
        mode_ack_delay_cycles: int = 0,
    ) -> None:
        self.name = name
        self.mode_ack_delay_cycles = mode_ack_delay_cycles
        self._mode_countdown = 0
        self.man = 0x000000FB
        self.id = 0x69500000
        self.rev = 0x01700000
        self.state = int(EtherCATState.PREOP)
        self.rx_pdo = rx_pdo
        self.tx_pdo = tx_pdo
        self.cycle_s = cycle_s

        self.output = bytes(rx_pdo.size_bytes)
        self.watchdogs: dict[str, float] = {}
        self.written_objects: dict[tuple[int, int], int] = {}

        self.drive_state = CiA402State.SWITCH_ON_DISABLED
        self.position = 0.0
        self.velocity = 0.0
        self.target_position = 0
        self.profile_velocity = 0
        self.mode = int(OperatingMode.PROFILE_POSITION)
        self.error_code = 0
        self.error_register = 0
        self.target_reached = True
        self.homing_attained = False

        self.last_controlword = 0
        self._goal: float | None = None
        self._objects: dict[tuple[int, int], int] = {}
        self.raw_objects: dict[tuple[int, int], bytes] = {}

    # --- pysoem slave surface ------------------------------------------------
    @property
    def input(self) -> bytes:
        """Return the input PDO image the master would have received."""
        return self.tx_pdo.pack(
            {
                "statusword": self.statusword,
                "position_actual_value": round(self.position),
                "velocity_actual_value": round(self.velocity),
                "following_error_actual_value": 0,
                "modes_of_operation_display": self.mode,
                "digital_inputs": 0,
            }
        )

    def sdo_read(self, index: int, subindex: int) -> bytes:
        if (index, subindex) in self.raw_objects:
            return self.raw_objects[(index, subindex)]
        entry = _object_entry(index, subindex)
        value = self._sdo_value(index, subindex)
        return struct.pack("<" + entry.fmt, value)

    def sdo_write(self, index: int, subindex: int, payload: bytes, ca: bool = False) -> None:
        entry = _object_entry(index, subindex)
        (value,) = struct.unpack("<" + entry.fmt, payload)
        self.written_objects[(index, subindex)] = value
        self._objects[(index, subindex)] = value

        if (index, subindex) == _key(EPOS4Object.CONTROLWORD):
            self.apply_controlword(value)
        elif (index, subindex) == _key(EPOS4Object.MODES_OF_OPERATION):
            self.mode = value
        elif (index, subindex) == _key(EPOS4Object.TARGET_POSITION):
            self.target_position = value
        elif (index, subindex) == _key(EPOS4Object.PROFILE_VELOCITY):
            self.profile_velocity = value

    def set_watchdog(self, kind: str, timeout_ms: float) -> None:
        self.watchdogs[kind] = timeout_ms

    def write_state(self) -> None:
        """Accept the requested EtherCAT state, which this fake always reaches."""

    # --- simulated drive behaviour ------------------------------------------
    @property
    def statusword(self) -> int:
        word = int(self.drive_state) | 1 << int(StatuswordBit.REMOTE)
        if self.drive_state not in (CiA402State.SWITCH_ON_DISABLED, CiA402State.FAULT):
            word |= 1 << int(StatuswordBit.VOLTAGE_ENABLED)
        if self.target_reached:
            word |= 1 << int(StatuswordBit.TARGET_REACHED)
        if self.homing_attained:
            word |= 1 << int(StatuswordBit.OPERATION_MODE_SPECIFIC_12)
        return word

    def inject_fault(self, error_code: int = 0x8182, error_register: int = 0x10) -> None:
        """Latch a drive fault, as a real EPOS4 does on its own fault reaction."""
        self.error_code = error_code
        self.error_register = error_register
        self.drive_state = CiA402State.FAULT
        self._goal = None
        self.velocity = 0.0
        self.target_reached = True

    def apply_controlword(self, controlword: int) -> None:
        """Run one state-machine and motion step for a controlword value."""
        command = controlword & _STATE_COMMAND_MASK
        self._apply_state_command(command)
        self._apply_motion(controlword)
        self.last_controlword = controlword

    def _apply_state_command(self, command: int) -> None:
        if self.drive_state is CiA402State.FAULT:
            if command & (1 << int(ControlwordBit.FAULT_RESET)):
                self.drive_state = CiA402State.SWITCH_ON_DISABLED
                self.error_code = 0
                self.error_register = 0
            return

        match command & 0b1111:
            case 0b0000:
                self.drive_state = CiA402State.SWITCH_ON_DISABLED
            case 0b0110:
                self.drive_state = CiA402State.READY_TO_SWITCH_ON
            case 0b0111:
                if self.drive_state is not CiA402State.SWITCH_ON_DISABLED:
                    self.drive_state = CiA402State.SWITCHED_ON
            case 0b1111:
                if self.drive_state in (
                    CiA402State.READY_TO_SWITCH_ON,
                    CiA402State.SWITCHED_ON,
                    CiA402State.OPERATION_ENABLED,
                    CiA402State.QUICK_STOP_ACTIVE,
                ):
                    self.drive_state = CiA402State.OPERATION_ENABLED
            case 0b0010:
                if self.drive_state is CiA402State.OPERATION_ENABLED:
                    self.drive_state = CiA402State.QUICK_STOP_ACTIVE
                    self._goal = None
                    self.velocity = 0.0
                    self.target_reached = True

        if self.drive_state is not CiA402State.OPERATION_ENABLED:
            self._goal = None
            self.velocity = 0.0

    def _apply_motion(self, controlword: int) -> None:
        if self.drive_state is not CiA402State.OPERATION_ENABLED:
            return

        started = bool(controlword & (1 << int(ControlwordBit.NEW_SETPOINT))) and not (
            self.last_controlword & (1 << int(ControlwordBit.NEW_SETPOINT))
        )

        if self.mode == int(OperatingMode.HOMING):
            if started:
                self.position = 0.0
                self.velocity = 0.0
                self.homing_attained = True
                self.target_reached = True
            return

        self.homing_attained = False
        if started:
            relative = bool(controlword & (1 << int(ControlwordBit.ABSOLUTE_RELATIVE)))
            self._goal = (
                self.position + self.target_position if relative else float(self.target_position)
            )
            self.target_reached = False

    def exchange(self) -> None:
        """Advance one PDO cycle: decode outputs, step motion, expose inputs."""
        values = self.rx_pdo.unpack(self.output)
        self._settle_mode(int(values["modes_of_operation"]))
        self.target_position = int(values["target_position"])
        self.profile_velocity = int(values["profile_velocity"])
        self.apply_controlword(int(values["controlword"]))
        self._advance()

    def _settle_mode(self, requested: int) -> None:
        """Take a requested operating mode after a configurable acknowledgement lag."""
        if requested == self.mode:
            self._mode_countdown = 0
            return

        if self._mode_countdown < self.mode_ack_delay_cycles:
            self._mode_countdown += 1
            return

        self.mode = requested
        self._mode_countdown = 0

    def _advance(self) -> None:
        if self._goal is None:
            self.velocity = 0.0
            return

        stride = abs(self.profile_velocity) * self.cycle_s
        remaining = self._goal - self.position
        if stride <= 0.0:
            self.velocity = 0.0
            return

        if abs(remaining) <= stride:
            self.position = self._goal
            self.velocity = 0.0
            self._goal = None
            self.target_reached = True
            return

        direction = 1.0 if remaining > 0 else -1.0
        self.position += direction * stride
        self.velocity = direction * abs(self.profile_velocity)

    def _sdo_value(self, index: int, subindex: int) -> int:
        match (index, subindex):
            case key if key == _key(EPOS4Object.STATUSWORD):
                return self.statusword
            case key if key == _key(EPOS4Object.CONTROLWORD):
                return self.last_controlword
            case key if key == _key(EPOS4Object.POSITION_ACTUAL_VALUE):
                return round(self.position)
            case key if key == _key(EPOS4Object.VELOCITY_ACTUAL_VALUE):
                return round(self.velocity)
            case key if key == _key(EPOS4Object.MODES_OF_OPERATION_DISPLAY):
                return self.mode
            case key if key == _key(EPOS4Object.ERROR_CODE):
                return self.error_code
            case key if key == _key(EPOS4Object.ERROR_REGISTER):
                return self.error_register
        return self._objects.get((index, subindex), 0)


class FakeMaster:
    """Stand-in for ``pysoem.Master`` over a set of simulated slaves."""

    def __init__(self, slaves: list[FakeEPOS4Slave] | None = None) -> None:
        self._all = slaves if slaves is not None else [FakeEPOS4Slave()]
        self.slaves: list[FakeEPOS4Slave] | None = None
        self.state = int(EtherCATState.PREOP)
        self.is_open = False
        self.exchanges = 0
        self.link_up = True

    def open(self, ifname: str) -> None:
        self.ifname = ifname
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def config_init(self) -> int:
        self.slaves = self._all
        return len(self._all)

    def config_map(self) -> None:
        """Accept the process-data mapping, which this fake needs no work for."""

    def send_processdata(self) -> int:
        return _WORKING_COUNTER_PER_SLAVE * len(self._all)

    def receive_processdata(self, timeout: int = 2000) -> int:
        if not self.link_up:
            raise ConnectionError("simulated link loss")

        self.exchanges += 1
        for slave in self._all:
            slave.exchange()
        return _WORKING_COUNTER_PER_SLAVE * len(self._all)

    def read_state(self) -> int:
        return self.state

    def write_state(self) -> None:
        """Move every slave to the requested network state, which this fake always reaches."""
        for slave in self._all:
            slave.state = self.state

    def state_check(self, state: int, timeout: int = 50_000) -> int:
        return state


def _key(entry: Any) -> tuple[int, int]:
    return entry.index, entry.subindex


_OBJECT_ENTRIES: Final = {
    _key(entry): entry
    for klass in reversed(EPOS4Object.__mro__)
    for entry in vars(klass).values()
    if hasattr(entry, "index") and hasattr(entry, "fmt")
}


def _object_entry(index: int, subindex: int) -> Any:
    try:
        return _OBJECT_ENTRIES[(index, subindex)]
    except KeyError:
        raise KeyError(f"fake EPOS4 has no object {index:#06x}:{subindex:02x}") from None


__all__ = ["FakeEPOS4Slave", "FakeMaster"]
