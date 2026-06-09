"""
Generic CiA 402 drive layer.

This module does the following:
- statusword parsing
- controlword commands
- CiA 402 state transitions
- operating mode selection
- generic SDO helpers for standard drive objects
"""

from __future__ import annotations

import logging
import time
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Iterable, Sequence

from .bus import EtherCATBus

LOG = logging.getLogger(__name__)


class CiA402Error(RuntimeError):
    """Base exception for CiA 402 drive errors."""


class CiA402StateError(CiA402Error):
    """Raised when a state transition cannot be completed."""


class CiA402ModeError(CiA402Error):
    """Raised when an operating mode transition cannot be completed."""


@dataclass(frozen=True)
class ObjectEntry:
    """Object dictionary entry used by the generic drive layer."""

    index: int
    subindex: int
    fmt: str
    bits: int


class CiA402Object:
    """Standard CiA 402 object dictionary entries used by this base class."""

    CONTROLWORD = ObjectEntry(0x6040, 0x00, "H", 16)
    STATUSWORD = ObjectEntry(0x6041, 0x00, "H", 16)
    MODES_OF_OPERATION = ObjectEntry(0x6060, 0x00, "b", 8)
    MODES_OF_OPERATION_DISPLAY = ObjectEntry(0x6061, 0x00, "b", 8)

    POSITION_ACTUAL_VALUE = ObjectEntry(0x6064, 0x00, "i", 32)
    VELOCITY_ACTUAL_VALUE = ObjectEntry(0x606C, 0x00, "i", 32)
    TARGET_POSITION = ObjectEntry(0x607A, 0x00, "i", 32)
    TARGET_VELOCITY = ObjectEntry(0x60FF, 0x00, "i", 32)
    PROFILE_VELOCITY = ObjectEntry(0x6081, 0x00, "I", 32)
    PROFILE_ACCELERATION = ObjectEntry(0x6083, 0x00, "I", 32)
    PROFILE_DECELERATION = ObjectEntry(0x6084, 0x00, "I", 32)
    QUICK_STOP_DECELERATION = ObjectEntry(0x6085, 0x00, "I", 32)
    FOLLOWING_ERROR_ACTUAL_VALUE = ObjectEntry(0x60F4, 0x00, "i", 32)


STATUSWORD_STATE_MASK = 0b1101111


class CiA402State(IntEnum):
    """CiA 402 finite-state automaton states encoded in the statusword."""

    NOT_READY_TO_SWITCH_ON = 0b0000000
    SWITCH_ON_DISABLED = 0b1000000
    READY_TO_SWITCH_ON = 0b0100001
    SWITCHED_ON = 0b0100011
    OPERATION_ENABLED = 0b0100111
    QUICK_STOP_ACTIVE = 0b0000111
    FAULT_REACTION_ACTIVE = 0b0001111
    FAULT = 0b0001000


class StatuswordBit(IntEnum):
    """Individual statusword bits."""

    READY_TO_SWITCH_ON = 0
    SWITCHED_ON = 1
    OPERATION_ENABLED = 2
    FAULT = 3
    VOLTAGE_ENABLED = 4
    QUICK_STOP = 5
    SWITCH_ON_DISABLED = 6
    WARNING = 7
    REMOTE = 9
    TARGET_REACHED = 10
    INTERNAL_LIMIT_ACTIVE = 11
    OPERATION_MODE_SPECIFIC_12 = 12
    OPERATION_MODE_SPECIFIC_13 = 13
    MANUFACTURER_SPECIFIC_14 = 14
    POSITION_REFERENCE_TO_HOME = 15


class ControlwordBit(IntEnum):
    """Individual controlword bits."""

    SWITCH_ON = 0
    ENABLE_VOLTAGE = 1
    QUICK_STOP = 2
    ENABLE_OPERATION = 3
    NEW_SETPOINT = 4
    START_HOMING = 4
    CHANGE_SET_IMMEDIATELY = 5
    ABSOLUTE_RELATIVE = 6
    FAULT_RESET = 7
    HALT = 8


class ControlwordCommand(IntEnum):
    """Common CiA 402 state-transition controlword commands."""

    SHUTDOWN = 0x0006
    SWITCH_ON = 0x0007
    SWITCH_ON_AND_ENABLE_OPERATION = 0x000F
    DISABLE_VOLTAGE = 0x0000
    QUICK_STOP = 0x0002
    DISABLE_OPERATION = 0x0007
    ENABLE_OPERATION = 0x000F
    FAULT_RESET = 0x0080


class MotionCommand(IntEnum):
    """Common controlword values used after a drive is operation-enabled."""

    START_HOMING = 0x001F
    HALT_HOMING = 0x011F
    ABSOLUTE_START_IMMEDIATELY = 0x003F
    RELATIVE_START_IMMEDIATELY = 0x007F


class OperatingMode(IntEnum):
    """CiA 402 modes of operation."""

    PROFILE_POSITION = 1
    PROFILE_VELOCITY = 3
    PROFILE_TORQUE = 4
    HOMING = 6
    INTERPOLATED_POSITION = 7
    CYCLIC_SYNCHRONOUS_POSITION = 8
    CYCLIC_SYNCHRONOUS_VELOCITY = 9
    CYCLIC_SYNCHRONOUS_TORQUE = 10


@dataclass(frozen=True)
class Statusword:
    """Parsed statusword value."""

    raw: int

    @property
    def masked_state(self) -> int:
        return self.raw & STATUSWORD_STATE_MASK

    @property
    def state(self) -> CiA402State | None:
        try:
            return CiA402State(self.masked_state)
        except ValueError:
            return None

    @property
    def bits_set(self) -> tuple[StatuswordBit, ...]:
        return tuple(bit for bit in StatuswordBit if self.has(bit))

    def has(self, bit: StatuswordBit) -> bool:
        return bool(self.raw & (1 << int(bit)))

    def matches(self, state: CiA402State) -> bool:
        return self.masked_state == int(state)

    def __contains__(self, bit: StatuswordBit) -> bool:
        return self.has(bit)

    def __repr__(self) -> str:
        state = self.state.name if self.state is not None else f"UNKNOWN({self.masked_state:#x})"
        return f"Statusword(raw={self.raw:#06x}, state={state}, bits={self.bits_set})"


STATE_TRANSITIONS: dict[CiA402State, dict[CiA402State, ControlwordCommand]] = {
    CiA402State.NOT_READY_TO_SWITCH_ON: {},
    CiA402State.SWITCH_ON_DISABLED: {
        CiA402State.READY_TO_SWITCH_ON: ControlwordCommand.SHUTDOWN,
    },
    CiA402State.READY_TO_SWITCH_ON: {
        CiA402State.SWITCH_ON_DISABLED: ControlwordCommand.DISABLE_VOLTAGE,
        CiA402State.SWITCHED_ON: ControlwordCommand.SWITCH_ON,
        CiA402State.OPERATION_ENABLED: ControlwordCommand.SWITCH_ON_AND_ENABLE_OPERATION,
    },
    CiA402State.SWITCHED_ON: {
        CiA402State.READY_TO_SWITCH_ON: ControlwordCommand.SHUTDOWN,
        CiA402State.OPERATION_ENABLED: ControlwordCommand.ENABLE_OPERATION,
        CiA402State.SWITCH_ON_DISABLED: ControlwordCommand.DISABLE_VOLTAGE,
    },
    CiA402State.OPERATION_ENABLED: {
        CiA402State.SWITCHED_ON: ControlwordCommand.DISABLE_OPERATION,
        CiA402State.QUICK_STOP_ACTIVE: ControlwordCommand.QUICK_STOP,
        CiA402State.READY_TO_SWITCH_ON: ControlwordCommand.SHUTDOWN,
        CiA402State.SWITCH_ON_DISABLED: ControlwordCommand.DISABLE_VOLTAGE,
    },
    CiA402State.QUICK_STOP_ACTIVE: {
        CiA402State.OPERATION_ENABLED: ControlwordCommand.ENABLE_OPERATION,
        CiA402State.SWITCH_ON_DISABLED: ControlwordCommand.DISABLE_VOLTAGE,
    },
    CiA402State.FAULT_REACTION_ACTIVE: {
        CiA402State.FAULT: ControlwordCommand.FAULT_RESET,
    },
    CiA402State.FAULT: {
        CiA402State.SWITCH_ON_DISABLED: ControlwordCommand.FAULT_RESET,
    },
}


def transition_sequence(
    start: CiA402State,
    target: CiA402State,
) -> list[ControlwordCommand]:
    """Return the shortest known controlword sequence between two states."""
    if start == target:
        return []

    queue = deque([(start, [])])
    visited = {start}

    while queue:
        state, commands = queue.popleft()
        for next_state, command in STATE_TRANSITIONS.get(state, {}).items():
            if next_state in visited:
                continue

            next_commands = [*commands, command]
            if next_state == target:
                return next_commands

            visited.add(next_state)
            queue.append((next_state, next_commands))

    raise CiA402StateError(f"No known CiA 402 transition from {start.name} to {target.name}")


class CiA402Drive:
    """
    Generic CiA 402 drive controlled through SDOs.

    Vendor-specific drives, such as MaxonEPOS4, should subclass this and add
    object dictionary details, diagnostics, PDO maps, and device-specific setup.
    """

    OBJECTS = CiA402Object
    CONTROLWORD_DELAY_S = 0.01
    STATUSWORD_POLL_S = 0.01

    def __init__(self, bus: EtherCATBus, slave_index: int, *, name: str | None = None) -> None:
        self.bus = bus
        self.slave_index = slave_index
        self.name = name or f"slave-{slave_index}"

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name={self.name!r}, slave_index={self.slave_index})"

    def read_object(self, obj: ObjectEntry) -> Any:
        """Read an object dictionary entry via SDO."""
        return self.bus.read_sdo(self.slave_index, obj.index, obj.subindex, obj.fmt)

    def write_object(self, obj: ObjectEntry, value: int | IntEnum, *, complete_access: bool = False) -> None:
        """Write an object dictionary entry via SDO."""
        value = int(value.value) if isinstance(value, IntEnum) else int(value)
        self.bus.write_sdo(
            self.slave_index,
            obj.index,
            obj.subindex,
            obj.fmt,
            value,
            complete_access=complete_access,
        )

    def read_statusword(self) -> Statusword:
        """Read and parse the drive statusword."""
        return Statusword(self.read_object(self.OBJECTS.STATUSWORD))

    @property
    def statusword(self) -> Statusword:
        return self.read_statusword()

    @property
    def state(self) -> CiA402State | None:
        return self.statusword.state

    @property
    def enabled(self) -> bool:
        return self.statusword.matches(CiA402State.OPERATION_ENABLED)

    @property
    def faulted(self) -> bool:
        return StatuswordBit.FAULT in self.statusword

    @property
    def target_reached(self) -> bool:
        return StatuswordBit.TARGET_REACHED in self.statusword

    def write_controlword(self, command: int | IntEnum) -> None:
        """Write the drive controlword."""
        self.write_object(self.OBJECTS.CONTROLWORD, command)

    def wait_for_state(
        self,
        state: CiA402State,
        *,
        timeout_s: float = 1.0,
        poll_s: float | None = None,
    ) -> Statusword:
        """Wait until the drive reaches a CiA 402 state."""
        poll_s = self.STATUSWORD_POLL_S if poll_s is None else poll_s
        deadline = time.monotonic() + timeout_s

        while True:
            statusword = self.read_statusword()
            if statusword.matches(state):
                return statusword

            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timed out waiting for {self.name} to reach {state.name}; "
                    f"current statusword: {statusword}"
                )

            time.sleep(poll_s)

    def wait_for_bits(
        self,
        bits: StatuswordBit | Iterable[StatuswordBit],
        *,
        any_bit: bool = True,
        timeout_s: float = 1.0,
        poll_s: float | None = None,
    ) -> Statusword:
        """Wait until any or all requested statusword bits are set."""
        if isinstance(bits, StatuswordBit):
            expected = (bits,)
        else:
            expected = tuple(bits)

        collapse = any if any_bit else all
        poll_s = self.STATUSWORD_POLL_S if poll_s is None else poll_s
        deadline = time.monotonic() + timeout_s

        while True:
            statusword = self.read_statusword()
            if collapse(bit in statusword for bit in expected):
                return statusword

            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timed out waiting for {self.name} statusword bits {expected}; "
                    f"current statusword: {statusword}"
                )

            time.sleep(poll_s)

    def transition_to(
        self,
        target: CiA402State,
        *,
        timeout_s: float = 2.0,
        poll_s: float | None = None,
    ) -> Statusword:
        """Move the drive through the CiA 402 state machine."""
        current = self.state
        if current is None:
            raise CiA402StateError(f"{self.name} is in an unknown state: {self.statusword}")

        commands = transition_sequence(current, target)
        LOG.debug("%s transition %s -> %s via %s", self.name, current.name, target.name, commands)

        for command in commands:
            self.write_controlword(command)
            time.sleep(self.CONTROLWORD_DELAY_S)

        return self.wait_for_state(target, timeout_s=timeout_s, poll_s=poll_s)

    def fault_reset(self, *, timeout_s: float = 2.0) -> Statusword:
        """Reset a fault and wait for Switch On Disabled."""
        self.write_controlword(ControlwordCommand.FAULT_RESET)
        time.sleep(self.CONTROLWORD_DELAY_S)
        return self.wait_for_state(CiA402State.SWITCH_ON_DISABLED, timeout_s=timeout_s)

    def enable(self, *, timeout_s: float = 2.0) -> Statusword:
        """Enable the drive for operation."""
        if self.statusword.matches(CiA402State.FAULT):
            self.fault_reset(timeout_s=timeout_s)
        return self.transition_to(CiA402State.OPERATION_ENABLED, timeout_s=timeout_s)

    def disable_operation(self, *, timeout_s: float = 2.0) -> Statusword:
        """Disable operation while keeping voltage enabled."""
        return self.transition_to(CiA402State.SWITCHED_ON, timeout_s=timeout_s)

    def shutdown(self, *, timeout_s: float = 2.0) -> Statusword:
        """Transition to Ready To Switch On."""
        return self.transition_to(CiA402State.READY_TO_SWITCH_ON, timeout_s=timeout_s)

    def disable_voltage(self, *, timeout_s: float = 2.0) -> Statusword:
        """Disable voltage and transition to Switch On Disabled."""
        return self.transition_to(CiA402State.SWITCH_ON_DISABLED, timeout_s=timeout_s)

    def quick_stop(self, *, timeout_s: float = 2.0) -> Statusword:
        """Command quick stop."""
        return self.transition_to(CiA402State.QUICK_STOP_ACTIVE, timeout_s=timeout_s)

    stop = quick_stop
    halt = quick_stop

    def set_operating_mode(
        self,
        mode: OperatingMode,
        *,
        verify: bool = True,
        timeout_s: float = 1.0,
    ) -> OperatingMode:
        """Set the CiA 402 operating mode."""
        self.write_object(self.OBJECTS.MODES_OF_OPERATION, mode)
        if verify:
            return self.wait_for_operating_mode(mode, timeout_s=timeout_s)
        return mode

    def read_operating_mode(self) -> OperatingMode | int:
        """Read the drive's displayed operating mode."""
        value = self.read_object(self.OBJECTS.MODES_OF_OPERATION_DISPLAY)
        try:
            return OperatingMode(value)
        except ValueError:
            return value

    def wait_for_operating_mode(
        self,
        mode: OperatingMode,
        *,
        timeout_s: float = 1.0,
        poll_s: float | None = None,
    ) -> OperatingMode:
        """Wait until the operating mode display matches the requested mode."""
        poll_s = self.STATUSWORD_POLL_S if poll_s is None else poll_s
        deadline = time.monotonic() + timeout_s

        while True:
            current = self.read_operating_mode()
            if current == mode:
                return mode

            if time.monotonic() >= deadline:
                raise CiA402ModeError(
                    f"Timed out waiting for {self.name} to enter {mode.name}; "
                    f"current mode display: {current}"
                )

            time.sleep(poll_s)

    def read_position(self) -> int:
        return int(self.read_object(self.OBJECTS.POSITION_ACTUAL_VALUE))

    def read_velocity(self) -> int:
        return int(self.read_object(self.OBJECTS.VELOCITY_ACTUAL_VALUE))

    def write_target_position(self, position: int) -> None:
        self.write_object(self.OBJECTS.TARGET_POSITION, position)

    def write_target_velocity(self, velocity: int) -> None:
        self.write_object(self.OBJECTS.TARGET_VELOCITY, velocity)

    def configure_profile_motion(
        self,
        *,
        velocity: int,
        acceleration: int,
        deceleration: int | None = None,
        quick_stop_deceleration: int | None = None,
    ) -> None:
        """Configure standard profile motion parameters."""
        deceleration = acceleration if deceleration is None else deceleration
        self.write_object(self.OBJECTS.PROFILE_VELOCITY, abs(int(velocity)))
        self.write_object(self.OBJECTS.PROFILE_ACCELERATION, abs(int(acceleration)))
        self.write_object(self.OBJECTS.PROFILE_DECELERATION, abs(int(deceleration)))
        if quick_stop_deceleration is not None:
            self.write_object(self.OBJECTS.QUICK_STOP_DECELERATION, abs(int(quick_stop_deceleration)))

    def profile_position_move_sdo(
        self,
        position: int,
        *,
        velocity: int,
        acceleration: int,
        deceleration: int | None = None,
        absolute: bool = True,
        timeout_s: float = 2.0,
    ) -> None:
        """
        Start a profile-position move using SDO writes. For regular
        cyclic control, a PDO layer should handle the process-data path.
        """
        self.set_operating_mode(OperatingMode.PROFILE_POSITION, timeout_s=timeout_s)
        self.configure_profile_motion(
            velocity=velocity,
            acceleration=acceleration,
            deceleration=deceleration,
        )
        self.enable(timeout_s=timeout_s)
        self.write_target_position(int(position))
        command = (
            MotionCommand.ABSOLUTE_START_IMMEDIATELY
            if absolute
            else MotionCommand.RELATIVE_START_IMMEDIATELY
        )
        self.write_controlword(command)
