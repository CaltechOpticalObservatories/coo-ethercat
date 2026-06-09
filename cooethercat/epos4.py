"""
This module is for Maxon/EPOS4-specific behavior:
- EPOS4 object dictionary entries not already covered by CiA 402
- default EPOS4 profile-position PDO layouts
- EPOS4 diagnostics and error lookup
- EPOS4 homing setup helpers
- PDO input/output buffering for PDOLoop
"""

from __future__ import annotations

import logging
import threading
import time
from enum import Enum, IntEnum, auto
from typing import Any, Mapping, MutableMapping

from .bus import EtherCATBus
from .cia402 import (
    CiA402Drive,
    CiA402Object,
    ControlwordCommand,
    MotionCommand,
    ObjectEntry,
    OperatingMode,
    Statusword,
    StatuswordBit,
)
from .maxon_errors import EPOS4Error, get_epos4_error
from .pdo import PDOEntry, PDOLayout

LOG = logging.getLogger(__name__)


class MaxonEPOS4Error(RuntimeError):
    """Base exception for Maxon EPOS4 errors."""


class PositionSource(Enum):
    """Sources for assigning the home position."""

    SSI = auto()
    CURRENT = auto()
    USER = auto()


class HomingMethod(IntEnum):
    """EPOS4 homing methods used by the existing driver."""

    ACTUAL_POSITION = 37
    INDEX_POSITIVE_SPEED = 34
    INDEX_NEGATIVE_SPEED = 33
    HOME_SWITCH_POSITIVE_SPEED = 23
    HOME_SWITCH_NEGATIVE_SPEED = 27
    LIMIT_SWITCH_POSITIVE = 18
    LIMIT_SWITCH_NEGATIVE = 17
    CURRENT_THRESHOLD_POS_SPEED_AND_INDEX = -1
    CURRENT_THRESHOLD_NEG_SPEED_AND_INDEX = -2


class ProgramControl(IntEnum):
    """EPOS4 program-control commands."""

    INITIATE_DEVICE_RESET = 0x02


class EPOS4Object(CiA402Object):
    """EPOS4 object dictionary entries used by MaxonEPOS4."""

    # Identity / diagnostics
    ERROR_REGISTER = ObjectEntry(0x1001, 0x00, "B", 8)
    ERROR_CODE = ObjectEntry(0x603F, 0x00, "H", 16)
    SERIAL_NUMBER = ObjectEntry(0x1018, 0x04, "I", 32)
    SERIAL_NUMBER_COMPLETE = ObjectEntry(0x2100, 0x01, "Q", 64)

    DIAGNOSIS_HISTORY_NEWEST_MESSAGE = ObjectEntry(0x10F3, 0x02, "B", 8)
    DIAGNOSIS_HISTORY_NEW_MESSAGES_AVAILABLE = ObjectEntry(0x10F3, 0x04, "c", 1)
    DIAGNOSIS_HISTORY_FLAGS = ObjectEntry(0x10F3, 0x05, "H", 16)
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_1 = ObjectEntry(0x10F3, 0x06, "IHHQ", 128)
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_2 = ObjectEntry(0x10F3, 0x07, "IHHQ", 128)
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_3 = ObjectEntry(0x10F3, 0x08, "IHHQ", 128)
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_4 = ObjectEntry(0x10F3, 0x09, "IHHQ", 128)
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_5 = ObjectEntry(0x10F3, 0x0A, "IHHQ", 128)

    NODE_ID = ObjectEntry(0x2000, 0x00, "B", 8)
    PROGRAM_CONTROL = ObjectEntry(0x1F51, 0x01, "B", 8)
    TEMPERATURE_DECICELSIUS = ObjectEntry(0x3201, 0x01, "h", 16)

    # Standard CiA 402 objects also used by Maxon-specific methods.
    VELOCITY_DEMAND_VALUE = ObjectEntry(0x606B, 0x00, "i", 32)
    TORQUE_ACTUAL_VALUE = ObjectEntry(0x6077, 0x00, "H", 16)
    DIGITAL_INPUTS = ObjectEntry(0x60FD, 0x00, "I", 32)
    PHYSICAL_OUTPUTS = ObjectEntry(0x60FE, 0x01, "i", 32)

    # Homing
    HOMING_METHOD = ObjectEntry(0x6098, 0x00, "b", 8)
    HOME_POSITION = ObjectEntry(0x30B0, 0x00, "i", 32)
    HOMING_ACCELERATION = ObjectEntry(0x609A, 0x00, "i", 32)
    HOME_OFFSET_MOVE_DISTANCE = ObjectEntry(0x30B1, 0x00, "i", 32)
    SPEED_FOR_SWITCH_SEARCH = ObjectEntry(0x6099, 0x01, "I", 32)
    SPEED_FOR_ZERO_SEARCH = ObjectEntry(0x6099, 0x02, "I", 32)
    HOMING_CURRENT_THRESHOLD = ObjectEntry(0x30B2, 0x00, "H", 16)
    CURRENT_ACTUAL_VALUE_AVERAGED = ObjectEntry(0x30D1, 0x01, "i", 32)
    CURRENT_ACTUAL_VALUE_INSTANT = ObjectEntry(0x30D1, 0x02, "i", 32)
    SSI_POSITION_RAW_VALUE = ObjectEntry(0x3012, 0x09, "I", 32)

    # Motor/controller configuration commonly fetched for commissioning.
    NOMINAL_CURRENT_MA = ObjectEntry(0x3001, 0x01, "I", 32)
    OUTPUT_CURRENT_LIMIT_MA = ObjectEntry(0x3001, 0x02, "I", 32)
    NUMBER_OF_POLE_PAIRS = ObjectEntry(0x3001, 0x03, "B", 8)
    THERMAL_TIME_CONSTANT_WINDING_DS = ObjectEntry(0x3001, 0x04, "H", 16)
    TORQUE_CONSTANT_UNM_A = ObjectEntry(0x3001, 0x05, "I", 32)

    DIGITAL_INCREMENTAL_ENCODER_1 = ObjectEntry(0x3010, 0x01, "I", 32)
    DIGITAL_INCREMENTAL_ENCODER_1_TYPE = ObjectEntry(0x3010, 0x02, "H", 16)
    GEAR_REDUCTION_NUMERATOR = ObjectEntry(0x3003, 0x01, "I", 32)
    GEAR_REDUCTION_DENOMINATOR = ObjectEntry(0x3003, 0x02, "I", 32)
    GEAR_MAX_INPUT_SPEED_RPM = ObjectEntry(0x3003, 0x03, "I", 32)
    GEAR_ORIENTATION = ObjectEntry(0x3003, 0x04, "I", 32)

    POSITION_CONTROLLER_P_GAIN = ObjectEntry(0x30A1, 0x01, "I", 32)
    POSITION_CONTROLLER_I_GAIN = ObjectEntry(0x30A1, 0x02, "I", 32)
    POSITION_CONTROLLER_D_GAIN = ObjectEntry(0x30A1, 0x03, "I", 32)
    POSITION_CONTROLLER_FF_VELOCITY_GAIN = ObjectEntry(0x30A1, 0x04, "I", 32)
    POSITION_CONTROLLER_FF_ACCELERATION_GAIN = ObjectEntry(0x30A1, 0x05, "I", 32)

    VELOCITY_CONTROLLER_P_GAIN = ObjectEntry(0x30A2, 0x01, "I", 32)
    VELOCITY_CONTROLLER_I_GAIN = ObjectEntry(0x30A2, 0x02, "I", 32)
    VELOCITY_CONTROLLER_FF_VELOCITY_GAIN = ObjectEntry(0x30A2, 0x03, "I", 32)
    VELOCITY_CONTROLLER_FF_ACCELERATION_GAIN = ObjectEntry(0x30A2, 0x04, "I", 32)


def _pdo_entry(name: str, obj: ObjectEntry) -> PDOEntry:
    return PDOEntry(name, obj.index, obj.subindex, obj.fmt, obj.bits)


DEFAULT_PROFILE_POSITION_RX_PDO = PDOLayout.from_entries(
    [
        _pdo_entry("controlword", EPOS4Object.CONTROLWORD),
        _pdo_entry("target_position", EPOS4Object.TARGET_POSITION),
        _pdo_entry("profile_acceleration", EPOS4Object.PROFILE_ACCELERATION),
        _pdo_entry("profile_deceleration", EPOS4Object.PROFILE_DECELERATION),
        _pdo_entry("profile_velocity", EPOS4Object.PROFILE_VELOCITY),
        _pdo_entry("modes_of_operation", EPOS4Object.MODES_OF_OPERATION),
        _pdo_entry("physical_outputs", EPOS4Object.PHYSICAL_OUTPUTS),
    ]
)


DEFAULT_PROFILE_POSITION_TX_PDO = PDOLayout.from_entries(
    [
        _pdo_entry("statusword", EPOS4Object.STATUSWORD),
        _pdo_entry("position_actual_value", EPOS4Object.POSITION_ACTUAL_VALUE),
        _pdo_entry("velocity_actual_value", EPOS4Object.VELOCITY_ACTUAL_VALUE),
        _pdo_entry("following_error_actual_value", EPOS4Object.FOLLOWING_ERROR_ACTUAL_VALUE),
        _pdo_entry("modes_of_operation_display", EPOS4Object.MODES_OF_OPERATION_DISPLAY),
        _pdo_entry("digital_inputs", EPOS4Object.DIGITAL_INPUTS),
    ]
)


class MaxonEPOS4(CiA402Drive):
    """
    This class is both:
    - a CiA402Drive for SDO/state-machine operations
    - a PDODevice-compatible object for PDOLoop
    """

    OBJECTS = EPOS4Object
    HOMING_ATTAINED_BIT = StatuswordBit.OPERATION_MODE_SPECIFIC_12
    HOMING_ERROR_BIT = StatuswordBit.OPERATION_MODE_SPECIFIC_13

    def __init__(
        self,
        bus: EtherCATBus,
        slave_index: int,
        *,
        name: str | None = None,
        rx_pdo: PDOLayout = DEFAULT_PROFILE_POSITION_RX_PDO,
        tx_pdo: PDOLayout = DEFAULT_PROFILE_POSITION_TX_PDO,
    ) -> None:
        super().__init__(bus, slave_index, name=name)
        self.rx_pdo = rx_pdo
        self.tx_pdo = tx_pdo
        self._pdo_lock = threading.RLock()
        self._pdo_outputs: MutableMapping[str, Any] = self.rx_pdo.empty_values()
        self._pdo_inputs: MutableMapping[str, Any] = self.tx_pdo.empty_values()

        # Safe-ish defaults for the default profile-position PDO map.
        self._pdo_outputs.update(
            controlword=int(ControlwordCommand.SHUTDOWN),
            target_position=0,
            profile_acceleration=0,
            profile_deceleration=0,
            profile_velocity=0,
            modes_of_operation=int(OperatingMode.PROFILE_POSITION),
            physical_outputs=0,
        )

    # Generic object helpers
    def read_epos_object(self, obj: ObjectEntry) -> Any:
        return self.read_object(obj)

    def write_epos_object(self, obj: ObjectEntry, value: int | IntEnum) -> None:
        self.write_object(obj, value)

    # Diagnostics
    def read_error_code(self) -> int:
        return int(self.read_object(self.OBJECTS.ERROR_CODE))

    def read_error_register(self) -> int:
        return int(self.read_object(self.OBJECTS.ERROR_REGISTER))

    def read_error(self) -> EPOS4Error:
        code = self.read_error_code()
        return get_epos4_error(code, error_register=self.read_error_register())

    def read_diagnosis_history(self) -> list[Any]:
        """Read the five EPOS4 diagnosis-history message slots."""
        return [
            self.read_object(self.OBJECTS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_1),
            self.read_object(self.OBJECTS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_2),
            self.read_object(self.OBJECTS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_3),
            self.read_object(self.OBJECTS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_4),
            self.read_object(self.OBJECTS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_5),
        ]

    def temperature_c(self) -> float:
        """Return drive temperature in Celsius."""
        return float(self.read_object(self.OBJECTS.TEMPERATURE_DECICELSIUS)) / 10.0

    def reset_device(self) -> int:
        """
        Request an EPOS4 device reset.

        A reset may require bus/device reinitialization afterward.
        """
        LOG.warning("Resetting EPOS4 %s; reinitialization may be required", self.name)
        self.write_object(self.OBJECTS.PROGRAM_CONTROL, ProgramControl.INITIATE_DEVICE_RESET)
        return int(self.read_object(self.OBJECTS.PROGRAM_CONTROL))

    def info(self) -> dict[str, Any]:
        """Small SDO-based status dictionary."""
        statusword = self.read_statusword()
        return {
            "name": self.name,
            "slave_index": self.slave_index,
            "state": statusword.state.name if statusword.state else None,
            "statusword": statusword.raw,
            "position": self.read_position(),
            "velocity": self.read_velocity(),
            "target_position": self.read_object(self.OBJECTS.TARGET_POSITION),
            "mode": self.read_operating_mode(),
            "error_register": self.read_error_register(),
            "error": self.read_error(),
        }

    def debug_info(self) -> dict[str, Any]:
        """More detailed SDO-based debug snapshot."""
        data = self.info()
        data.update(
            velocity_demand=self.read_object(self.OBJECTS.VELOCITY_DEMAND_VALUE),
            target_velocity=self.read_object(self.OBJECTS.TARGET_VELOCITY),
            profile_velocity=self.read_object(self.OBJECTS.PROFILE_VELOCITY),
            torque_actual=self.read_object(self.OBJECTS.TORQUE_ACTUAL_VALUE),
            controlword=self.read_object(self.OBJECTS.CONTROLWORD),
            temperature_c=self.temperature_c(),
        )
        return data


    # Configuration
    CONFIG_OBJECT_NAMES: tuple[str, ...] = (
        "NODE_ID",
        "SERIAL_NUMBER_COMPLETE",
        "NOMINAL_CURRENT_MA",
        "OUTPUT_CURRENT_LIMIT_MA",
        "NUMBER_OF_POLE_PAIRS",
        "THERMAL_TIME_CONSTANT_WINDING_DS",
        "TORQUE_CONSTANT_UNM_A",
        "DIGITAL_INCREMENTAL_ENCODER_1",
        "DIGITAL_INCREMENTAL_ENCODER_1_TYPE",
        "GEAR_REDUCTION_NUMERATOR",
        "GEAR_REDUCTION_DENOMINATOR",
        "GEAR_MAX_INPUT_SPEED_RPM",
        "GEAR_ORIENTATION",
        "POSITION_CONTROLLER_P_GAIN",
        "POSITION_CONTROLLER_I_GAIN",
        "POSITION_CONTROLLER_D_GAIN",
        "POSITION_CONTROLLER_FF_VELOCITY_GAIN",
        "POSITION_CONTROLLER_FF_ACCELERATION_GAIN",
        "VELOCITY_CONTROLLER_P_GAIN",
        "VELOCITY_CONTROLLER_I_GAIN",
        "VELOCITY_CONTROLLER_FF_VELOCITY_GAIN",
        "VELOCITY_CONTROLLER_FF_ACCELERATION_GAIN",
    )

    def fetch_config(self) -> dict[str, Any]:
        """Fetch commonly useful EPOS4 commissioning/configuration values."""
        config: dict[str, Any] = {}
        for name in self.CONFIG_OBJECT_NAMES:
            config[name] = self.read_object(getattr(self.OBJECTS, name))
        return config

    def load_config(self, values: Mapping[str, int]) -> None:
        """
        Write EPOS4 configuration values by object-name.

        Use with care. This intentionally does no policy/safety validation.
        """
        for name, value in values.items():
            self.write_object(getattr(self.OBJECTS, name), value)

    # Homing
    def setup_homing(
        self,
        method: HomingMethod,
        *,
        position_source: PositionSource | None = None,
        position: int | None = None,
        current_threshold: int = 400,
        homing_acceleration: int = 5000,
        switch_search_speed: int = 4000,
        zero_search_speed: int = 2000,
        offset_distance: int = 100,
        home_position: int = 0,
        timeout_s: float = 1.0,
    ) -> None:
        """Configure EPOS4 homing parameters using SDO writes."""
        home_position_value = self._resolve_home_position(
            method,
            position_source=position_source,
            position=position,
        )

        self.shutdown(timeout_s=timeout_s)
        self.set_operating_mode(OperatingMode.HOMING, timeout_s=timeout_s)

        sign = -1 if method in (
            HomingMethod.CURRENT_THRESHOLD_POS_SPEED_AND_INDEX,
            HomingMethod.LIMIT_SWITCH_POSITIVE,
            HomingMethod.HOME_SWITCH_POSITIVE_SPEED,
            HomingMethod.INDEX_POSITIVE_SPEED,
        ) else 1

        self.write_object(self.OBJECTS.HOME_POSITION, home_position_value)
        self.write_object(self.OBJECTS.HOMING_ACCELERATION, int(homing_acceleration))
        self.write_object(self.OBJECTS.SPEED_FOR_SWITCH_SEARCH, int(switch_search_speed))
        self.write_object(self.OBJECTS.SPEED_FOR_ZERO_SEARCH, int(zero_search_speed))
        self.write_object(self.OBJECTS.HOMING_CURRENT_THRESHOLD, int(current_threshold))
        self.write_object(self.OBJECTS.HOME_OFFSET_MOVE_DISTANCE, int(offset_distance) * sign)
        self.write_object(self.OBJECTS.HOME_POSITION, int(home_position))
        self.write_object(self.OBJECTS.HOMING_METHOD, int(method))

    def home(
        self,
        method: HomingMethod,
        *,
        position_source: PositionSource | None = None,
        position: int | None = None,
        current_threshold: int = 400,
        wait: bool = True,
        timeout_s: float = 30.0,
    ) -> Statusword | None:
        """Configure and start homing."""
        self.setup_homing(
            method,
            position_source=position_source,
            position=position,
            current_threshold=current_threshold,
        )
        self.enable()
        self.write_controlword(MotionCommand.START_HOMING)

        if not wait:
            return None

        return self.wait_for_homing_done(timeout_s=timeout_s)

    def wait_for_homing_done(self, *, timeout_s: float = 30.0) -> Statusword:
        """Wait until homing attained, homing error, or fault."""
        deadline = time.monotonic() + timeout_s
        while True:
            statusword = self.read_statusword()
            if self.HOMING_ATTAINED_BIT in statusword:
                return statusword
            if self.HOMING_ERROR_BIT in statusword or StatuswordBit.FAULT in statusword:
                raise MaxonEPOS4Error(f"Homing failed for {self.name}: {statusword}")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out waiting for homing on {self.name}: {statusword}")
            time.sleep(self.STATUSWORD_POLL_S)

    def _resolve_home_position(
        self,
        method: HomingMethod,
        *,
        position_source: PositionSource | None,
        position: int | None,
    ) -> int:
        if method != HomingMethod.ACTUAL_POSITION:
            return 0

        if position_source is None:
            raise ValueError("position_source is required for ACTUAL_POSITION homing")

        if position_source == PositionSource.SSI:
            return int(self.read_object(self.OBJECTS.SSI_POSITION_RAW_VALUE))

        if position_source == PositionSource.CURRENT:
            return 0

        if position_source == PositionSource.USER:
            if position is None:
                raise ValueError("position is required when position_source=USER")
            if abs(position) > 0x0FFFFFFF:
                raise ValueError("position must fit in a signed 32-bit EPOS4 home-position value")
            return int(position)

        raise NotImplementedError(f"Unsupported position source: {position_source}")

    # PDODevice implementation
    def get_pdo_outputs(self) -> Mapping[str, Any]:
        with self._pdo_lock:
            return dict(self._pdo_outputs)

    def set_pdo_outputs(self, **values: Any) -> None:
        with self._pdo_lock:
            unknown = set(values) - set(self.rx_pdo.names)
            if unknown:
                raise KeyError(f"Unknown EPOS4 output PDO field(s): {sorted(unknown)}")
            self._pdo_outputs.update(values)

    def set_pdo_inputs(self, values: Mapping[str, Any]) -> None:
        with self._pdo_lock:
            self._pdo_inputs.update(values)

    def get_pdo_inputs(self) -> dict[str, Any]:
        with self._pdo_lock:
            return dict(self._pdo_inputs)

    def pdo_input(self, name: str) -> Any:
        with self._pdo_lock:
            return self._pdo_inputs[name]

    def pdo_output(self, name: str) -> Any:
        with self._pdo_lock:
            return self._pdo_outputs[name]

    @property
    def statusword_pdo(self) -> Statusword:
        return Statusword(int(self.pdo_input("statusword")))

    @property
    def position_pdo(self) -> int:
        return int(self.pdo_input("position_actual_value"))

    @property
    def velocity_pdo(self) -> int:
        return int(self.pdo_input("velocity_actual_value"))

    @property
    def following_error_pdo(self) -> int:
        return int(self.pdo_input("following_error_actual_value"))

    @property
    def moving_pdo(self) -> bool:
        return StatuswordBit.TARGET_REACHED not in self.statusword_pdo

    def set_controlword_pdo(self, command: int | IntEnum) -> None:
        value = int(command.value) if isinstance(command, IntEnum) else int(command)
        self.set_pdo_outputs(controlword=value)

    def set_operating_mode_pdo(self, mode: OperatingMode) -> None:
        self.set_pdo_outputs(modes_of_operation=int(mode))

    def configure_profile_position_pdo(
        self,
        *,
        velocity: int,
        acceleration: int,
        deceleration: int | None = None,
    ) -> None:
        deceleration = acceleration if deceleration is None else deceleration
        self.set_pdo_outputs(
            profile_velocity=abs(int(velocity)),
            profile_acceleration=abs(int(acceleration)),
            profile_deceleration=abs(int(deceleration)),
            modes_of_operation=int(OperatingMode.PROFILE_POSITION),
        )

    def queue_profile_position_pdo(
        self,
        position: int,
        *,
        velocity: int,
        acceleration: int,
        deceleration: int | None = None,
        absolute: bool = True,
    ) -> None:
        """
        Queue a profile-position PDO command.

        PDOLoop will pick this up on the next cycle. This method does not block
        and does not check that the drive is already operation-enabled.
        """
        self.configure_profile_position_pdo(
            velocity=velocity,
            acceleration=acceleration,
            deceleration=deceleration,
        )
        command = (
            MotionCommand.ABSOLUTE_START_IMMEDIATELY
            if absolute
            else MotionCommand.RELATIVE_START_IMMEDIATELY
        )
        self.set_pdo_outputs(
            controlword=int(command),
            target_position=int(position),
        )

    def queue_quick_stop_pdo(self) -> None:
        self.set_controlword_pdo(ControlwordCommand.QUICK_STOP)

    def queue_shutdown_pdo(self) -> None:
        self.set_controlword_pdo(ControlwordCommand.SHUTDOWN)

    def queue_enable_pdo(self) -> None:
        self.set_controlword_pdo(ControlwordCommand.SWITCH_ON_AND_ENABLE_OPERATION)
