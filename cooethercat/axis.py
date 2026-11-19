"""
Backend-neutral axis API.

The field tables here are the interface contract. Every backend answers the same
field names and every consumer, whether the CLI, the libby adapter or a test,
goes through them. A backend translates a field name into whatever its master
understands: a PLC symbol for TwinCAT, a CiA 402 object or PDO slot for pysoem.

Two things stay out of the tables on purpose. Backend-specific addressing lives
in the backend, so a PLC symbol path never appears here. Unit strings come from
the axis config, so a field declares only its dimension and ``mm`` versus ``deg``
is a per-topology fact.

An Axis always speaks engineering units; backends that command in encoder
increments convert using the axis's configured scaling.
"""

from __future__ import annotations

import enum
import time
from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
from typing import Any, Final, Protocol, Self

from .config import AxisConfig


class AxisError(RuntimeError):
    """Raised when a drive reports a fault or a command does not complete."""


class AxisState(enum.IntEnum):
    """PLCopen axis state model, which every backend reports in these terms."""

    DISABLED = 0
    STANDSTILL = 1
    HOMING = 2
    DISCRETE_MOTION = 3
    CONTINUOUS_MOTION = 4
    STOPPING = 5
    ERROR_STOP = 6


class MoveMode(enum.IntEnum):
    """How a commanded target is interpreted."""

    ABSOLUTE = 0
    RELATIVE = 1
    VELOCITY = 2


class CoeType(enum.Enum):
    """Width and signedness of a CoE object; an oversized read turns negatives positive."""

    INT8 = "int8"
    UINT8 = "uint8"
    INT16 = "int16"
    UINT16 = "uint16"
    INT32 = "int32"
    UINT32 = "uint32"


class ControllerState(enum.StrEnum):
    """Run state of the motion controller behind a backend."""

    RUN = "run"
    STOP = "stop"
    CONFIG = "config"
    OTHER = "other"


class FieldKind(enum.StrEnum):
    """Value type of an axis field, mirroring libby's typed keyword classes."""

    BOOL = "bool"
    INT = "int"
    FLOAT = "float"
    TRIGGER = "trigger"


class Dimension(enum.StrEnum):
    """Physical dimension of a field, resolved against an axis's unit name."""

    NONE = "none"
    POSITION = "position"
    VELOCITY = "velocity"
    ACCELERATION = "acceleration"


@dataclass(frozen=True)
class AxisField:
    """One field of the axis interface contract."""

    name: str
    kind: FieldKind
    dimension: Dimension
    description: str

    def unit_label(self, unit_name: str) -> str | None:
        """Return this field's unit string for an axis measured in ``unit_name``."""
        match self.dimension:
            case Dimension.POSITION:
                return unit_name
            case Dimension.VELOCITY:
                return f"{unit_name}/s"
            case Dimension.ACCELERATION:
                return f"{unit_name}/s^2"
            case Dimension.NONE:
                return None


def _field(
    name: str,
    kind: FieldKind,
    dimension: Dimension,
    description: str,
) -> tuple[str, AxisField]:
    return name, AxisField(name, kind, dimension, description)


# Values the caller owns; a backend latches them when a trigger fires.
SETTINGS: Final[dict[str, AxisField]] = dict(
    [
        _field("enable", FieldKind.BOOL, Dimension.NONE,
               "Hold true to keep the drive energised"),
        _field("mode", FieldKind.INT, Dimension.NONE,
               "Move mode, see MoveMode"),
        _field("target", FieldKind.FLOAT, Dimension.POSITION,
               "Absolute position, or distance when the mode is relative"),
        _field("velocity", FieldKind.FLOAT, Dimension.VELOCITY,
               "Commanded profile velocity"),
        _field("acceleration", FieldKind.FLOAT, Dimension.ACCELERATION,
               "Commanded profile acceleration"),
        _field("deceleration", FieldKind.FLOAT, Dimension.ACCELERATION,
               "Commanded profile deceleration"),
    ]
)

# Write-only actions carrying no value.
TRIGGERS: Final[dict[str, AxisField]] = dict(
    [
        _field("move", FieldKind.TRIGGER, Dimension.NONE,
               "Start the move described by target, velocity and mode"),
        _field("stop", FieldKind.TRIGGER, Dimension.NONE,
               "Decelerate to standstill"),
        _field("reset", FieldKind.TRIGGER, Dimension.NONE,
               "Clear a latched fault"),
        _field("home", FieldKind.TRIGGER, Dimension.NONE,
               "Run the homing sequence"),
    ]
)

# Readbacks the drive or its master owns.
STATUS: Final[dict[str, AxisField]] = dict(
    [
        _field("position", FieldKind.FLOAT, Dimension.POSITION,
               "Actual position"),
        _field("actual_velocity", FieldKind.FLOAT, Dimension.VELOCITY,
               "Actual velocity"),
        _field("state", FieldKind.INT, Dimension.NONE,
               "PLCopen axis state, see AxisState"),
        _field("moving", FieldKind.BOOL, Dimension.NONE,
               "Axis is in motion"),
        _field("enabled", FieldKind.BOOL, Dimension.NONE,
               "Drive is energised"),
        _field("ready", FieldKind.BOOL, Dimension.NONE,
               "Ready to accept a command"),
        _field("error", FieldKind.BOOL, Dimension.NONE,
               "Fault is latched"),
        _field("error_id", FieldKind.INT, Dimension.NONE,
               "Backend-specific error identifier"),
        _field("sequence", FieldKind.INT, Dimension.NONE,
               "Increments on each completed command"),
    ]
)

ALL_FIELDS: Final[dict[str, AxisField]] = {**SETTINGS, **TRIGGERS, **STATUS}

_COMPLETION_POLL_S: Final = 0.02


class AxisBackend(Protocol):
    """
    Transport a set of axes is driven through.

    Field access is per axis and per field name rather than per symbol string,
    because only some masters have a symbol namespace to address.
    """

    def open(self) -> None:
        """Bring the transport up."""

    def close(self) -> None:
        """Take the transport down, leaving drive state to the master."""

    def read_field(self, axis: AxisConfig, field: str) -> Any:
        """Return one field's current value in engineering units."""

    def read_fields(self, axis: AxisConfig, fields: Sequence[str]) -> dict[str, Any]:
        """
        Return several fields as one coherent snapshot.

        Backends whose transport charges per round trip batch here, which for a
        24-axis topology is the difference between one request and hundreds.
        """

    def read_axes_fields(
        self, axes: Sequence[AxisConfig], fields: Sequence[str]
    ) -> dict[str, dict[str, Any]]:
        """Return the same fields for several axes as one snapshot, keyed by axis name."""

    def write_field(self, axis: AxisConfig, field: str, value: Any) -> None:
        """Set one settable field, in engineering units."""

    def fire_trigger(self, axis: AxisConfig, field: str) -> None:
        """Perform one action, handling whatever edge the master needs."""

    def read_object(self, axis: AxisConfig, index: int, subindex: int, kind: CoeType) -> int:
        """Read one CoE object from the axis's drive."""

    def controller_state(self) -> ControllerState:
        """Return whether the motion controller is running."""


class Axis:
    """One commandable axis, in engineering units, over any backend."""

    def __init__(self, backend: AxisBackend, config: AxisConfig) -> None:
        self.backend = backend
        self.config = config

    def __repr__(self) -> str:
        return f"Axis(name={self.name!r}, units={self.config.units.name!r})"

    @property
    def name(self) -> str:
        """Return the configured axis name."""
        return self.config.name

    def get(self, field: str) -> Any:
        """Read one interface field, which backs the libby getters."""
        spec = self._spec(field)
        if spec.kind is FieldKind.TRIGGER:
            raise AxisError(f"{self.name}: {field!r} is an action and cannot be read")
        return self.backend.read_field(self.config, field)

    def set(self, field: str, value: Any) -> None:
        """Write one interface field, which backs the libby setters."""
        if field not in SETTINGS:
            self._spec(field)
            raise AxisError(f"{self.name}: {field!r} is not settable")
        self.backend.write_field(self.config, field, value)

    def _spec(self, field: str) -> AxisField:
        try:
            return ALL_FIELDS[field]
        except KeyError:
            raise AxisError(f"{self.name}: unknown field {field!r}") from None

    @property
    def position(self) -> float:
        return float(self.get("position"))

    @property
    def velocity(self) -> float:
        return float(self.get("actual_velocity"))

    @property
    def state(self) -> AxisState:
        return AxisState(self.get("state"))

    @property
    def moving(self) -> bool:
        return bool(self.get("moving"))

    @property
    def enabled(self) -> bool:
        return bool(self.get("enabled"))

    @property
    def ready(self) -> bool:
        return bool(self.get("ready"))

    @property
    def error(self) -> bool:
        return bool(self.get("error"))

    @property
    def error_id(self) -> int:
        return int(self.get("error_id"))

    def read_object(self, index: int, subindex: int, kind: CoeType) -> int:
        """Read one CoE object from this axis's drive."""
        return self.backend.read_object(self.config, index, subindex, kind)

    def status(self) -> dict[str, Any]:
        """Return every readback as one coherent snapshot."""
        return self.backend.read_fields(self.config, tuple(STATUS))

    def enable(self, *, timeout_s: float = 5.0) -> None:
        """Energise the drive and wait for it to report enabled."""
        self.set("enable", True)
        self._wait_until(lambda: self.enabled, timeout_s, "enable")

    def disable(self, *, timeout_s: float = 5.0) -> None:
        """De-energise the drive and wait for it to report disabled."""
        self.set("enable", False)
        self._wait_until(lambda: not self.enabled, timeout_s, "disable")

    def move(self) -> None:
        """Start the move described by the current target, velocity and mode."""
        self.backend.fire_trigger(self.config, "move")

    def stop(self) -> None:
        """Decelerate to standstill."""
        self.backend.fire_trigger(self.config, "stop")

    def reset(self) -> None:
        """Clear a latched fault."""
        self.backend.fire_trigger(self.config, "reset")

    def home(self) -> None:
        """Run the homing sequence."""
        self.backend.fire_trigger(self.config, "home")

    def move_absolute(
        self,
        target: float,
        *,
        velocity: float | None = None,
        wait: bool = True,
        timeout_s: float = 60.0,
    ) -> float | None:
        """Move to an absolute position, by default blocking until it completes."""
        return self._move(MoveMode.ABSOLUTE, target, velocity, wait, timeout_s)

    def move_relative(
        self,
        distance: float,
        *,
        velocity: float | None = None,
        wait: bool = True,
        timeout_s: float = 60.0,
    ) -> float | None:
        """Move by a distance, by default blocking until it completes."""
        return self._move(MoveMode.RELATIVE, distance, velocity, wait, timeout_s)

    def _move(
        self,
        mode: MoveMode,
        target: float,
        velocity: float | None,
        wait: bool,
        timeout_s: float,
    ) -> float | None:
        if self.error:
            raise AxisError(
                f"{self.name}: axis is in fault {self.error_id}, reset before moving"
            )
        if not self.enabled:
            raise AxisError(f"{self.name}: axis is not enabled")

        self._check_travel(mode, target)
        if velocity is not None:
            self._check_velocity(velocity)

        sequence_before = self.get("sequence")
        if velocity is not None:
            self.set("velocity", float(velocity))
        self.set("mode", int(mode))
        self.set("target", float(target))
        self.move()

        if not wait:
            return None
        return self.wait(sequence_before=sequence_before, timeout_s=timeout_s)

    def wait(
        self,
        *,
        sequence_before: int | None = None,
        timeout_s: float = 60.0,
    ) -> float:
        """Block until the in-flight move completes, then return the position."""
        if sequence_before is None:
            sequence_before = self.get("sequence")

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.error:
                raise AxisError(f"{self.name}: fault {self.error_id} during move")
            if self.get("sequence") != sequence_before and not self.moving:
                return self.position
            time.sleep(_COMPLETION_POLL_S)

        raise AxisError(f"{self.name}: move did not complete within {timeout_s} s")

    def _check_velocity(self, velocity: float) -> None:
        allowed = self.config.limits.velocity
        if abs(velocity) > allowed:
            raise AxisError(
                f"{self.name}: velocity {velocity} exceeds the configured limit "
                f"{allowed} {self.config.units.name}/s"
            )

    def _check_travel(self, mode: MoveMode, target: float) -> None:
        if mode is MoveMode.VELOCITY:
            return

        absolute = target if mode is MoveMode.ABSOLUTE else self.position + target
        if not self.config.limits.position_is_allowed(absolute):
            limits = self.config.limits
            raise AxisError(
                f"{self.name}: target {absolute} {self.config.units.name} is outside "
                f"the configured travel [{limits.min_position}, {limits.max_position}]"
            )

    def _wait_until(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        action: str,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(_COMPLETION_POLL_S)
        raise AxisError(f"{self.name}: {action} timed out after {timeout_s} s")


class AxisSet:
    """Every axis in one topology, sharing a backend."""

    def __init__(self, backend: AxisBackend, axes: dict[str, Axis]) -> None:
        self.backend = backend
        self._axes = axes

    def __enter__(self) -> Self:
        self.backend.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.backend.close()

    def __len__(self) -> int:
        return len(self._axes)

    def __iter__(self) -> Iterator[Axis]:
        return iter(self._axes.values())

    def read(self, fields: Sequence[str] = tuple(STATUS)) -> dict[str, dict[str, Any]]:
        """Return the fields of every axis as one snapshot, keyed by axis name."""
        return self.backend.read_axes_fields([axis.config for axis in self], fields)

    @property
    def names(self) -> tuple[str, ...]:
        """Return configured axis names in file order."""
        return tuple(axis.name for axis in self._axes.values())

    def axis(self, name: str) -> Axis:
        """Return one axis by name, case-insensitively."""
        try:
            return self._axes[name.lower()]
        except KeyError:
            raise AxisError(
                f"no axis {name!r}; known axes: {', '.join(self.names)}"
            ) from None
