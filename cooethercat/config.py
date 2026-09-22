"""
Topology configuration.

One YAML file describes one EtherCAT topology: which master backend drives it,
how to reach that backend, and which axes it carries. Parsing is strict, so a
config that loads is one the backends can consume without re-validating it.

Axis identifiers stay separate because on the CSU bench they disagree. A pysoem
master addresses a drive by enumeration order (``slave_index``), CoE over ADS by
EtherCAT station address (``station_address``), and a TwinCAT PLC motion server
by its slot in the symbol array (``plc_index``).
"""

from __future__ import annotations

import enum
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Final

import yaml


class ConfigError(ValueError):
    """Raised when a topology configuration is malformed or self-inconsistent."""


class Backend(enum.StrEnum):
    """Master implementation that drives a topology."""

    PYSOEM = "pysoem"
    TWINCAT = "twincat"
    SIM = "sim"


@dataclass(frozen=True)
class PysoemTransport:
    """Connection details for a pysoem soft master on a dedicated NIC."""

    ifname: str


@dataclass(frozen=True)
class TwincatTransport:
    """Connection details for a TwinCAT master reached over ADS."""

    host: str
    ams_net_id: str
    ams_port: int = 851
    master_net_id: str | None = None
    local_net_id: str | None = None


@dataclass(frozen=True)
class SimTransport:
    """Transport for the in-process simulator, which reaches no hardware."""

    tick_s: float = 0.01


Transport = PysoemTransport | TwincatTransport | SimTransport


@dataclass(frozen=True)
class Units:
    """
    An axis's engineering units and its encoder scaling.

    A TwinCAT NC axis is scaled in the PLC project and reports engineering units
    already, so ``increments_per_unit`` is optional and only required where the
    host itself speaks increments.
    """

    name: str
    increments_per_unit: float | None = None

    def require_scaling(self) -> float:
        """Return increments per unit, raising if the config omits the scaling."""
        if self.increments_per_unit is None:
            raise ConfigError(f"units {self.name!r} have no increments_per_unit")
        return self.increments_per_unit

    def to_increments(self, value: float) -> int:
        """Convert an engineering-unit value to whole encoder increments."""
        return round(value * self.require_scaling())

    def to_units(self, increments: float) -> float:
        """Convert encoder increments to engineering units."""
        return increments / self.require_scaling()


@dataclass(frozen=True)
class Limits:
    """
    Motion limits checked before a move is commanded.

    Only ``velocity`` is mandatory, so every axis has a speed ceiling. An unset
    acceleration leaves the choice to the backend, which for a TwinCAT NC axis
    means the axis's own configured default.
    """

    velocity: float
    acceleration: float | None = None
    deceleration: float | None = None
    min_position: float | None = None
    max_position: float | None = None

    @property
    def effective_deceleration(self) -> float | None:
        """Return the deceleration to use, falling back to the acceleration limit."""
        return self.acceleration if self.deceleration is None else self.deceleration

    def position_is_allowed(self, position: float) -> bool:
        """Return True when a target lies inside the configured travel."""
        below = self.min_position is not None and position < self.min_position
        above = self.max_position is not None and position > self.max_position
        return not (below or above)


@dataclass(frozen=True)
class AxisConfig:
    """One commandable axis and the addresses each backend needs to reach it."""

    name: str
    units: Units
    limits: Limits
    device: str = "maxon_epos4"
    pdo_profile: str = "profile_position"
    slave_index: int | None = None
    station_address: int | None = None
    plc_index: int | None = None

    @property
    def keyword_name(self) -> str:
        """Return the lowercase name used to build libby keywords."""
        return self.name.lower()

    def require_station_address(self) -> int:
        """Return the station address, raising if CoE object access cannot work."""
        if self.station_address is None:
            raise ConfigError(
                f"axis {self.name!r} has no station_address, which CoE object "
                f"access requires"
            )
        return self.station_address


@dataclass(frozen=True)
class TopologyConfig:
    """A complete EtherCAT topology: one backend, one transport, many axes."""

    name: str
    backend: Backend
    transport: Transport
    axes: tuple[AxisConfig, ...]

    @property
    def axis_names(self) -> tuple[str, ...]:
        """Return configured axis names in file order."""
        return tuple(axis.name for axis in self.axes)

    def axis(self, name: str) -> AxisConfig:
        """Return one axis by name, case-insensitively."""
        wanted = name.lower()
        for axis in self.axes:
            if axis.keyword_name == wanted:
                return axis
        raise ConfigError(
            f"topology {self.name!r} has no axis {name!r}; "
            f"known axes: {', '.join(self.axis_names)}"
        )


_TOPOLOGY_KEYS: Final = ("name", "backend", "transport", "defaults", "axes")
_AXIS_KEYS: Final = (
    "name",
    "device",
    "pdo_profile",
    "units",
    "limits",
    "slave_index",
    "station_address",
    "plc_index",
)
_DEFAULTS_KEYS: Final = ("device", "pdo_profile", "units", "limits")
_UNITS_KEYS: Final = ("name", "increments_per_unit")
_LIMITS_KEYS: Final = (
    "velocity",
    "acceleration",
    "deceleration",
    "min_position",
    "max_position",
)

_TRANSPORT_KEYS: Final[dict[Backend, tuple[str, ...]]] = {
    Backend.PYSOEM: ("ifname",),
    Backend.TWINCAT: ("host", "ams_net_id", "ams_port", "master_net_id", "local_net_id"),
    Backend.SIM: ("tick_s",),
}
_TRANSPORT_REQUIRED: Final[dict[Backend, tuple[str, ...]]] = {
    Backend.PYSOEM: ("ifname",),
    Backend.TWINCAT: ("host", "ams_net_id"),
    Backend.SIM: (),
}

# Address fields each backend needs before it can command an axis.
_AXIS_REQUIRED: Final[dict[Backend, tuple[str, ...]]] = {
    Backend.PYSOEM: ("slave_index",),
    Backend.TWINCAT: ("plc_index",),
    Backend.SIM: (),
}

_UNIQUE_AXIS_FIELDS: Final = ("slave_index", "station_address", "plc_index")

# Axis names become dotted libby keyword components and wildcard targets.
_NAME_FORBIDDEN: Final = (".", " ", "\t", "%")


def load_topology(path: str | Path) -> TopologyConfig:
    """Load and validate a topology from a YAML file."""
    source = Path(path)
    try:
        text = source.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read topology file {source}: {exc}") from exc

    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f"{source}: invalid YAML: {exc}") from exc

    return parse_topology(_mapping(data, str(source)), source=str(source))


def parse_topology(
    data: Mapping[str, Any],
    *,
    source: str = "<mapping>",
) -> TopologyConfig:
    """Validate an already-parsed mapping and return a topology."""
    _reject_unknown(data, _TOPOLOGY_KEYS, source)

    backend = _parse_backend(_require(data, "backend", source), source)
    defaults = _submapping(data, "defaults", source)
    _reject_unknown(defaults, _DEFAULTS_KEYS, f"{source}: defaults")

    raw_axes = _require(data, "axes", source)
    if not isinstance(raw_axes, list) or not raw_axes:
        raise ConfigError(f"{source}: axes must be a non-empty list")

    axes = tuple(
        # An axis entry overrides a default key outright rather than merging into it.
        _parse_axis({**defaults, **_mapping(entry, f"{source}: axes[{position}]")},
                    backend, f"{source}: axes[{position}]")
        for position, entry in enumerate(raw_axes)
    )
    _check_collisions(axes, source)

    return TopologyConfig(
        name=_text(_require(data, "name", source), f"{source}.name"),
        backend=backend,
        transport=_parse_transport(_submapping(data, "transport", source), backend, source),
        axes=axes,
    )


def _parse_backend(value: Any, source: str) -> Backend:
    raw = _text(value, f"{source}.backend")
    try:
        return Backend(raw)
    except ValueError as exc:
        known = ", ".join(member.value for member in Backend)
        raise ConfigError(f"{source}: unknown backend {raw!r}; expected one of {known}") from exc


def _parse_transport(data: Mapping[str, Any], backend: Backend, source: str) -> Transport:
    where = f"{source}: transport"
    _reject_unknown(data, _TRANSPORT_KEYS[backend], where)

    missing = [key for key in _TRANSPORT_REQUIRED[backend] if data.get(key) is None]
    if missing:
        raise ConfigError(
            f"{where}: backend {backend.value!r} requires {', '.join(sorted(missing))}"
        )

    match backend:
        case Backend.PYSOEM:
            return PysoemTransport(ifname=_text(data["ifname"], f"{where}.ifname"))
        case Backend.TWINCAT:
            return TwincatTransport(
                host=_text(data["host"], f"{where}.host"),
                ams_net_id=_text(data["ams_net_id"], f"{where}.ams_net_id"),
                ams_port=_whole(data.get("ams_port", 851), f"{where}.ams_port", minimum=1),
                master_net_id=_opt_text(data.get("master_net_id"),
                                        f"{where}.master_net_id"),
                local_net_id=_opt_text(data.get("local_net_id"),
                                       f"{where}.local_net_id"),
            )
        case Backend.SIM:
            return SimTransport(
                tick_s=_number(data.get("tick_s", 0.01), f"{where}.tick_s", greater_than=0)
            )


def _parse_axis(data: Mapping[str, Any], backend: Backend, where: str) -> AxisConfig:
    _reject_unknown(data, _AXIS_KEYS, where)

    name = _parse_axis_name(_require(data, "name", where), where)
    axis = AxisConfig(
        name=name,
        units=_parse_units(_require(data, "units", where), where),
        limits=_parse_limits(_require(data, "limits", where), where),
        device=_text(data.get("device", "maxon_epos4"), f"{where}.device"),
        pdo_profile=_text(data.get("pdo_profile", "profile_position"),
                          f"{where}.pdo_profile"),
        slave_index=_opt_whole(data.get("slave_index"), f"{where}.slave_index", minimum=0),
        station_address=_opt_whole(data.get("station_address"),
                                   f"{where}.station_address", minimum=1),
        plc_index=_opt_whole(data.get("plc_index"), f"{where}.plc_index", minimum=1),
    )

    missing = [field for field in _AXIS_REQUIRED[backend] if getattr(axis, field) is None]
    if missing:
        raise ConfigError(
            f"{where}: axis {name!r} needs {', '.join(missing)} for backend "
            f"{backend.value!r}"
        )
    if backend is Backend.PYSOEM and axis.units.increments_per_unit is None:
        raise ConfigError(
            f"{where}: axis {name!r} needs units.increments_per_unit for backend "
            f"'pysoem', which commands in encoder increments"
        )
    return axis


def _parse_axis_name(value: Any, where: str) -> str:
    name = _text(value, f"{where}.name")
    if not name:
        raise ConfigError(f"{where}: axis name must not be empty")

    found = [token for token in _NAME_FORBIDDEN if token in name]
    if found:
        raise ConfigError(
            f"{where}: axis name {name!r} must not contain {found!r}, because names "
            f"become dotted libby keyword components"
        )
    return name


def _parse_units(value: Any, where: str) -> Units:
    data = _mapping(value, f"{where}.units")
    _reject_unknown(data, _UNITS_KEYS, f"{where}.units")
    return Units(
        name=_text(_require(data, "name", f"{where}.units"), f"{where}.units.name"),
        increments_per_unit=_opt_number(
            data.get("increments_per_unit"),
            f"{where}.units.increments_per_unit", greater_than=0,
        ),
    )


def _parse_limits(value: Any, where: str) -> Limits:
    data = _mapping(value, f"{where}.limits")
    _reject_unknown(data, _LIMITS_KEYS, f"{where}.limits")

    limits = Limits(
        velocity=_number(_require(data, "velocity", f"{where}.limits"),
                         f"{where}.limits.velocity", greater_than=0),
        acceleration=_opt_number(data.get("acceleration"),
                                 f"{where}.limits.acceleration", greater_than=0),
        deceleration=_opt_number(data.get("deceleration"),
                                 f"{where}.limits.deceleration", greater_than=0),
        min_position=_opt_number(data.get("min_position"), f"{where}.limits.min_position"),
        max_position=_opt_number(data.get("max_position"), f"{where}.limits.max_position"),
    )
    if (
        limits.min_position is not None
        and limits.max_position is not None
        and limits.min_position >= limits.max_position
    ):
        raise ConfigError(
            f"{where}.limits: min_position {limits.min_position} must be below "
            f"max_position {limits.max_position}"
        )
    return limits


def _check_collisions(axes: tuple[AxisConfig, ...], source: str) -> None:
    names: dict[str, str] = {}
    for axis in axes:
        clash = names.setdefault(axis.keyword_name, axis.name)
        if clash != axis.name:
            raise ConfigError(
                f"{source}: axis names {clash!r} and {axis.name!r} collide once "
                f"lowercased for keywords"
            )

    for field in _UNIQUE_AXIS_FIELDS:
        owners: dict[int, str] = {}
        for axis in axes:
            value = getattr(axis, field)
            if value is None:
                continue
            owner = owners.setdefault(value, axis.name)
            if owner != axis.name:
                raise ConfigError(
                    f"{source}: axes {owner!r} and {axis.name!r} share {field}={value}"
                )


def _require(data: Mapping[str, Any], key: str, where: str) -> Any:
    if key not in data or data[key] is None:
        raise ConfigError(f"{where}: missing required key {key!r}")
    return data[key]


def _submapping(data: Mapping[str, Any], key: str, where: str) -> Mapping[str, Any]:
    if key not in data or data[key] is None:
        return {}
    return _mapping(data[key], f"{where}.{key}")


def _reject_unknown(data: Mapping[str, Any], allowed: tuple[str, ...], where: str) -> None:
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise ConfigError(
            f"{where}: unknown key(s) {', '.join(repr(key) for key in unknown)}; "
            f"allowed: {', '.join(allowed)}"
        )


def _mapping(value: Any, where: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigError(f"{where}: expected a mapping, got {type(value).__name__}")
    return value


def _text(value: Any, where: str) -> str:
    if not isinstance(value, str):
        raise ConfigError(f"{where}: expected a string, got {type(value).__name__}")
    return value


def _number(value: Any, where: str, *, greater_than: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{where}: expected a number, got {type(value).__name__}")
    if greater_than is not None and value <= greater_than:
        raise ConfigError(f"{where}: expected a number above {greater_than}, got {value}")
    return float(value)


def _whole(value: Any, where: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{where}: expected an integer, got {type(value).__name__}")
    if value < minimum:
        raise ConfigError(f"{where}: expected an integer at least {minimum}, got {value}")
    return value


def _opt_text(value: Any, where: str) -> str | None:
    return None if value is None else _text(value, where)


def _opt_number(
    value: Any,
    where: str,
    *,
    greater_than: float | None = None,
) -> float | None:
    return None if value is None else _number(value, where, greater_than=greater_than)


def _opt_whole(value: Any, where: str, *, minimum: int = 0) -> int | None:
    return None if value is None else _whole(value, where, minimum=minimum)
