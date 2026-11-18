"""
TwinCAT axis backend.

Drives axes through a motion server already deployed on a TwinCAT PLC, over ADS
named symbols. Each axis is one ``GVL_Axes.Axis[plc_index]`` struct: Python
writes the ``Cmd`` half and reads the ``Sts`` half, while the PLC owns the
PLCopen state machine, fault reaction and limits. Nothing here sits in the
control loop, so a crash or a lost connection on this side cannot perturb an
axis in motion.

The NC axis behind each slot is scaled in the PLC project, so values cross ADS
in engineering units unchanged.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, Final, Protocol

import pyads

from ..axis import Axis, AxisSet, CoeType, ControllerState
from ..config import AxisConfig, TopologyConfig, TwincatTransport

SYMBOL_ROOT: Final = "GVL_Axes"

# The PLC latches a trigger on the rising edge; holding it this long spans
# several PLC cycles at any realistic task period.
TRIGGER_PULSE_S: Final = 0.05


@dataclass(frozen=True)
class _Symbol:
    """Where one interface field lives in the axis struct, and its PLC type."""

    member: str
    plc_type: Any


SYMBOLS: Final[dict[str, _Symbol]] = {
    "enable": _Symbol("Cmd.bEnable", pyads.PLCTYPE_BOOL),
    "mode": _Symbol("Cmd.eMode", pyads.PLCTYPE_DINT),
    "target": _Symbol("Cmd.fTarget", pyads.PLCTYPE_LREAL),
    "velocity": _Symbol("Cmd.fVelocity", pyads.PLCTYPE_LREAL),
    "acceleration": _Symbol("Cmd.fAcceleration", pyads.PLCTYPE_LREAL),
    "deceleration": _Symbol("Cmd.fDeceleration", pyads.PLCTYPE_LREAL),
    "move": _Symbol("Cmd.bExecute", pyads.PLCTYPE_BOOL),
    "stop": _Symbol("Cmd.bStop", pyads.PLCTYPE_BOOL),
    "reset": _Symbol("Cmd.bReset", pyads.PLCTYPE_BOOL),
    "home": _Symbol("Cmd.bHome", pyads.PLCTYPE_BOOL),
    "position": _Symbol("Sts.fPosition", pyads.PLCTYPE_LREAL),
    "actual_velocity": _Symbol("Sts.fVelocity", pyads.PLCTYPE_LREAL),
    "state": _Symbol("Sts.eState", pyads.PLCTYPE_DINT),
    "moving": _Symbol("Sts.bMoving", pyads.PLCTYPE_BOOL),
    "enabled": _Symbol("Sts.bEnabled", pyads.PLCTYPE_BOOL),
    "ready": _Symbol("Sts.bReady", pyads.PLCTYPE_BOOL),
    "error": _Symbol("Sts.bError", pyads.PLCTYPE_BOOL),
    "error_id": _Symbol("Sts.nErrorId", pyads.PLCTYPE_UDINT),
    "sequence": _Symbol("Sts.nSeq", pyads.PLCTYPE_UDINT),
}

_COE_TYPES: Final[dict[CoeType, Any]] = {
    CoeType.INT8: pyads.PLCTYPE_SINT,
    CoeType.UINT8: pyads.PLCTYPE_USINT,
    CoeType.INT16: pyads.PLCTYPE_INT,
    CoeType.UINT16: pyads.PLCTYPE_UINT,
    CoeType.INT32: pyads.PLCTYPE_DINT,
    CoeType.UINT32: pyads.PLCTYPE_UDINT,
}

# CoE SDO access through the EtherCAT master: offset is (index << 16) | subindex
COE_INDEX_GROUP: Final = 0xF302

# The TwinCAT system service, whose ADS state is the runtime's Run/Config mode
SYSTEM_PORT: Final = 10000
_CONTROLLER_STATES: Final = {
    pyads.ADSSTATE_RUN: ControllerState.RUN,
    pyads.ADSSTATE_STOP: ControllerState.STOP,
    pyads.ADSSTATE_CONFIG: ControllerState.CONFIG,
    pyads.ADSSTATE_RECONFIG: ControllerState.CONFIG,
}

_SETTABLE: Final = frozenset({"enable", "mode", "target", "velocity", "acceleration", "deceleration"})
_TRIGGERS: Final = frozenset({"move", "stop", "reset", "home"})


class AdsConnection(Protocol):
    """The slice of ``pyads.Connection`` this backend uses."""

    def open(self) -> None:
        """Open the ADS port."""

    def close(self) -> None:
        """Close the ADS port."""

    def read_by_name(self, data_name: str, plc_datatype: Any = None) -> Any:
        """Read one symbol."""

    def write_by_name(self, data_name: str, value: Any, plc_datatype: Any = None) -> None:
        """Write one symbol."""

    def read_list_by_name(self, data_names: list[str]) -> dict[str, Any]:
        """Read several symbols in one sum request."""


class AdsDevice(Protocol):
    """The slice of ``pyads.Connection`` used for raw index-group reads."""

    def open(self) -> None:
        """Open the ADS port."""

    def close(self) -> None:
        """Close the ADS port."""

    def read(self, index_group: int, index_offset: int, plc_datatype: Any) -> Any:
        """Read one value by index group and offset."""


DeviceFactory = Callable[[int], AdsDevice]


class AdsStateReader(Protocol):
    """The slice of ``pyads.Connection`` used to read a service's ADS state."""

    def open(self) -> None:
        """Open the ADS port."""

    def close(self) -> None:
        """Close the ADS port."""

    def read_state(self) -> tuple[int, int] | None:
        """Return the ADS state and device state."""


def symbol_name(axis: AxisConfig, field: str) -> str:
    """Return the PLC symbol carrying one field of one axis."""
    if axis.plc_index is None:
        raise KeyError(f"axis {axis.name!r} has no plc_index")
    return f"{SYMBOL_ROOT}.Axis[{axis.plc_index}].{SYMBOLS[field].member}"


class TwincatBackend:
    """Axis backend speaking to a TwinCAT PLC motion server over ADS."""

    def __init__(
        self,
        connection: AdsConnection,
        drives: DeviceFactory | None = None,
        register_routes: Callable[[], None] | None = None,
        system: AdsStateReader | None = None,
    ) -> None:
        self._plc = connection
        self._system = system
        self._drive_factory = drives
        self._register_routes = register_routes
        self._drives: dict[int, AdsDevice] = {}
        # One ADS port fails intermittently under concurrent requests from several threads
        self._lock = threading.Lock()

    def open(self) -> None:
        with self._lock:
            if self._register_routes is not None:
                self._register_routes()
            self._plc.open()
            if self._system is not None:
                self._system.open()

    def close(self) -> None:
        with self._lock:
            for drive in self._drives.values():
                drive.close()
            self._drives.clear()
            self._plc.close()
            if self._system is not None:
                self._system.close()

    def controller_state(self) -> ControllerState:
        if self._system is None:
            raise KeyError("this backend has no TwinCAT system connection")
        with self._lock:
            state = self._system.read_state()
        return _CONTROLLER_STATES.get(state[0], ControllerState.OTHER) if state else ControllerState.OTHER

    def read_object(self, axis: AxisConfig, index: int, subindex: int, kind: CoeType) -> int:
        """Read one CoE object from the axis's drive."""
        if self._drive_factory is None:
            raise KeyError("this topology has no master_net_id, so CoE objects are unreachable")
        station = axis.require_station_address()
        with self._lock:
            drive = self._drives.get(station)
            if drive is None:
                drive = self._drive_factory(station)
                drive.open()
                self._drives[station] = drive
            return int(drive.read(COE_INDEX_GROUP, (index << 16) | subindex, _COE_TYPES[kind]))

    def read_field(self, axis: AxisConfig, field: str) -> Any:
        with self._lock:
            return self._plc.read_by_name(symbol_name(axis, field), SYMBOLS[field].plc_type)

    def read_fields(self, axis: AxisConfig, fields: Sequence[str]) -> dict[str, Any]:
        names = {symbol_name(axis, field): field for field in fields}
        with self._lock:
            values = self._plc.read_list_by_name(list(names))
        return {names[name]: value for name, value in values.items()}

    def write_field(self, axis: AxisConfig, field: str, value: Any) -> None:
        if field not in _SETTABLE:
            raise KeyError(f"{axis.name}: {field!r} is not a settable field")
        self._write(symbol_name(axis, field), value, SYMBOLS[field].plc_type)

    def fire_trigger(self, axis: AxisConfig, field: str) -> None:
        if field not in _TRIGGERS:
            raise KeyError(f"{axis.name}: {field!r} is not an action")
        name, plc_type = symbol_name(axis, field), SYMBOLS[field].plc_type
        self._write(name, True, plc_type)
        try:
            time.sleep(TRIGGER_PULSE_S)
        finally:
            self._write(name, False, plc_type)

    def _write(self, name: str, value: Any, plc_type: Any) -> None:
        with self._lock:
            self._plc.write_by_name(name, value, plc_type)


def connect_ads(transport: TwincatTransport) -> pyads.Connection:
    """Build an ADS connection to the PLC; routes are registered when the backend opens."""
    if transport.local_net_id is not None:
        pyads.open_port()
        pyads.set_local_address(transport.local_net_id)
    return pyads.Connection(transport.ams_net_id, transport.ams_port, transport.host)


def route_registrar(transport: TwincatTransport) -> Callable[[], None] | None:
    """Return a callable that (re)creates the local ADS routes, or None where none are needed.

    pyads' Linux router keeps one TCP connection per IP, shared by every route to it, and
    reuses it after the peer drops it. Only once no route is left is the socket closed, so
    every route is deleted before any is added back.
    """
    if transport.local_net_id is None or not hasattr(pyads, "add_route"):
        return None
    net_ids = [net_id for net_id in (transport.ams_net_id, transport.master_net_id) if net_id]

    def register() -> None:
        for net_id in net_ids:
            pyads.delete_route(pyads.AmsAddr(net_id, 0))
        for net_id in net_ids:
            pyads.add_route(net_id, transport.host)

    return register


def system_connection(transport: TwincatTransport) -> pyads.Connection:
    """Build an ADS connection to the TwinCAT system service on the PLC."""
    return pyads.Connection(transport.ams_net_id, SYSTEM_PORT, transport.host)


def drive_factory(transport: TwincatTransport) -> DeviceFactory | None:
    """Return a builder of per-drive CoE connections, or None without a master net id."""
    master = transport.master_net_id
    if master is None:
        return None
    return lambda station: pyads.Connection(master, station, transport.host)


def open_twincat(
    topology: TopologyConfig,
    connection: AdsConnection | None = None,
    drives: DeviceFactory | None = None,
    system: AdsStateReader | None = None,
) -> AxisSet:
    """Build a started AxisSet driving the PLC described by ``topology``."""
    if not isinstance(topology.transport, TwincatTransport):
        raise TypeError(f"topology {topology.name!r} is not a TwinCAT topology")
    transport = topology.transport
    if connection is None:
        backend = TwincatBackend(
            connect_ads(transport), drives or drive_factory(transport), route_registrar(transport),
            system or system_connection(transport),
        )
    else:
        backend = TwincatBackend(connection, drives or drive_factory(transport), system=system)
    axes = {config.keyword_name: Axis(backend, config) for config in topology.axes}
    backend.open()
    return AxisSet(backend, axes)
