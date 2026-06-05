"""
EtherCAT bus layer.

This module is focuses on:
- open/close the EtherCAT interface
- discover/configure slaves
- read/write SDOs
- exchange PDO process data
- manage EtherCAT network/slave states
"""

from __future__ import annotations

import functools
import logging
import struct
import sys
from dataclasses import dataclass
from enum import IntEnum
from threading import RLock
from typing import Any, Optional, Sequence

import pysoem

LOG = logging.getLogger(__name__)


class EtherCATError(RuntimeError):
    """Base exception for EtherCAT bus errors."""


class BusNotOpenError(EtherCATError):
    """Raised when a bus operation requires an opened interface."""


class SlaveNotFoundError(EtherCATError):
    """Raised when a requested slave index is invalid."""


class StateTransitionError(EtherCATError):
    """Raised when a slave or network fails to reach the requested state."""


class EtherCATState(IntEnum):
    """Common EtherCAT application-layer states."""

    INIT = 0x01
    PREOP = 0x02
    BOOT = 0x03
    SAFEOP = 0x04
    OP = 0x08


@dataclass(frozen=True)
class SlaveInfo:
    """Lightweight description of an EtherCAT slave."""

    index: int
    name: str
    manufacturer_id: Any
    product_id: Any
    revision: Any
    state: Any


class EtherCATBus:
    """
    Thin wrapper around a pysoem Master.

    Higher-level device classes should depend on this class instead of reaching
    directly into pysoem. 
    """

    def __init__(
        self,
        ifname: str,
        *,
        master: Optional[pysoem.Master] = None,
        validate_interface: bool = True,
    ) -> None:
        self.ifname = ifname
        self.master = master or pysoem.Master()
        self.validate_interface = validate_interface
        self.lock = RLock()
        self._is_open = False
        self._is_configured = False

    @staticmethod
    def locked(method):
        """Serialize access to the underlying pysoem master."""

        @functools.wraps(method)
        def wrapper(self: "EtherCATBus", *args, **kwargs):
            with self.lock:
                return method(self, *args, **kwargs)

        return wrapper

    def __enter__(self) -> "EtherCATBus":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def is_configured(self) -> bool:
        return self._is_configured

    def _require_open(self) -> None:
        if not self._is_open:
            raise BusNotOpenError(f"EtherCAT interface {self.ifname!r} is not open")

    @staticmethod
    def _normalize_state(state: int | IntEnum) -> int:
        return int(state.value) if isinstance(state, IntEnum) else int(state)

    def _slave(self, slave_index: int):
        slaves = self._require_slaves()
        try:
            return slaves[slave_index]
        except IndexError as exc:
            raise SlaveNotFoundError(f"No EtherCAT slave at index {slave_index}") from exc

    def _validate_linux_interface_is_up(self) -> None:
        """
        Check interface state on Linux.

        macOS and other platforms do not expose /sys/class/net, so on those
        systems pysoem is allowed to attempt opening the interface directly.
        """
        if not self.validate_interface:
            return

        if not sys.platform.startswith("linux"):
            return

        operstate = f"/sys/class/net/{self.ifname}/operstate"
        try:
            with open(operstate, "r", encoding="utf-8") as f:
                state = f.read().strip().lower()
        except FileNotFoundError as exc:
            raise EtherCATError(
                f"Interface {self.ifname!r} was not found at {operstate}"
            ) from exc

        if state != "up":
            raise EtherCATError(
                f"Interface {self.ifname!r} is not UP; current state is {state!r}. "
                f"Try: sudo ip link set dev {self.ifname} up"
            )

    @locked
    def open(self) -> None:
        """Open the EtherCAT network interface."""
        if self._is_open:
            return

        self._validate_linux_interface_is_up()
        self.master.open(self.ifname)
        self._is_open = True
        LOG.info("Opened EtherCAT interface %s", self.ifname)

    @locked
    def close(self) -> None:
        """Close the EtherCAT network interface."""
        if not self._is_open:
            return

        try:
            self.master.close()
        finally:
            self._is_open = False
            self._is_configured = False
            LOG.info("Closed EtherCAT interface %s", self.ifname)

    @locked
    def scan(self) -> list[SlaveInfo]:
        """
        Discover slaves and return their basic information.

        This calls pysoem config_init(), which initializes the slave list.
        """
        self._require_open()
        count = self.master.config_init()
        LOG.info("Discovered %d EtherCAT slave(s)", count)
        return self.slave_info()

    @locked
    def configure(self) -> None:
        """Configure process-data mapping for discovered slaves."""
        self._require_open()
        self.master.config_map()
        self._is_configured = True
        LOG.info("Configured EtherCAT PDO mapping")

    @locked
    def slave_info(self) -> list[SlaveInfo]:
        """Return basic information for all currently discovered slaves."""
        self._require_open()

        info: list[SlaveInfo] = []
        for index, slave in enumerate(self.master.slaves):
            info.append(
                SlaveInfo(
                    index=index,
                    name=getattr(slave, "name", ""),
                    manufacturer_id=getattr(slave, "man", None),
                    product_id=getattr(slave, "id", None),
                    revision=getattr(slave, "rev", None),
                    state=getattr(slave, "state", None),
                )
            )
        return info

    def format_slave_info(self) -> str:
        """Return a human-readable slave summary."""
        lines = ["EtherCAT slaves:"]
        for slave in self.slave_info():
            lines.append(
                f"  [{slave.index}] {slave.name} "
                f"man={slave.manufacturer_id} "
                f"id={slave.product_id} "
                f"rev={slave.revision} "
                f"state={slave.state}"
            )
        return "\n".join(lines)

    @locked
    def read_sdo(
        self,
        slave_index: int,
        index: int,
        subindex: int,
        fmt: str,
    ) -> Any:
        """
        Read an SDO and unpack it using a struct format.

        Example:
            statusword = bus.read_sdo(0, 0x6041, 0x00, "H")
        """
        slave = self._slave(slave_index)
        raw = slave.sdo_read(index, subindex)
        values = struct.unpack("<" + fmt, raw)
        return values[0] if len(values) == 1 else values

    @locked
    def write_sdo(
        self,
        slave_index: int,
        index: int,
        subindex: int,
        fmt: str,
        value: Any,
        *,
        complete_access: bool = False,
    ) -> None:
        """
        Pack and write an SDO.

        Example:
            bus.write_sdo(0, 0x6040, 0x00, "H", 0x0006)
        """
        slave = self._slave(slave_index)

        if isinstance(value, Sequence) and not isinstance(value, (bytes, bytearray, str)):
            payload = struct.pack("<" + fmt, *value)
        else:
            payload = struct.pack("<" + fmt, value)

        slave.sdo_write(index, subindex, payload, ca=complete_access)

    @locked
    def send_processdata(self) -> int:
        """Send PDO process data to slaves."""
        self._require_open()
        return self.master.send_processdata()

    @locked
    def receive_processdata(self, timeout_us: int = 2000) -> int:
        """Receive PDO process data from slaves."""
        self._require_open()
        return self.master.receive_processdata(timeout_us)

    @locked
    def exchange_processdata(self, timeout_us: int = 2000) -> int:
        """Perform one send/receive PDO exchange."""
        self._require_open()
        self.master.send_processdata()
        return self.master.receive_processdata(timeout_us)

    @locked
    def write_pdo(self, slave_index: int, fmt: str, values: Sequence[Any]) -> None:
        """Pack values into a slave output PDO buffer."""
        slave = self._slave(slave_index)
        slave.output = struct.pack("<" + fmt, *values)

    @locked
    def read_pdo(self, slave_index: int, fmt: str) -> Any:
        """Unpack values from a slave input PDO buffer."""
        slave = self._slave(slave_index)
        size = struct.calcsize("<" + fmt)
        values = struct.unpack("<" + fmt, slave.input[:size])
        return values[0] if len(values) == 1 else values

    @locked
    def set_watchdog(self, slave_index: int, timeout_ms: float) -> None:
        """Set PDI and process-data watchdogs for a slave."""
        slave = self._slave(slave_index)
        slave.set_watchdog("pdi", timeout_ms)
        slave.set_watchdog("processdata", timeout_ms)

    @locked
    def read_states(self) -> list[int]:
        """Refresh and return all slave states."""
        slaves = self._require_slaves()
        self.master.read_state()
        return [slave.state for slave in slaves]

    @locked
    def set_master_state(self, state: int | IntEnum) -> None:
        """Request a state for the whole EtherCAT network."""
        self._require_open()
        self.master.state = self._normalize_state(state)
        self.master.write_state()

    @locked
    def wait_for_master_state(
        self,
        state: int | IntEnum,
        *,
        timeout_us: int = 50_000,
    ) -> bool:
        """Return True if the network reaches the requested state."""
        self._require_open()
        requested = self._normalize_state(state)
        return self.master.state_check(requested, timeout_us) == requested

    def assert_master_state(
        self,
        state: int | IntEnum,
        *,
        timeout_us: int = 50_000,
    ) -> None:
        """Raise if the whole network does not reach the requested state."""
        requested = self._normalize_state(state)
        if not self.wait_for_master_state(requested, timeout_us=timeout_us):
            raise StateTransitionError(
                f"EtherCAT network did not reach state {requested:#x}; "
                f"current states: {self.read_states()}"
            )

    @locked
    def set_slave_state(self, slave_index: int, state: int | IntEnum) -> None:
        """Request a state for one slave."""
        slave = self._slave(slave_index)
        slave.state = self._normalize_state(state)
        slave.write_state()

    @locked
    def get_slave_state(self, slave_index: int) -> int:
        """Refresh and return one slave state."""
        self._require_open()
        self.master.read_state()
        return self._slave(slave_index).state

    def is_slave_state(self, slave_index: int, state: int | IntEnum) -> bool:
        """Return True if one slave is currently in the requested state."""
        return self.get_slave_state(slave_index) == self._normalize_state(state)

    def _require_slaves(self):
        self._require_open()

        if self.master.slaves is None:
            raise EtherCATError(
                "No EtherCAT slaves have been initialized yet. "
                "Call bus.scan() before reading slave states or creating devices."
            )

        return self.master.slaves