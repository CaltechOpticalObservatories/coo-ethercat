"""
This module does cyclic process-data exchange:
- PDO layout descriptions
- packing output PDO values
- unpacking input PDO values
- optional background cyclic PDO loop
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Mapping, MutableMapping, Optional, Protocol, Sequence

from .bus import EtherCATBus

LOG = logging.getLogger(__name__)


class PDOError(RuntimeError):
    """Base exception for PDO errors."""


class PDOLoopAlreadyRunningError(PDOError):
    """Raised when starting an already-running PDO loop."""


class PDOLoopNotRunningError(PDOError):
    """Raised when an operation requires an active PDO loop."""


@dataclass(frozen=True)
class PDOEntry:
    """
    One mapped PDO object.

    The mapping value follows the common EtherCAT/CANopen format:
        index:subindex:bit_length encoded as 0xIIII_SS_LL

    Example:
        CONTROLWORD = PDOEntry("controlword", 0x6040, 0x00, "H", 16)
    """

    name: str
    index: int
    subindex: int
    fmt: str
    bits: int

    @property
    def mapping_value(self) -> int:
        """Return the 32-bit PDO mapping descriptor."""
        return (self.index << 16) | (self.subindex << 8) | self.bits


@dataclass(frozen=True)
class PDOLayout:
    """Ordered PDO layout used to pack or unpack process data."""

    entries: tuple[PDOEntry, ...]
    endian: str = "<"

    @classmethod
    def from_entries(cls, entries: Iterable[PDOEntry], *, endian: str = "<") -> "PDOLayout":
        return cls(tuple(entries), endian=endian)

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(entry.name for entry in self.entries)

    @property
    def struct_format(self) -> str:
        """Struct format without byte-order prefix, suitable for EtherCATBus."""
        return "".join(entry.fmt for entry in self.entries)

    @property
    def pack_format(self) -> str:
        """Struct format with byte-order prefix, suitable for struct.pack/unpack."""
        return self.endian + self.struct_format

    @property
    def size_bytes(self) -> int:
        return struct.calcsize(self.pack_format)

    @property
    def mapping_values(self) -> tuple[int, ...]:
        return tuple(entry.mapping_value for entry in self.entries)

    def ordered_values(self, values: Mapping[str, Any] | Sequence[Any]) -> list[Any]:
        """Return values ordered to match this layout."""
        if isinstance(values, Mapping):
            ordered_values = [values[entry.name] for entry in self.entries]
        else:
            ordered_values = list(values)

        if len(ordered_values) != len(self.entries):
            raise ValueError(
                f"Expected {len(self.entries)} PDO values, got {len(ordered_values)}"
            )
        return ordered_values

    def values_to_dict(self, values: Any) -> dict[str, Any]:
        """Convert unpacked PDO values into a dictionary keyed by entry name."""
        if len(self.entries) == 1 and not isinstance(values, tuple):
            values = (values,)
        return dict(zip(self.names, values))

    def pack(self, values: Mapping[str, Any] | Sequence[Any]) -> bytes:
        """Pack values into a PDO byte buffer."""
        return struct.pack(self.pack_format, *self.ordered_values(values))

    def unpack(self, payload: bytes) -> dict[str, Any]:
        """Unpack a PDO byte buffer into a dictionary keyed by entry name."""
        expected = self.size_bytes
        if len(payload) < expected:
            raise ValueError(f"PDO payload too small: expected {expected} bytes, got {len(payload)}")

        values = struct.unpack(self.pack_format, payload[:expected])
        return dict(zip(self.names, values))

    def empty_values(self, fill: int = 0) -> dict[str, Any]:
        """Return a zero-filled value dictionary matching this layout."""
        return {name: fill for name in self.names}


class PDODevice(Protocol):
    """
    Protocol implemented by devices that participate in cyclic PDO exchange.

    A MaxonEPOS4 class can implement this without inheriting from a PDO base
    class. The loop only needs to know the slave index, layouts, and hooks.
    """

    slave_index: int
    rx_pdo: PDOLayout
    tx_pdo: PDOLayout

    def get_pdo_outputs(self) -> Mapping[str, Any] | Sequence[Any]:
        """Return output PDO values to be packed and sent to the slave."""

    def set_pdo_inputs(self, values: Mapping[str, Any]) -> None:
        """Receive unpacked input PDO values from the slave."""


@dataclass
class PDODeviceState:
    """Runtime PDO state for one device."""

    device: PDODevice
    output_dirty: bool = True
    last_inputs: dict[str, Any] = field(default_factory=dict)
    last_outputs: Mapping[str, Any] | Sequence[Any] | None = None


@dataclass(frozen=True)
class PDOCycleStats:
    """Stats returned from one PDO cycle."""

    working_counter: int
    elapsed_s: float
    deadline_miss_s: float


class PDOLoop:
    """
    The loop can be used manually by calling cycle_once(), or in a background
    thread by calling start().
    """

    def __init__(
        self,
        bus: EtherCATBus,
        *,
        period_s: float = 0.002,
        timeout_us: int = 2_000,
        expected_working_counter: int | None = None,
        on_warning: Callable[[str], None] | None = None,
    ) -> None:
        self.bus = bus
        self.period_s = period_s
        self.timeout_us = timeout_us
        self.expected_working_counter = expected_working_counter
        self.on_warning = on_warning or LOG.warning

        self._devices: dict[int, PDODeviceState] = {}
        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_stats: PDOCycleStats | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def last_stats(self) -> PDOCycleStats | None:
        return self._last_stats

    @property
    def devices(self) -> tuple[PDODevice, ...]:
        with self._lock:
            return tuple(state.device for state in self._devices.values())

    def add_device(self, device: PDODevice) -> None:
        """Register a device for cyclic PDO exchange."""
        with self._lock:
            if device.slave_index in self._devices:
                raise ValueError(f"Device already registered for slave {device.slave_index}")
            self._devices[device.slave_index] = PDODeviceState(device=device)

    def remove_device(self, slave_index: int) -> None:
        """Remove a device from cyclic PDO exchange."""
        with self._lock:
            self._devices.pop(slave_index)

    def mark_dirty(self, slave_index: int) -> None:
        """Mark one device's output PDO as needing to be sent."""
        with self._lock:
            self._devices[slave_index].output_dirty = True

    def mark_all_dirty(self) -> None:
        """Mark all device output PDOs as needing to be sent."""
        with self._lock:
            for state in self._devices.values():
                state.output_dirty = True

    def write_outputs(self, *, only_dirty: bool = False) -> None:
        """Pack and write output PDO buffers for registered devices."""
        with self._lock:
            states = list(self._devices.values())

        for state in states:
            if only_dirty and not state.output_dirty:
                continue

            device = state.device
            outputs = device.get_pdo_outputs()
            ordered_outputs = device.rx_pdo.ordered_values(outputs)
            self.bus.write_pdo(device.slave_index, device.rx_pdo.struct_format, ordered_outputs)

            state.last_outputs = outputs
            state.output_dirty = False

    def read_inputs(self) -> None:
        """Read and unpack input PDO buffers for registered devices."""
        with self._lock:
            states = list(self._devices.values())

        for state in states:
            device = state.device
            raw_values = self.bus.read_pdo(device.slave_index, device.tx_pdo.struct_format)
            values = device.tx_pdo.values_to_dict(raw_values)
            state.last_inputs = values
            device.set_pdo_inputs(values)

    def cycle_once(self, *, only_dirty_outputs: bool = False) -> PDOCycleStats:
        """Perform one process-data cycle."""
        start = time.perf_counter()

        self.write_outputs(only_dirty=only_dirty_outputs)
        working_counter = self.bus.exchange_processdata(timeout_us=self.timeout_us)
        self.read_inputs()

        elapsed_s = time.perf_counter() - start
        deadline_miss_s = max(elapsed_s - self.period_s, 0.0)

        if (
            self.expected_working_counter is not None
            and working_counter != self.expected_working_counter
        ):
            self.on_warning(
                f"PDO working counter mismatch: got {working_counter}, "
                f"expected {self.expected_working_counter}"
            )

        if deadline_miss_s > 0:
            self.on_warning(
                f"PDO cycle exceeded period by {deadline_miss_s:.6f} s "
                f"elapsed={elapsed_s:.6f} s period={self.period_s:.6f} s"
            )

        stats = PDOCycleStats(
            working_counter=working_counter,
            elapsed_s=elapsed_s,
            deadline_miss_s=deadline_miss_s,
        )
        self._last_stats = stats
        return stats

    def start(self, *, only_dirty_outputs: bool = False) -> None:
        """Start cyclic PDO exchange in a background thread."""
        if self.running:
            raise PDOLoopAlreadyRunningError("PDO loop is already running")

        self._shutdown.clear()
        self._thread = threading.Thread(
            name="EtherCAT PDO loop",
            target=self._run,
            kwargs={"only_dirty_outputs": only_dirty_outputs},
            daemon=True,
        )
        self._thread.start()

    def stop(self, *, timeout_s: float = 2.0) -> None:
        """Stop the background PDO loop."""
        if self._thread is None:
            return

        self._shutdown.set()
        self._thread.join(timeout=timeout_s)
        if self._thread.is_alive():
            raise TimeoutError("Timed out waiting for PDO loop thread to stop")
        self._thread = None

    def require_running(self) -> None:
        if not self.running:
            raise PDOLoopNotRunningError("PDO loop is not running")

    def _run(self, *, only_dirty_outputs: bool) -> None:
        next_cycle = time.perf_counter()

        while not self._shutdown.is_set():
            cycle_start = time.perf_counter()
            self.cycle_once(only_dirty_outputs=only_dirty_outputs)

            next_cycle += self.period_s
            sleep_s = next_cycle - time.perf_counter()

            if sleep_s <= 0:
                # We missed the scheduled cycle. Resynchronize to avoid a long
                # catch-up loop if Python was descheduled for a while.
                next_cycle = time.perf_counter()
                continue

            self._shutdown.wait(sleep_s)


class BufferedPDODevice:
    """
    Device-specific classes may inherit from this, or just implement the
    PDODevice protocol directly.
    """

    def __init__(self, slave_index: int, rx_pdo: PDOLayout, tx_pdo: PDOLayout) -> None:
        self.slave_index = slave_index
        self.rx_pdo = rx_pdo
        self.tx_pdo = tx_pdo
        self._pdo_outputs: MutableMapping[str, Any] = rx_pdo.empty_values()
        self._pdo_inputs: MutableMapping[str, Any] = tx_pdo.empty_values()
        self._pdo_lock = threading.RLock()

    def get_pdo_outputs(self) -> Mapping[str, Any]:
        with self._pdo_lock:
            return dict(self._pdo_outputs)

    def set_pdo_outputs(self, **values: Any) -> None:
        with self._pdo_lock:
            unknown = set(values) - set(self.rx_pdo.names)
            if unknown:
                raise KeyError(f"Unknown output PDO field(s): {sorted(unknown)}")
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
