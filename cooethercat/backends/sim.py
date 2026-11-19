"""
In-process axis simulator.

Enough of a master's observable behaviour to drive the axis API through its
paces with no hardware: a level-held enable, edge-triggered actions, constant
velocity motion between targets, a completion counter, and latched faults.

It exists so the axis API, the CLI and the libby adapter are testable in CI. It
models the interface contract, not the physics, and never stands in for a bench
measurement.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from typing import Any, Final

from ..axis import Axis, AxisSet, AxisState, CoeType, ControllerState, MoveMode
from ..config import AxisConfig, SimTransport, TopologyConfig

_DEFAULT_TICK_S: Final = 0.01


class _SimAxis:
    """Simulated state of one axis."""

    def __init__(self, config: AxisConfig) -> None:
        self.config = config
        self.settings: dict[str, Any] = {
            "enable": False,
            "mode": int(MoveMode.ABSOLUTE),
            "target": 0.0,
            "velocity": min(1.0, config.limits.velocity),
            "acceleration": config.limits.acceleration or 0.0,
            "deceleration": config.limits.effective_deceleration or 0.0,
        }
        self.status: dict[str, Any] = {
            "position": 0.0,
            "actual_velocity": 0.0,
            "state": int(AxisState.DISABLED),
            "moving": False,
            "enabled": False,
            "ready": True,
            "error": False,
            "error_id": 0,
            "sequence": 0,
        }
        self._goal: float | None = None

    def start_move(self) -> None:
        """Latch the commanded target, which a real master does on a rising edge."""
        if self.status["error"] or not self.status["enabled"]:
            return

        mode = MoveMode(self.settings["mode"])
        if mode is MoveMode.RELATIVE:
            self._goal = self.status["position"] + self.settings["target"]
        else:
            self._goal = self.settings["target"]

        self.status["moving"] = True
        self.status["state"] = int(AxisState.DISCRETE_MOTION)

    def halt(self) -> None:
        """Abandon the in-flight move and settle at the current position."""
        self._goal = None
        self.status.update(moving=False, actual_velocity=0.0)
        if self.status["enabled"]:
            self.status["state"] = int(AxisState.STANDSTILL)

    def clear_fault(self) -> None:
        """Clear a latched fault, as a controlword fault reset would."""
        self.status.update(error=False, error_id=0)
        if self.status["state"] == int(AxisState.ERROR_STOP):
            self.status["state"] = int(AxisState.DISABLED)

    def home(self) -> None:
        """Define the current position as zero and count it as one command."""
        if self.status["error"] or not self.status["enabled"]:
            return
        self._goal = None
        self.status.update(
            position=0.0,
            actual_velocity=0.0,
            moving=False,
            state=int(AxisState.STANDSTILL),
            sequence=self.status["sequence"] + 1,
        )

    def fault(self, error_id: int) -> None:
        """Latch a fault, which stops motion the way a drive fault reaction would."""
        self._goal = None
        self.status.update(
            error=True,
            error_id=error_id,
            moving=False,
            actual_velocity=0.0,
            state=int(AxisState.ERROR_STOP),
        )

    def step(self, elapsed_s: float) -> None:
        """Advance the simulated axis by one tick."""
        self.status["enabled"] = bool(self.settings["enable"]) and not self.status["error"]

        if not self.status["enabled"]:
            self._goal = None
            self.status.update(
                moving=False,
                actual_velocity=0.0,
                state=int(AxisState.ERROR_STOP)
                if self.status["error"]
                else int(AxisState.DISABLED),
            )
            return

        if self._goal is None:
            if not self.status["moving"]:
                self.status["state"] = int(AxisState.STANDSTILL)
            return

        speed = abs(float(self.settings["velocity"]))
        remaining = self._goal - self.status["position"]
        stride = speed * elapsed_s

        # Settle exactly on target once it is within one tick's travel, so a move
        # converges instead of stepping past the goal.
        if abs(remaining) <= stride:
            self.status.update(
                position=self._goal,
                actual_velocity=0.0,
                moving=False,
                state=int(AxisState.STANDSTILL),
                sequence=self.status["sequence"] + 1,
            )
            self._goal = None
            return

        direction = 1.0 if remaining > 0 else -1.0
        self.status["position"] += direction * stride
        self.status["actual_velocity"] = direction * speed


class SimBackend:
    """Axis backend that models the interface contract with no hardware."""

    def __init__(self, topology: TopologyConfig) -> None:
        transport = topology.transport
        self.tick_s = transport.tick_s if isinstance(transport, SimTransport) else _DEFAULT_TICK_S
        self._axes = {config.keyword_name: _SimAxis(config) for config in topology.axes}
        self._objects: dict[tuple[str, int, int], int] = {}
        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None

    def open(self) -> None:
        """Start the background tick that advances every simulated axis."""
        if self._thread is not None:
            return
        self._shutdown.clear()
        self._thread = threading.Thread(name="axis simulator", target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        """Stop the background tick."""
        if self._thread is None:
            return
        self._shutdown.set()
        self._thread.join(timeout=2.0)
        self._thread = None

    def read_field(self, axis: AxisConfig, field: str) -> Any:
        with self._lock:
            return self._read(self._locate(axis), field)

    def read_fields(self, axis: AxisConfig, fields: Sequence[str]) -> dict[str, Any]:
        with self._lock:
            simulated = self._locate(axis)
            return {field: self._read(simulated, field) for field in fields}

    def read_axes_fields(
        self, axes: Sequence[AxisConfig], fields: Sequence[str]
    ) -> dict[str, dict[str, Any]]:
        """Read the fields of every axis under one lock."""
        with self._lock:
            return {axis.name: self.read_fields(axis, fields) for axis in axes}

    @staticmethod
    def _read(simulated: _SimAxis, field: str) -> Any:
        if field in simulated.status:
            return simulated.status[field]
        return simulated.settings[field]

    def write_field(self, axis: AxisConfig, field: str, value: Any) -> None:
        with self._lock:
            simulated = self._locate(axis)
            if field not in simulated.settings:
                raise KeyError(f"{axis.name}: {field!r} is not a settable field")
            simulated.settings[field] = value

    def fire_trigger(self, axis: AxisConfig, field: str) -> None:
        with self._lock:
            simulated = self._locate(axis)
            match field:
                case "move":
                    simulated.start_move()
                case "stop":
                    simulated.halt()
                case "reset":
                    simulated.clear_fault()
                case "home":
                    simulated.home()
                case _:
                    raise KeyError(f"{axis.name}: {field!r} is not an action")

    def controller_state(self) -> ControllerState:
        return ControllerState.RUN

    def read_object(self, axis: AxisConfig, index: int, subindex: int, kind: CoeType) -> int:
        """Return a value set with ``set_object``, or 0 for an object never set."""
        with self._lock:
            self._locate(axis)
            return self._objects.get((axis.keyword_name, index, subindex), 0)

    def set_object(self, axis_name: str, index: int, subindex: int, value: int) -> None:
        """Give a simulated drive object a value, e.g. a board temperature."""
        with self._lock:
            self._objects[(axis_name.lower(), index, subindex)] = value

    def inject_fault(self, axis_name: str, error_id: int = 1) -> None:
        """Latch a fault on one axis so callers can exercise recovery paths."""
        with self._lock:
            self._axes[axis_name.lower()].fault(error_id)

    def _locate(self, axis: AxisConfig) -> _SimAxis:
        try:
            return self._axes[axis.keyword_name]
        except KeyError:
            raise KeyError(f"no simulated axis {axis.name!r}") from None

    def _run(self) -> None:
        last = time.monotonic()
        while not self._shutdown.wait(self.tick_s):
            now = time.monotonic()
            elapsed_s, last = now - last, now
            with self._lock:
                for simulated in self._axes.values():
                    simulated.step(elapsed_s)


def open_simulated(topology: TopologyConfig) -> AxisSet:
    """Build a started AxisSet backed by the simulator."""
    backend = SimBackend(topology)
    axes = {config.keyword_name: Axis(backend, config) for config in topology.axes}
    backend.open()
    return AxisSet(backend, axes)
