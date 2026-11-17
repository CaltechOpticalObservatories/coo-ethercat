"""
Fake ADS connection carrying the PLC motion server's symbol contract.

It reacts to symbol writes the way ``FB_AxisServer`` does on the PLC: a held
``bEnable`` energises the axis, a rising ``bExecute`` edge completes the
commanded move and bumps ``nSeq``, ``bReset`` clears a latched fault. Moves
complete instantly; timing belongs to the bench, not to this fake.
"""

from __future__ import annotations

import re
from typing import Any

_SYMBOL = re.compile(r"GVL_Axes\.Axis\[(\d+)\]\.(Cmd|Sts)\.(\w+)$")


class FakeAdsConnection:
    """In-memory PLC exposing ``GVL_Axes.Axis[1..n]``."""

    def __init__(self, axis_count: int) -> None:
        self.is_open = False
        self.writes: list[tuple[str, Any]] = []
        self.sum_reads = 0
        self.axes = {index: _fresh_axis() for index in range(1, axis_count + 1)}

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def read_by_name(self, data_name: str, plc_datatype: Any = None) -> Any:
        return self._slot(data_name)

    def read_list_by_name(self, data_names: list[str]) -> dict[str, Any]:
        self.sum_reads += 1
        return {name: self._slot(name) for name in data_names}

    def write_by_name(self, data_name: str, value: Any, plc_datatype: Any = None) -> None:
        index, half, member = self._parse(data_name)
        if half != "Cmd":
            raise PermissionError(f"{data_name} is PLC-owned")
        self.writes.append((data_name, value))
        axis = self.axes[index]
        rising = bool(value) and not axis["Cmd"][member]
        axis["Cmd"][member] = value
        self._react(axis, member, rising)

    def fault(self, index: int, error_id: int) -> None:
        """Latch a fault on one axis, as an NC following-error reaction would."""
        sts = self.axes[index]["Sts"]
        sts.update(bError=True, nErrorId=error_id, bEnabled=False, bMoving=False, eState=6)

    def _react(self, axis: dict[str, dict[str, Any]], member: str, rising: bool) -> None:
        cmd, sts = axis["Cmd"], axis["Sts"]
        if member == "bEnable":
            enabled = bool(cmd["bEnable"]) and not sts["bError"]
            sts.update(bEnabled=enabled, eState=1 if enabled else 0)
        elif member == "bExecute" and rising and sts["bEnabled"] and not sts["bError"]:
            relative = cmd["eMode"] == 1
            sts["fPosition"] = sts["fPosition"] + cmd["fTarget"] if relative else cmd["fTarget"]
            sts["nSeq"] += 1
        elif member == "bReset" and rising:
            sts.update(bError=False, nErrorId=0, eState=0)

    def _slot(self, data_name: str) -> Any:
        index, half, member = self._parse(data_name)
        return self.axes[index][half][member]

    def _parse(self, data_name: str) -> tuple[int, str, str]:
        match = _SYMBOL.match(data_name)
        if match is None or int(match[1]) not in self.axes:
            raise KeyError(f"symbol not found: {data_name}")
        return int(match[1]), match[2], match[3]


def _fresh_axis() -> dict[str, dict[str, Any]]:
    return {
        "Cmd": {
            "bEnable": False, "bExecute": False, "bStop": False, "bReset": False,
            "bHome": False, "eMode": 0, "fTarget": 0.0, "fVelocity": 0.0,
            "fAcceleration": 0.0, "fDeceleration": 0.0,
        },
        "Sts": {
            "bReady": True, "bEnabled": False, "bMoving": False, "bError": False,
            "nErrorId": 0, "eState": 0, "fPosition": 0.0, "fVelocity": 0.0, "nSeq": 0,
        },
    }


class FakeDrive:
    """One EPOS4 answering CoE reads through the master's ADS port."""

    def __init__(self, objects: dict[tuple[int, int], int]) -> None:
        self.objects = objects
        self.is_open = False
        self.reads: list[tuple[int, int, Any]] = []

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def read(self, index_group: int, index_offset: int, plc_datatype: Any) -> Any:
        self.reads.append((index_group, index_offset, plc_datatype))
        return self.objects[(index_offset >> 16, index_offset & 0xFFFF)]


class FakeSystemService:
    """The TwinCAT system service, reporting the runtime's ADS state."""

    def __init__(self, state: int = 5) -> None:
        self.state = state
        self.is_open = False

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def read_state(self) -> tuple[int, int]:
        return self.state, 0
