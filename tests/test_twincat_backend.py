"""Tests for the TwinCAT backend against a fake PLC motion server."""

from __future__ import annotations

import threading
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pyads
import pytest
from fake_ads import FakeAdsConnection, FakeDrive, FakeSystemService

from cooethercat.axis import (
    ALL_FIELDS, STATUS, AxisError, AxisSet, AxisState, CoeType, ControllerState,
)
from cooethercat.backends import twincat
from cooethercat.backends.twincat import SYMBOLS, open_twincat, symbol_name
from cooethercat.config import load_topology

CSU_DEMO = Path(__file__).parent.parent / "configs" / "csu-demo.yaml"


@pytest.fixture(name="plc")
def plc_fixture() -> FakeAdsConnection:
    return FakeAdsConnection(axis_count=23)


BOARD_TEMPERATURE = (0x3201, 1)


@pytest.fixture(name="drives")
def drives_fixture() -> dict[int, FakeDrive]:
    return {}


@pytest.fixture(name="system")
def system_fixture() -> FakeSystemService:
    return FakeSystemService()


@pytest.fixture(name="axes")
def axes_fixture(
    plc: FakeAdsConnection,
    drives: dict[int, FakeDrive],
    system: FakeSystemService,
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[AxisSet]:
    monkeypatch.setattr(twincat, "TRIGGER_PULSE_S", 0.0)

    def open_drive(station: int) -> FakeDrive:
        return drives.setdefault(station, FakeDrive({BOARD_TEMPERATURE: 724 + station}))

    with open_twincat(
        load_topology(CSU_DEMO), connection=plc, drives=open_drive, system=system
    ) as axis_set:
        yield axis_set


def test_every_interface_field_has_a_symbol() -> None:
    assert set(SYMBOLS) == set(ALL_FIELDS)


def test_symbol_follows_plc_index() -> None:
    topology = load_topology(CSU_DEMO)
    assert symbol_name(topology.axis("CSU-0"), "position") == "GVL_Axes.Axis[1].Sts.fPosition"
    assert symbol_name(topology.axis("CSU-11"), "enable") == "GVL_Axes.Axis[12].Cmd.bEnable"


def test_open_and_close_follow_the_axis_set(plc: FakeAdsConnection) -> None:
    with open_twincat(load_topology(CSU_DEMO), connection=plc):
        assert plc.is_open
    assert not plc.is_open


def test_status_is_one_sum_read(axes: AxisSet, plc: FakeAdsConnection) -> None:
    status = axes.axis("CSU-3").status()
    assert set(status) == set(STATUS)
    assert plc.sum_reads == 1


def test_every_axis_is_one_sum_read(axes: AxisSet, plc: FakeAdsConnection) -> None:
    snapshot = axes.read(("position", "velocity"))
    assert set(snapshot) == set(axes.names)
    assert all(set(fields) == {"position", "velocity"} for fields in snapshot.values())
    assert plc.sum_reads == 1


def test_trigger_is_a_rising_then_falling_edge(axes: AxisSet, plc: FakeAdsConnection) -> None:
    axes.axis("CSU-0").stop()
    assert plc.writes == [("GVL_Axes.Axis[1].Cmd.bStop", True), ("GVL_Axes.Axis[1].Cmd.bStop", False)]


def test_status_fields_are_not_writable(axes: AxisSet) -> None:
    with pytest.raises(AxisError):
        axes.axis("CSU-0").set("position", 1.0)


def test_enable_then_relative_move(axes: AxisSet) -> None:
    axis = axes.axis("CSU-2")
    axis.enable(timeout_s=0.5)
    assert axis.state is AxisState.STANDSTILL
    assert axis.move_relative(3.0, velocity=2.0, timeout_s=0.5) == pytest.approx(3.0)
    assert axis.move_relative(-1.0, velocity=2.0, timeout_s=0.5) == pytest.approx(2.0)


def test_velocity_above_config_limit_is_rejected(axes: AxisSet, plc: FakeAdsConnection) -> None:
    axis = axes.axis("CSU-0")
    axis.enable(timeout_s=0.5)
    with pytest.raises(AxisError, match="velocity"):
        axis.move_relative(1.0, velocity=50.0)
    assert not any("bExecute" in name for name, _ in plc.writes)


def test_enable_times_out_when_the_plc_never_reports_enabled(
    axes: AxisSet, plc: FakeAdsConnection, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(plc, "_react", lambda *_: None)
    with pytest.raises(AxisError, match="enable timed out"):
        axes.axis("CSU-5").enable(timeout_s=0.1)


def test_fault_blocks_moves_until_reset(axes: AxisSet, plc: FakeAdsConnection) -> None:
    axis = axes.axis("CSU-1")
    axis.enable(timeout_s=0.5)
    plc.fault(index=2, error_id=0x4550)
    with pytest.raises(AxisError, match="fault 17744"):
        axis.move_relative(1.0)
    axis.reset()
    assert not axis.error


def test_coe_read_addresses_the_drive_by_station(axes: AxisSet, drives: dict[int, FakeDrive]) -> None:
    assert axes.axis("CSU-6").read_object(*BOARD_TEMPERATURE, CoeType.INT16) == 724 + 1007
    assert drives[1007].reads == [(0xF302, 0x32010001, pyads.PLCTYPE_INT)]


def test_coe_connection_is_reused_and_closed(
    plc: FakeAdsConnection, drives: dict[int, FakeDrive], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(twincat, "TRIGGER_PULSE_S", 0.0)
    opened: list[int] = []

    def open_drive(station: int) -> FakeDrive:
        opened.append(station)
        return drives.setdefault(station, FakeDrive({BOARD_TEMPERATURE: 700}))

    with open_twincat(load_topology(CSU_DEMO), connection=plc, drives=open_drive) as axis_set:
        for _ in range(3):
            axis_set.axis("CSU-0").read_object(*BOARD_TEMPERATURE, CoeType.INT16)
        assert opened == [1001]
    assert not drives[1001].is_open


def test_coe_without_master_net_id_is_refused(plc: FakeAdsConnection) -> None:
    topology = load_topology(CSU_DEMO)
    backend = twincat.TwincatBackend(plc)
    with pytest.raises(KeyError, match="master_net_id"):
        backend.read_object(topology.axis("CSU-0"), *BOARD_TEMPERATURE, CoeType.INT16)


def test_every_csu_bar_has_a_station_address() -> None:
    topology = load_topology(CSU_DEMO)
    assert [axis.station_address for axis in topology.axes] == list(range(1001, 1024))


@pytest.mark.parametrize(
    ("ads_state", "expected"),
    [(5, ControllerState.RUN), (15, ControllerState.CONFIG), (6, ControllerState.STOP),
     (11, ControllerState.OTHER)],
)
def test_controller_state_follows_the_system_service(
    axes: AxisSet, system: FakeSystemService, ads_state: int, expected: ControllerState
) -> None:
    system.state = ads_state
    assert axes.backend.controller_state() is expected


def test_system_connection_follows_the_axis_set(system: FakeSystemService) -> None:
    with open_twincat(load_topology(CSU_DEMO), connection=FakeAdsConnection(23), system=system):
        assert system.is_open
    assert not system.is_open


def test_route_order_frees_the_shared_socket(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[str, str]] = []
    monkeypatch.setattr(pyads, "delete_route", lambda addr: calls.append(("delete", addr.netid)))
    monkeypatch.setattr(pyads, "add_route", lambda net_id, host: calls.append(("add", net_id)))
    transport = load_topology(CSU_DEMO).transport
    assert isinstance(transport, twincat.TwincatTransport)
    register = twincat.route_registrar(transport)
    assert register is not None
    register()
    plc, master = "5.57.12.96.1.1", "5.57.12.96.4.1"
    assert calls == [("delete", plc), ("delete", plc), ("add", master), ("delete", master),
                     ("add", plc), ("add", master)]


class GatedDrive(FakeDrive):
    """A drive whose reads wait until the test opens the gate."""

    def __init__(self, objects: dict[tuple[int, int], int]) -> None:
        super().__init__(objects)
        self.entered = threading.Event()
        self.gate = threading.Event()
        self.waiting = False

    def read(self, index_group: int, index_offset: int, plc_datatype: Any) -> Any:
        self.waiting = True
        self.entered.set()
        self.gate.wait(5.0)
        self.waiting = False
        return super().read(index_group, index_offset, plc_datatype)


def test_a_stalled_drive_holds_up_only_its_own_reads(
    axes: AxisSet, drives: dict[int, FakeDrive]
) -> None:
    """While one drive's CoE read hangs, PLC reads and other drives still answer."""
    stalled = drives[1001] = GatedDrive({BOARD_TEMPERATURE: 700})
    reader = threading.Thread(
        target=axes.axis("CSU-0").read_object, args=(*BOARD_TEMPERATURE, CoeType.INT16)
    )
    reader.start()
    try:
        assert stalled.entered.wait(5.0)
        assert set(axes.read(("position",))) == set(axes.names)
        assert axes.axis("CSU-6").read_object(*BOARD_TEMPERATURE, CoeType.INT16) == 724 + 1007
        assert axes.backend.controller_state() is ControllerState.RUN
        assert stalled.waiting
    finally:
        stalled.gate.set()
        reader.join(5.0)


def test_axes_read_within_max_age_is_one_sum_read(axes: AxisSet, plc: FakeAdsConnection) -> None:
    """A second snapshot inside max_age comes from the cache."""
    first = axes.read(("position",), max_age=10.0)
    assert axes.read(("position",), max_age=10.0) == first
    assert plc.sum_reads == 1
    axes.read(("position",))
    assert plc.sum_reads == 2


def test_write_clears_the_cached_snapshot(axes: AxisSet, plc: FakeAdsConnection) -> None:
    """A read after a write reaches the PLC even inside max_age."""
    axes.read(("velocity",), max_age=10.0)
    axes.axis("CSU-0").set("velocity", 3.0)
    assert axes.read(("velocity",), max_age=10.0)["CSU-0"]["velocity"] == 3.0
    assert plc.sum_reads == 2


def test_object_read_within_max_age_reads_the_drive_once(
    axes: AxisSet, drives: dict[int, FakeDrive]
) -> None:
    """A CoE object read inside max_age comes from the cache."""
    axis = axes.axis("CSU-6")
    for _ in range(3):
        assert axis.read_object(*BOARD_TEMPERATURE, CoeType.INT16, max_age=10.0) == 724 + 1007
    assert len(drives[1007].reads) == 1
