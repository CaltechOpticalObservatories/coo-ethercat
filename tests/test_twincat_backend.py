"""Tests for the TwinCAT backend against a fake PLC motion server."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

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
    return FakeAdsConnection(axis_count=12)


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
