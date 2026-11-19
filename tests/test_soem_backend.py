"""
Tests for the pysoem axis backend, run against a fake master.

These cover the master-side logic Python owns: the CiA 402 state machine, the
set-point handshake, unit conversion and fault handling. They do not stand in
for a bench run, which is the only thing that can confirm the drive's own
velocity units and timing.
"""

from __future__ import annotations

import struct
import time
from collections.abc import Iterator
from typing import Any

import pysoem
import pytest
from fake_soem import FakeEPOS4Slave, FakeMaster

from cooethercat.axis import (
    SETTINGS, STATUS, Axis, AxisSet, AxisState, CoeType, ControllerState, MoveMode,
)
from cooethercat.backends.soem import SoemBackend, open_soem
from cooethercat.bus import EtherCATBus, EtherCATState
from cooethercat.cia402 import ControlwordBit, OperatingMode
from cooethercat.config import TopologyConfig, parse_topology

INCREMENTS_PER_DEGREE = 100.0
PERIOD_S = 0.001
SETTLE_S = 0.08


def build_topology(axis_count: int = 1) -> TopologyConfig:
    return parse_topology(
        {
            "name": "fake-bench",
            "backend": "pysoem",
            "transport": {"ifname": "fake0"},
            "defaults": {
                "units": {"name": "deg", "increments_per_unit": INCREMENTS_PER_DEGREE},
                "limits": {
                    "velocity": 50.0,
                    "min_position": -100.0,
                    "max_position": 100.0,
                },
            },
            "axes": [
                {"name": f"EPOS{index + 1}", "slave_index": index}
                for index in range(axis_count)
            ],
        }
    )


def build_bus(master: FakeMaster) -> EtherCATBus:
    return EtherCATBus("fake0", master=master, validate_interface=False)


@pytest.fixture(name="master")
def master_fixture() -> FakeMaster:
    return FakeMaster([FakeEPOS4Slave(name="EPOS1")])


@pytest.fixture(name="axes")
def axes_fixture(master: FakeMaster) -> Iterator[AxisSet]:
    topology = build_topology()
    with open_soem(topology, bus=build_bus(master), period_s=PERIOD_S) as axis_set:
        yield axis_set


@pytest.fixture(name="axis")
def axis_fixture(axes: AxisSet) -> Axis:
    return axes.axis("EPOS1")


@pytest.fixture(name="enabled_axis")
def enabled_axis_fixture(axis: Axis) -> Axis:
    axis.enable(timeout_s=2.0)
    return axis


def test_open_reaches_operational_and_registers_every_axis(master: FakeMaster) -> None:
    topology = build_topology(axis_count=2)
    master_two = FakeMaster([FakeEPOS4Slave(name="EPOS1"), FakeEPOS4Slave(name="EPOS2")])
    backend = SoemBackend(topology, bus=build_bus(master_two), period_s=PERIOD_S)

    backend.open()
    try:
        assert backend.loop.running
        assert master_two.state == int(EtherCATState.OP)
        assert set(backend.drives) == {"epos1", "epos2"}
        assert backend.loop.expected_working_counter == 6
    finally:
        backend.close()

    assert not backend.loop.running
    assert not master_two.is_open


def test_open_refuses_a_bus_with_too_few_slaves() -> None:
    topology = build_topology(axis_count=3)
    master = FakeMaster([FakeEPOS4Slave(name="EPOS1")])
    backend = SoemBackend(topology, bus=build_bus(master), period_s=PERIOD_S)

    with pytest.raises(RuntimeError, match="expects 3 slave"):
        backend.open()


def test_missing_pysoem_transport_is_rejected() -> None:
    topology = parse_topology(
        {
            "name": "sim-only",
            "backend": "sim",
            "transport": {},
            "defaults": {"units": {"name": "mm"}, "limits": {"velocity": 1.0}},
            "axes": [{"name": "AX1"}],
        }
    )
    with pytest.raises(TypeError, match="no pysoem transport"):
        SoemBackend(topology)


def test_enable_is_held_as_a_level_through_the_state_machine(axis: Axis) -> None:
    assert not axis.enabled
    assert axis.state is AxisState.DISABLED

    axis.enable(timeout_s=2.0)
    assert axis.enabled
    assert axis.state is AxisState.STANDSTILL

    axis.disable(timeout_s=2.0)
    assert not axis.enabled
    assert axis.state is AxisState.DISABLED


def test_moves_land_on_target_in_engineering_units(
    enabled_axis: Axis, master: FakeMaster
) -> None:
    assert enabled_axis.move_absolute(10.0, velocity=50.0, timeout_s=5.0) == pytest.approx(10.0)
    assert master.slaves[0].position == pytest.approx(10.0 * INCREMENTS_PER_DEGREE)

    assert enabled_axis.move_relative(-2.5, velocity=50.0, timeout_s=5.0) == pytest.approx(7.5)
    assert master.slaves[0].position == pytest.approx(7.5 * INCREMENTS_PER_DEGREE)
    assert enabled_axis.state is AxisState.STANDSTILL


def test_commanded_values_reach_the_drive_as_increments(
    enabled_axis: Axis, master: FakeMaster
) -> None:
    enabled_axis.set("velocity", 4.0)
    enabled_axis.set("target", -3.0)
    time.sleep(SETTLE_S)

    assert master.slaves[0].profile_velocity == 4.0 * INCREMENTS_PER_DEGREE
    assert master.slaves[0].target_position == -3.0 * INCREMENTS_PER_DEGREE
    assert enabled_axis.get("velocity") == pytest.approx(4.0)
    assert enabled_axis.get("target") == pytest.approx(-3.0)


def test_a_zero_distance_move_completes_instead_of_hanging(enabled_axis: Axis) -> None:
    enabled_axis.move_absolute(5.0, velocity=50.0, timeout_s=5.0)

    assert enabled_axis.move_absolute(5.0, velocity=50.0, timeout_s=5.0) == pytest.approx(5.0)


def test_each_move_advances_the_sequence_counter_once(enabled_axis: Axis) -> None:
    start = enabled_axis.get("sequence")

    for target in (1.0, 2.0, 3.0):
        enabled_axis.move_absolute(target, velocity=50.0, timeout_s=5.0)

    assert enabled_axis.get("sequence") == start + 3


def test_homing_zeroes_the_axis(enabled_axis: Axis, master: FakeMaster) -> None:
    enabled_axis.move_absolute(9.0, velocity=50.0, timeout_s=5.0)

    enabled_axis.home()
    enabled_axis.wait(timeout_s=5.0)

    assert enabled_axis.position == pytest.approx(0.0)
    assert master.slaves[0].mode == int(OperatingMode.HOMING)


def test_the_setpoint_edge_waits_for_the_drive_to_acknowledge_its_mode() -> None:
    slave = FakeEPOS4Slave(name="EPOS1", mode_ack_delay_cycles=25)
    master = FakeMaster([slave])

    with open_soem(build_topology(), bus=build_bus(master), period_s=PERIOD_S) as axes:
        axis = axes.axis("EPOS1")
        axis.enable(timeout_s=2.0)
        axis.move_absolute(4.0, velocity=50.0, timeout_s=5.0)

        # Homing changes the operating mode, which this drive acknowledges slowly.
        axis.home()
        asserted_early = []
        for _ in range(12):
            time.sleep(PERIOD_S)
            if slave.mode != int(OperatingMode.HOMING):
                asserted_early.append(
                    bool(slave.last_controlword & (1 << int(ControlwordBit.NEW_SETPOINT)))
                )

        assert asserted_early, "the delayed mode acknowledgement was never observed"
        assert not any(asserted_early), "start bit asserted before the mode was acknowledged"

        axis.wait(timeout_s=5.0)
        assert axis.position == pytest.approx(0.0)


def test_a_drive_fault_surfaces_with_its_error_code(axis: Axis, master: FakeMaster) -> None:
    axis.enable(timeout_s=2.0)
    master.slaves[0].inject_fault(error_code=0x8182)
    time.sleep(SETTLE_S)

    assert axis.error
    assert axis.error_id == 0x8182
    assert axis.state is AxisState.ERROR_STOP


def test_reset_clears_a_fault_and_the_held_enable_restores_operation(
    axis: Axis, master: FakeMaster
) -> None:
    axis.enable(timeout_s=2.0)
    master.slaves[0].inject_fault()
    time.sleep(SETTLE_S)
    assert axis.error

    axis.reset()
    time.sleep(SETTLE_S)

    assert not axis.error
    assert axis.error_id == 0
    assert axis.enabled, "enable is a level, so clearing the fault re-energises"
    assert axis.move_absolute(1.0, velocity=50.0, timeout_s=5.0) == pytest.approx(1.0)


def test_a_fault_mid_move_surfaces_from_wait(axis: Axis, master: FakeMaster) -> None:
    axis.enable(timeout_s=2.0)
    axis.move_absolute(90.0, velocity=0.5, wait=False)
    time.sleep(SETTLE_S)
    master.slaves[0].inject_fault(error_code=0x4210)

    from cooethercat.axis import AxisError

    with pytest.raises(AxisError, match="fault 16912 during move"):
        axis.wait(timeout_s=2.0)


def test_stop_halts_an_in_flight_move(axis: Axis) -> None:
    axis.enable(timeout_s=2.0)
    axis.move_absolute(90.0, velocity=1.0, wait=False)
    time.sleep(SETTLE_S)

    axis.stop()
    time.sleep(SETTLE_S)
    halted_at = axis.position

    assert abs(halted_at) < 90.0
    assert axis.state in (AxisState.STOPPING, AxisState.STANDSTILL)


def test_status_reads_cost_no_ethercat_traffic(enabled_axis: Axis, master: FakeMaster) -> None:
    slave = master.slaves[0]
    reads_before = len(slave.written_objects)
    sdo_calls: list[tuple[int, int]] = []
    original = slave.sdo_read

    def counted(index: int, subindex: int) -> bytes:
        sdo_calls.append((index, subindex))
        return original(index, subindex)

    slave.sdo_read = counted  # type: ignore[method-assign]
    try:
        for _ in range(20):
            enabled_axis.status()
    finally:
        slave.sdo_read = original  # type: ignore[method-assign]

    assert sdo_calls == [], "status must be served from the process-data image"
    assert len(slave.written_objects) == reads_before


def test_backend_serves_every_contract_field(master: FakeMaster) -> None:
    topology = build_topology()
    backend = SoemBackend(topology, bus=build_bus(master), period_s=PERIOD_S)
    backend.open()
    try:
        drive = backend.drives["epos1"]
        assert set(drive.status_snapshot()) == set(STATUS)
        assert set(drive.settings_snapshot()) == set(SETTINGS)

        config = topology.axes[0]
        every = backend.read_fields(config, tuple(STATUS) + tuple(SETTINGS))
        assert set(every) == set(STATUS) | set(SETTINGS)
    finally:
        backend.close()


def test_velocity_mode_is_refused_by_this_backend(axis: Axis) -> None:
    with pytest.raises(NotImplementedError, match="profile position and homing"):
        axis.set("mode", int(MoveMode.VELOCITY))


def test_axes_are_driven_independently() -> None:
    topology = build_topology(axis_count=2)
    master = FakeMaster([FakeEPOS4Slave(name="EPOS1"), FakeEPOS4Slave(name="EPOS2")])

    with open_soem(topology, bus=build_bus(master), period_s=PERIOD_S) as axes:
        first, second = axes.axis("EPOS1"), axes.axis("EPOS2")
        first.enable(timeout_s=2.0)
        second.enable(timeout_s=2.0)

        first.move_absolute(6.0, velocity=50.0, timeout_s=5.0)

        assert first.position == pytest.approx(6.0)
        assert second.position == pytest.approx(0.0)
        assert second.enabled


def test_close_de_energises_every_drive(master: FakeMaster) -> None:
    topology = build_topology()
    backend = SoemBackend(topology, bus=build_bus(master), period_s=PERIOD_S)
    backend.open()
    config = topology.axes[0]
    backend.write_field(config, "enable", True)
    time.sleep(SETTLE_S)
    assert backend.read_field(config, "enabled")

    backend.close()

    assert master.slaves[0].last_controlword == 0
    assert master.state == int(EtherCATState.SAFEOP)


def test_unknown_fields_are_rejected(master: FakeMaster) -> None:
    topology = build_topology()
    backend = SoemBackend(topology, bus=build_bus(master), period_s=PERIOD_S)
    backend.open()
    try:
        config = topology.axes[0]
        with pytest.raises(KeyError, match="not a settable field"):
            backend.write_field(config, "position", 1.0)
        with pytest.raises(KeyError, match="not an action"):
            backend.fire_trigger(config, "jump")
    finally:
        backend.close()


def test_an_axis_not_open_on_the_backend_is_rejected(master: FakeMaster) -> None:
    topology = build_topology()
    backend = SoemBackend(topology, bus=build_bus(master), period_s=PERIOD_S)
    backend.open()
    try:
        stranger: Any = build_topology(axis_count=2).axes[1]
        with pytest.raises(KeyError, match="not open on this backend"):
            backend.read_field(stranger, "position")
    finally:
        backend.close()


BOARD_TEMPERATURE = (0x3201, 1)


def test_coe_read_decodes_by_type(axis: Axis, master: FakeMaster) -> None:
    master.slaves[0].raw_objects[BOARD_TEMPERATURE] = struct.pack("<h", -125)
    assert axis.read_object(*BOARD_TEMPERATURE, CoeType.INT16) == -125
    master.slaves[0].raw_objects[BOARD_TEMPERATURE] = struct.pack("<H", 0xFF83)
    assert axis.read_object(*BOARD_TEMPERATURE, CoeType.UINT16) == 0xFF83


def test_a_failed_coe_read_is_a_connection_error(
    axis: Axis, master: FakeMaster, monkeypatch: pytest.MonkeyPatch
) -> None:
    def refuse(index: int, subindex: int) -> bytes:
        raise pysoem.SdoError(0, index, subindex, 0x05040000, "SDO protocol timed out")

    monkeypatch.setattr(master.slaves[0], "sdo_read", refuse)
    with pytest.raises(ConnectionError, match="EPOS1: SDO 0x3201:1"):
        axis.read_object(*BOARD_TEMPERATURE, CoeType.INT16)


def test_controller_state_follows_the_bus(master: FakeMaster) -> None:
    backend = SoemBackend(build_topology(), bus=build_bus(master), period_s=PERIOD_S)
    assert backend.controller_state() is ControllerState.STOP
    backend.open()
    try:
        assert backend.controller_state() is ControllerState.RUN
        master.slaves[0].state = int(EtherCATState.SAFEOP)
        assert backend.controller_state() is ControllerState.CONFIG
    finally:
        backend.close()
    assert backend.controller_state() is ControllerState.STOP


def test_every_axis_reads_from_the_process_image() -> None:
    master = FakeMaster([FakeEPOS4Slave(name="EPOS1"), FakeEPOS4Slave(name="EPOS2")])
    with open_soem(build_topology(axis_count=2), bus=build_bus(master), period_s=PERIOD_S) as axes:
        snapshot = axes.backend.read_axes_fields(
            [axis.config for axis in axes], ("position", "enabled"))
        assert set(snapshot) == {"EPOS1", "EPOS2"}
        assert snapshot["EPOS2"] == {"position": 0.0, "enabled": False}
