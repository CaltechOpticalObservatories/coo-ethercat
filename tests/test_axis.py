"""Tests for the backend-neutral axis API, exercised against the simulator."""

from __future__ import annotations

from typing import Any

import pytest

from cooethercat.axis import (
    ALL_FIELDS,
    SETTINGS,
    STATUS,
    TRIGGERS,
    Axis,
    AxisError,
    AxisSet,
    AxisState,
    Dimension,
    FieldKind,
    MoveMode,
)
from cooethercat.backends.sim import SimBackend, open_simulated
from cooethercat.config import TopologyConfig, parse_topology

TOPOLOGY: dict[str, Any] = {
    "name": "sim-bench",
    "backend": "sim",
    "transport": {"tick_s": 0.002},
    "defaults": {
        "units": {"name": "mm"},
        "limits": {"velocity": 20.0, "min_position": -50.0, "max_position": 50.0},
    },
    "axes": [{"name": "CSU-0"}, {"name": "CSU-1"}],
}


@pytest.fixture(name="topology")
def topology_fixture() -> TopologyConfig:
    return parse_topology(TOPOLOGY)


@pytest.fixture(name="axes")
def axes_fixture(topology: TopologyConfig) -> Any:
    with open_simulated(topology) as axis_set:
        yield axis_set


@pytest.fixture(name="axis")
def axis_fixture(axes: AxisSet) -> Axis:
    return axes.axis("CSU-0")


def test_simulator_models_every_contract_field(topology: TopologyConfig) -> None:
    backend = SimBackend(topology)
    config = topology.axes[0]

    for name, spec in ALL_FIELDS.items():
        if spec.kind is FieldKind.TRIGGER:
            backend.fire_trigger(config, name)
        else:
            assert backend.read_field(config, name) is not None


def test_field_tables_are_disjoint_and_complete() -> None:
    assert set(SETTINGS) | set(TRIGGERS) | set(STATUS) == set(ALL_FIELDS)
    assert len(SETTINGS) + len(TRIGGERS) + len(STATUS) == len(ALL_FIELDS)
    assert all(spec.kind is FieldKind.TRIGGER for spec in TRIGGERS.values())


def test_status_is_read_as_one_snapshot(axis: Axis, axes: AxisSet) -> None:
    calls: list[int] = []
    backend: Any = axes.backend
    original = backend.read_fields

    def counted(config: Any, fields: Any) -> Any:
        calls.append(len(tuple(fields)))
        return original(config, fields)

    backend.read_fields = counted
    try:
        assert set(axis.status()) == set(STATUS)
    finally:
        backend.read_fields = original

    assert calls == [len(STATUS)]


def test_unit_labels_come_from_the_axis_not_the_table() -> None:
    position = ALL_FIELDS["position"]
    velocity = ALL_FIELDS["velocity"]
    acceleration = ALL_FIELDS["acceleration"]

    assert position.unit_label("mm") == "mm"
    assert position.unit_label("deg") == "deg"
    assert velocity.unit_label("mm") == "mm/s"
    assert acceleration.unit_label("deg") == "deg/s^2"
    assert ALL_FIELDS["state"].unit_label("mm") is None
    assert ALL_FIELDS["state"].dimension is Dimension.NONE


def test_axis_set_exposes_axes_by_name(axes: AxisSet) -> None:
    assert len(axes) == 2
    assert axes.names == ("CSU-0", "CSU-1")
    assert axes.axis("csu-0") is axes.axis("CSU-0")
    with pytest.raises(AxisError, match="known axes"):
        axes.axis("CSU-9")


def test_enable_then_disable_reports_state(axis: Axis) -> None:
    assert not axis.enabled
    assert axis.state is AxisState.DISABLED

    axis.enable()
    assert axis.enabled
    assert axis.state is AxisState.STANDSTILL

    axis.disable()
    assert not axis.enabled
    assert axis.state is AxisState.DISABLED


def test_absolute_and_relative_moves_reach_their_targets(axis: Axis) -> None:
    axis.enable()

    assert axis.move_absolute(10.0, velocity=20.0) == pytest.approx(10.0)
    assert axis.state is AxisState.STANDSTILL

    assert axis.move_absolute(-4.0, velocity=20.0) == pytest.approx(-4.0)
    assert axis.move_relative(5.0, velocity=20.0) == pytest.approx(1.0)
    assert axis.position == pytest.approx(1.0)


def test_move_requires_an_enabled_fault_free_axis(axis: Axis, axes: AxisSet) -> None:
    with pytest.raises(AxisError, match="not enabled"):
        axis.move_absolute(1.0)

    axis.enable()
    backend: Any = axes.backend
    backend.inject_fault("CSU-0", error_id=0x8182)

    assert axis.error
    assert axis.error_id == 0x8182
    with pytest.raises(AxisError, match="in fault 33154"):
        axis.move_absolute(1.0)


def test_reset_clears_a_fault_and_restores_motion(axis: Axis, axes: AxisSet) -> None:
    axis.enable()
    backend: Any = axes.backend
    backend.inject_fault("CSU-0", error_id=7)
    assert axis.state is AxisState.ERROR_STOP

    axis.reset()
    assert not axis.error
    assert axis.error_id == 0

    axis.enable()
    assert axis.move_absolute(2.0, velocity=20.0) == pytest.approx(2.0)


def test_stop_halts_an_in_flight_move_short_of_target(axis: Axis) -> None:
    axis.enable()
    axis.move_absolute(40.0, velocity=1.0, wait=False)
    axis.stop()

    assert not axis.moving
    assert axis.state is AxisState.STANDSTILL
    assert abs(axis.position) < 40.0


def test_home_zeroes_the_axis_and_counts_a_command(axis: Axis) -> None:
    axis.enable()
    axis.move_absolute(6.0, velocity=20.0)
    sequence_before = axis.get("sequence")

    axis.home()
    assert axis.position == pytest.approx(0.0)
    assert axis.get("sequence") > sequence_before


def test_wait_times_out_rather_than_blocking_forever(axis: Axis) -> None:
    axis.enable()
    axis.move_absolute(45.0, velocity=0.1, wait=False)

    with pytest.raises(AxisError, match="did not complete within"):
        axis.wait(timeout_s=0.05)


def test_a_fault_during_a_move_surfaces_from_wait(axis: Axis, axes: AxisSet) -> None:
    axis.enable()
    axis.move_absolute(45.0, velocity=1.0, wait=False)
    backend: Any = axes.backend
    backend.inject_fault("CSU-0", error_id=42)

    with pytest.raises(AxisError, match="fault 42 during move"):
        axis.wait(timeout_s=1.0)


def test_enable_times_out_when_the_drive_stays_faulted(axis: Axis, axes: AxisSet) -> None:
    backend: Any = axes.backend
    backend.inject_fault("CSU-0", error_id=1)

    with pytest.raises(AxisError, match="enable timed out"):
        axis.enable(timeout_s=0.05)


def test_configured_travel_limits_reject_a_target(axis: Axis) -> None:
    axis.enable()

    with pytest.raises(AxisError, match="outside the configured travel"):
        axis.move_absolute(80.0, velocity=10.0)

    axis.move_absolute(45.0, velocity=20.0)
    with pytest.raises(AxisError, match="outside the configured travel"):
        axis.move_relative(10.0, velocity=10.0)


def test_configured_velocity_limit_rejects_a_fast_move(axis: Axis) -> None:
    axis.enable()

    with pytest.raises(AxisError, match="exceeds the configured limit"):
        axis.move_absolute(1.0, velocity=25.0)
    with pytest.raises(AxisError, match="exceeds the configured limit"):
        axis.move_absolute(1.0, velocity=-25.0)


def test_status_returns_every_readback(axis: Axis) -> None:
    axis.enable()
    status = axis.status()

    assert set(status) == set(STATUS)
    assert status["enabled"] is True


def test_triggers_are_not_readable_and_status_is_not_writable(axis: Axis) -> None:
    with pytest.raises(AxisError, match="is an action and cannot be read"):
        axis.get("move")
    with pytest.raises(AxisError, match="not settable"):
        axis.set("position", 1.0)
    with pytest.raises(AxisError, match="unknown field"):
        axis.get("nonsense")


def test_axes_move_independently(axes: AxisSet) -> None:
    first, second = axes.axis("CSU-0"), axes.axis("CSU-1")
    first.enable()
    second.enable()

    first.move_absolute(5.0, velocity=20.0)
    assert first.position == pytest.approx(5.0)
    assert second.position == pytest.approx(0.0)


def test_move_mode_is_recorded_for_the_backend(axis: Axis) -> None:
    axis.enable()
    axis.move_relative(3.0, velocity=20.0)

    assert axis.get("mode") == int(MoveMode.RELATIVE)
