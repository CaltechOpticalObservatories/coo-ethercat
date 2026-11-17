"""Tests for topology configuration parsing and validation."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import pytest

from cooethercat.config import (
    Backend,
    ConfigError,
    Limits,
    PysoemTransport,
    SimTransport,
    TwincatTransport,
    Units,
    load_topology,
    parse_topology,
)

CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs"

MINIMAL: dict[str, Any] = {
    "name": "bench",
    "backend": "pysoem",
    "transport": {"ifname": "eth0"},
    "defaults": {
        "units": {"name": "deg", "increments_per_unit": 100.0},
        "limits": {"velocity": 10.0},
    },
    "axes": [{"name": "AX1", "slave_index": 0}],
}


def build(**overrides: Any) -> dict[str, Any]:
    """Return a deep copy of the minimal valid topology with overrides applied."""
    data = copy.deepcopy(MINIMAL)
    data.update(copy.deepcopy(overrides))
    return data


def test_every_shipped_config_loads() -> None:
    configs = sorted(CONFIG_DIR.glob("*.yaml"))
    assert configs, "no shipped configs found"
    for path in configs:
        topology = load_topology(path)
        assert topology.axes


def test_lab_137_twincat_transport() -> None:
    topology = load_topology(CONFIG_DIR / "lab-137.yaml")

    assert topology.backend is Backend.TWINCAT
    assert isinstance(topology.transport, TwincatTransport)
    assert topology.transport.host == "172.16.10.60"
    assert topology.transport.master_net_id == "5.57.12.96.4.1"
    assert topology.transport.ams_port == 851

    axis = topology.axis("epos1")
    assert axis.name == "EPOS1"
    assert axis.plc_index == 1
    assert axis.require_station_address() == 1001


def test_lab_137_pysoem_pairs_with_the_twincat_config() -> None:
    twincat = load_topology(CONFIG_DIR / "lab-137.yaml")
    pysoem = load_topology(CONFIG_DIR / "lab-137-pysoem.yaml")

    assert isinstance(pysoem.transport, PysoemTransport)
    assert pysoem.axis_names == twincat.axis_names
    assert pysoem.axis("EPOS1").units == twincat.axis("EPOS1").units


def test_csu_demo_axis_slots_follow_the_documented_offset() -> None:
    topology = load_topology(CONFIG_DIR / "csu-demo.yaml")

    assert len(topology.axes) == 23
    for number, axis in enumerate(topology.axes):
        assert axis.name == f"CSU-{number}"
        assert axis.plc_index == number + 1


def test_csu_demo_has_units_without_encoder_scaling() -> None:
    axis = load_topology(CONFIG_DIR / "csu-demo.yaml").axis("CSU-0")

    assert axis.units.name == "mm"
    assert axis.units.increments_per_unit is None
    with pytest.raises(ConfigError, match="increments_per_unit"):
        axis.units.require_scaling()
    assert axis.require_station_address() == 1001


def test_axis_lookup_is_case_insensitive_and_reports_known_names() -> None:
    topology = parse_topology(build())

    assert topology.axis("ax1") is topology.axis("AX1")
    with pytest.raises(ConfigError, match="AX1"):
        topology.axis("nope")


def test_defaults_apply_and_axes_override_them() -> None:
    topology = parse_topology(
        build(
            axes=[
                {"name": "AX1", "slave_index": 0},
                {
                    "name": "AX2",
                    "slave_index": 1,
                    "units": {"name": "mm", "increments_per_unit": 5.0},
                    "limits": {"velocity": 1.0, "acceleration": 2.0},
                },
            ]
        )
    )

    assert topology.axis("AX1").units.name == "deg"
    assert topology.axis("AX1").limits.velocity == 10.0
    assert topology.axis("AX2").units.increments_per_unit == 5.0
    assert topology.axis("AX2").limits.acceleration == 2.0


def test_sim_backend_needs_no_addresses() -> None:
    topology = parse_topology(
        build(
            backend="sim",
            transport={"tick_s": 0.005},
            axes=[{"name": "AX1"}],
        )
    )

    assert isinstance(topology.transport, SimTransport)
    assert topology.transport.tick_s == 0.005
    assert topology.axis("AX1").slave_index is None


def test_units_round_trip_through_increments() -> None:
    units = Units(name="deg", increments_per_unit=2161.7778)

    assert units.to_increments(1.0) == 2162
    assert units.to_units(778240) == pytest.approx(360.0, abs=1e-3)


def test_limits_bound_travel_and_fall_back_for_deceleration() -> None:
    limits = Limits(velocity=5.0, acceleration=2.0, min_position=-1.0, max_position=1.0)

    assert limits.effective_deceleration == 2.0
    assert Limits(velocity=1.0).effective_deceleration is None
    assert limits.position_is_allowed(0.0)
    assert not limits.position_is_allowed(1.5)
    assert not limits.position_is_allowed(-1.5)
    assert Limits(velocity=1.0).position_is_allowed(1e9)


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"backend": "igh"}, "unknown backend"),
        ({"transport": {}}, "requires ifname"),
        ({"transport": {"if_name": "eth0"}}, "unknown key"),
        ({"axes": []}, "non-empty list"),
        ({"axes": [{"name": "AX1"}]}, "needs slave_index"),
        ({"name": 137}, "expected a string"),
    ],
)
def test_topology_level_errors(overrides: dict[str, Any], message: str) -> None:
    with pytest.raises(ConfigError, match=message):
        parse_topology(build(**overrides))


@pytest.mark.parametrize(
    ("axes", "message"),
    [
        ([{"name": "AX1", "slave_index": 0, "colour": "red"}], "unknown key"),
        ([{"name": "AX.1", "slave_index": 0}], "must not contain"),
        ([{"name": "motion%", "slave_index": 0}], "must not contain"),
        ([{"name": "", "slave_index": 0}], "must not be empty"),
        ([{"name": "AX1", "slave_index": -1}], "at least 0"),
        ([{"name": "AX1", "plc_index": 0, "slave_index": 0}], "at least 1"),
        ([{"name": "AX1", "slave_index": True}], "expected an integer"),
        (
            [{"name": "AX1", "slave_index": 0}, {"name": "ax1", "slave_index": 1}],
            "collide once lowercased",
        ),
        (
            [{"name": "AX1", "slave_index": 0}, {"name": "AX2", "slave_index": 0}],
            "share slave_index=0",
        ),
    ],
)
def test_axis_level_errors(axes: list[dict[str, Any]], message: str) -> None:
    with pytest.raises(ConfigError, match=message):
        parse_topology(build(axes=axes))


@pytest.mark.parametrize(
    ("defaults", "message"),
    [
        ({"units": {"name": "deg"}, "limits": {"velocity": 1.0}}, "increments_per_unit"),
        (
            {"units": {"name": "deg", "increments_per_unit": 0}, "limits": {"velocity": 1.0}},
            "above 0",
        ),
        (
            {"units": {"name": "deg", "increments_per_unit": 1.0}, "limits": {"velocity": -1}},
            "above 0",
        ),
        (
            {
                "units": {"name": "deg", "increments_per_unit": 1.0},
                "limits": {"velocity": 1.0, "min_position": 5.0, "max_position": 1.0},
            },
            "must be below",
        ),
        (
            {"units": {"name": "deg", "increments_per_unit": 1.0}, "limits": {}},
            "missing required key 'velocity'",
        ),
    ],
)
def test_units_and_limits_errors(defaults: dict[str, Any], message: str) -> None:
    with pytest.raises(ConfigError, match=message):
        parse_topology(build(defaults=defaults))


def test_pysoem_requires_encoder_scaling() -> None:
    data = build(
        backend="twincat",
        transport={"host": "10.0.0.1", "ams_net_id": "1.2.3.4.1.1"},
        defaults={"units": {"name": "mm"}, "limits": {"velocity": 1.0}},
        axes=[{"name": "AX1", "plc_index": 1}],
    )
    assert parse_topology(data).axis("AX1").units.increments_per_unit is None

    data["backend"] = "pysoem"
    data["transport"] = {"ifname": "eth0"}
    data["axes"] = [{"name": "AX1", "slave_index": 0}]
    with pytest.raises(ConfigError, match="increments_per_unit"):
        parse_topology(data)


def test_load_reports_a_missing_file_and_bad_yaml(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="cannot read"):
        load_topology(tmp_path / "absent.yaml")

    broken = tmp_path / "broken.yaml"
    broken.write_text("name: [unclosed\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_topology(broken)

    not_a_mapping = tmp_path / "list.yaml"
    not_a_mapping.write_text("- one\n- two\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="expected a mapping"):
        load_topology(not_a_mapping)
