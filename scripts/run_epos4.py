#!/usr/bin/env python3
"""

Examples
--------
Scan the bus:
    sudo python scripts/run_epos4.py --ifname enp4s0 scan

Read drive status:
    sudo python scripts/run_epos4.py --ifname enp4s0 --slave 0 status

Enable the drive:
    sudo python scripts/run_epos4.py --ifname enp4s0 --slave 0 enable

Move using SDO profile-position mode:
    sudo python scripts/run_epos4.py --ifname enp4s0 --slave 0 move-sdo --position 10000 --velocity 5000 --acceleration 10000 --yes

Read an SDO:
    sudo python scripts/run_epos4.py --ifname enp4s0 --slave 0 read-sdo --index 0x6041 --subindex 0 --fmt H
"""

from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


import argparse
import logging
import sys
import time
from typing import Any


def _parse_int(value: str) -> int:
    """Parse decimal or 0x-prefixed integer strings."""
    return int(value, 0)


def _load_library():
    """Import cooethercat with compatibility for either epos4.py or maxon_epos4.py."""
    from cooethercat.bus import EtherCATBus, EtherCATState
    from cooethercat.cia402 import OperatingMode

    try:
        from cooethercat.epos4 import MaxonEPOS4
    except ImportError:
        from cooethercat.epos4 import MaxonEPOS4

    try:
        from cooethercat.pdo import PDOLoop
    except ImportError:
        PDOLoop = None

    return EtherCATBus, EtherCATState, OperatingMode, MaxonEPOS4, PDOLoop


def _ethercat_state(EtherCATState: Any, name: str) -> int:
    """Resolve a network state across slightly different enum naming styles."""
    candidates = {
        "init": ("INIT",),
        "preop": ("PREOP", "PRE_OP"),
        "safeop": ("SAFEOP", "SAFE_OP"),
        "op": ("OP", "OPERATIONAL"),
    }[name]

    for attr in candidates:
        if hasattr(EtherCATState, attr):
            return getattr(EtherCATState, attr)
    raise AttributeError(f"Could not resolve EtherCATState for {name!r}")


def _mode(OperatingMode: Any, value: str):
    """Resolve an operating mode by common name or integer value."""
    normalized = value.strip().upper().replace("-", "_")
    aliases = {
        "PPM": "PROFILE_POSITION",
        "PROFILE_POSITION_MODE": "PROFILE_POSITION",
        "PVM": "PROFILE_VELOCITY",
        "PROFILE_VELOCITY_MODE": "PROFILE_VELOCITY",
        "HOMING_MODE": "HOMING",
        "HMM": "HOMING",
        "CSP": "CYCLIC_SYNCHRONOUS_POSITION",
        "CSV": "CYCLIC_SYNCHRONOUS_VELOCITY",
        "CST": "CYCLIC_SYNCHRONOUS_TORQUE",
    }
    normalized = aliases.get(normalized, normalized)

    if hasattr(OperatingMode, normalized):
        return getattr(OperatingMode, normalized)
    return OperatingMode(int(value, 0))


def open_bus(args):
    EtherCATBus, EtherCATState, OperatingMode, MaxonEPOS4, PDOLoop = _load_library()
    bus = EtherCATBus(args.ifname)
    bus.open()
    return bus, EtherCATState, OperatingMode, MaxonEPOS4, PDOLoop


def make_drive(args, bus, MaxonEPOS4):
    return MaxonEPOS4(bus, slave_index=args.slave, name=args.name or f"epos4-{args.slave}")


def cmd_scan(args) -> int:
    bus, *_ = open_bus(args)
    try:
        slaves = bus.scan()
        print(f"Found {len(slaves)} EtherCAT slave(s)")
        for slave in slaves:
            print(
                f"[{slave.index}] name={slave.name!r} "
                f"manufacturer={slave.manufacturer_id} product={slave.product_id} "
                f"revision={slave.revision} state={slave.state}"
            )
        return 0
    finally:
        bus.close()


def cmd_status(args) -> int:
    bus, _, _, MaxonEPOS4, _ = open_bus(args)
    try:
        bus.scan()
        drive = make_drive(args, bus, MaxonEPOS4)
        info = drive.debug_info() if args.debug else drive.info()
        for key, value in info.items():
            print(f"{key}: {value}")
        return 0
    finally:
        bus.close()


def cmd_net_state(args) -> int:
    bus, EtherCATState, *_ = open_bus(args)
    try:
        bus.scan()
        state = _ethercat_state(EtherCATState, args.state)
        bus.set_master_state(state)
        if args.wait:
            bus.assert_master_state(state, timeout_us=args.timeout_us)
        print(f"Network state requested: {args.state}")
        print(f"Current slave states: {bus.read_states()}")
        return 0
    finally:
        bus.close()


def cmd_drive_command(args) -> int:
    bus, _, _, MaxonEPOS4, _ = open_bus(args)
    try:
        bus.scan()
        drive = make_drive(args, bus, MaxonEPOS4)
        result = getattr(drive, args.method)()
        print(f"{args.method} -> {result}")
        print(f"statusword -> {drive.statusword}")
        return 0
    finally:
        bus.close()


def cmd_mode(args) -> int:
    bus, _, OperatingMode, MaxonEPOS4, _ = open_bus(args)
    try:
        bus.scan()
        drive = make_drive(args, bus, MaxonEPOS4)
        mode = _mode(OperatingMode, args.mode)
        drive.set_operating_mode(mode)
        print(f"mode -> {drive.read_operating_mode()}")
        return 0
    finally:
        bus.close()


def cmd_move_sdo(args) -> int:
    if not args.yes:
        print("Refusing to move without --yes. Verify motor setup, limits, and direction first.", file=sys.stderr)
        return 2

    bus, _, _, MaxonEPOS4, _ = open_bus(args)
    try:
        bus.scan()
        drive = make_drive(args, bus, MaxonEPOS4)
        drive.profile_position_move_sdo(
            position=args.position,
            velocity=args.velocity,
            acceleration=args.acceleration,
            deceleration=args.deceleration,
            absolute=not args.relative,
        )
        print("Move command sent")
        print(f"statusword -> {drive.statusword}")
        return 0
    finally:
        bus.close()


def cmd_read_sdo(args) -> int:
    bus, *_ = open_bus(args)
    try:
        bus.scan()
        value = bus.read_sdo(args.slave, args.index, args.subindex, args.fmt)
        print(value)
        return 0
    finally:
        bus.close()


def cmd_write_sdo(args) -> int:
    if not args.yes:
        print("Refusing to write SDO without --yes.", file=sys.stderr)
        return 2

    bus, *_ = open_bus(args)
    try:
        bus.scan()
        bus.write_sdo(args.slave, args.index, args.subindex, args.fmt, args.value)
        print("SDO write complete")
        return 0
    finally:
        bus.close()


def cmd_monitor_pdo(args) -> int:
    bus, EtherCATState, _, MaxonEPOS4, PDOLoop = open_bus(args)
    if PDOLoop is None:
        print("cooethercat.pdo.PDOLoop is not available in this package.", file=sys.stderr)
        bus.close()
        return 2

    loop = None
    try:
        bus.scan()
        bus.configure()
        drive = make_drive(args, bus, MaxonEPOS4)
        loop = PDOLoop(bus, period_s=args.period_s)
        loop.add_device(drive)

        # Start cyclic exchange before requesting OP so the drive does not drop
        # back to SAFE-OP because of a missing PDO cycle.
        loop.start()
        op_state = _ethercat_state(EtherCATState, "op")
        bus.set_master_state(op_state)
        bus.assert_master_state(op_state, timeout_us=args.timeout_us)

        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            print(
                f"status={drive.statusword_pdo} "
                f"pos={drive.position_pdo} "
                f"vel={drive.velocity_pdo} "
                f"following_error={drive.following_error_pdo}"
            )
            time.sleep(args.print_period_s)
        return 0
    finally:
        if loop is not None:
            loop.stop()
        try:
            safeop_state = _ethercat_state(EtherCATState, "safeop")
            bus.set_master_state(safeop_state)
        except Exception:
            pass
        bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run and test the cooethercat EPOS4 library")
    parser.add_argument("--ifname", required=True, help="EtherCAT network interface, e.g. enp4s0")
    parser.add_argument("--slave", type=int, default=0, help="EtherCAT slave index")
    parser.add_argument("--name", default=None, help="Friendly drive name")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")

    sub = parser.add_subparsers(required=True)

    p = sub.add_parser("scan", help="Scan EtherCAT slaves")
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser("status", help="Read EPOS4 status via SDO")
    p.add_argument("--debug", action="store_true", help="Read extended debug info")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("net-state", help="Request an EtherCAT network state")
    p.add_argument("state", choices=["init", "preop", "safeop", "op"])
    p.add_argument("--wait", action="store_true", help="Wait for requested state")
    p.add_argument("--timeout-us", type=int, default=500_000)
    p.set_defaults(func=cmd_net_state)

    for command, method in {
        "fault-reset": "fault_reset",
        "enable": "enable",
        "shutdown": "shutdown",
        "disable-operation": "disable_operation",
        "disable-voltage": "disable_voltage",
        "quick-stop": "quick_stop",
    }.items():
        p = sub.add_parser(command, help=f"Run drive.{method}()")
        p.set_defaults(func=cmd_drive_command, method=method)

    p = sub.add_parser("mode", help="Set operating mode")
    p.add_argument("mode", help="Mode name like PPM, HOMING, CSP, or numeric value")
    p.set_defaults(func=cmd_mode)

    p = sub.add_parser("move-sdo", help="Send a profile-position move using SDOs")
    p.add_argument("--position", type=int, required=True)
    p.add_argument("--velocity", type=int, required=True)
    p.add_argument("--acceleration", type=int, required=True)
    p.add_argument("--deceleration", type=int, default=None)
    p.add_argument("--relative", action="store_true")
    p.add_argument("--yes", action="store_true", help="Required safety confirmation")
    p.set_defaults(func=cmd_move_sdo)

    p = sub.add_parser("read-sdo", help="Read a raw SDO")
    p.add_argument("--index", type=_parse_int, required=True)
    p.add_argument("--subindex", type=_parse_int, default=0)
    p.add_argument("--fmt", required=True, help="struct format, e.g. H, I, i, b")
    p.set_defaults(func=cmd_read_sdo)

    p = sub.add_parser("write-sdo", help="Write a raw SDO")
    p.add_argument("--index", type=_parse_int, required=True)
    p.add_argument("--subindex", type=_parse_int, default=0)
    p.add_argument("--fmt", required=True, help="struct format, e.g. H, I, i, b")
    p.add_argument("--value", type=_parse_int, required=True)
    p.add_argument("--yes", action="store_true", help="Required safety confirmation")
    p.set_defaults(func=cmd_write_sdo)

    p = sub.add_parser("monitor-pdo", help="Start PDO loop and print PDO inputs")
    p.add_argument("--period-s", type=float, default=0.002)
    p.add_argument("--duration-s", type=float, default=5.0)
    p.add_argument("--print-period-s", type=float, default=0.25)
    p.add_argument("--timeout-us", type=int, default=500_000)
    p.set_defaults(func=cmd_monitor_pdo)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
