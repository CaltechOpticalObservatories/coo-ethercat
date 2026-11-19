# coo-ethercat

EtherCAT motion control for LRIS-2, principally Maxon EPOS4 (CiA 402) drives.

The library is arranged in two layers that can be used independently:

- **Axis API.** A backend-neutral view of a set of axes, described by a topology
  config file and commanded in engineering units. Use this for anything that
  should outlive a choice of master.
- **Soft-master stack.** A pysoem EtherCAT master with a vendor-neutral CiA 402
  drive layer, EPOS4 specifics, and a cyclic PDO loop. Use this for bring-up,
  commissioning and object-level work.

## Install

```sh
pip install -e .          # pysoem and pyyaml
pip install -e '.[dev]'   # pytest, for the test suite
pip install -e '.[gui]'   # PyQt5, for the EPOS4 control GUI
```

EtherCAT needs a dedicated NIC cabled straight to a drive. The traffic is not
routable and must not cross a switch.

## Topology configs

One YAML file describes one topology: which master drives it, how to reach that
master, and the axes it carries. Shipped examples:

| File | Bench |
|---|---|
| `configs/lab-137.yaml` | single EPOS4 through the CX2030 TwinCAT master |
| `configs/lab-137-pysoem.yaml` | the same drive under a pysoem soft master |
| `configs/csu-demo.yaml` | the 23 CSU axes on CX-LAB01 |

```yaml
name: csu-demo
backend: twincat            # pysoem | twincat | sim
transport:
  host: 192.168.30.99
  ams_net_id: 5.57.12.96.1.1
defaults:
  units: {name: mm}
  limits: {velocity: 5.0}
axes:
  - {name: CSU-0, plc_index: 1}
```

An axis carries up to three addresses, because on the CSU bench they disagree.
`slave_index` is pysoem enumeration order, `station_address` is what CoE over
ADS addresses, and `plc_index` is the slot in the PLC symbol array. The NC axis
number behind a PLC slot is an XAE-side link and never appears here.

`units.increments_per_unit` is required only for backends that command in
encoder increments; a TwinCAT NC axis is scaled in the PLC project and already
reports engineering units. Only `limits.velocity` is mandatory, so every axis
has a speed ceiling. Unknown keys are rejected rather than ignored.

```python
from cooethercat import load_topology

topology = load_topology("configs/csu-demo.yaml")
print(topology.axis_names)
print(topology.axis("CSU-0").limits.velocity)
```

## Axis API

Every backend answers the same field names, so a script does not change when the
master does. Positions, velocities and accelerations are in the axis's
configured units.

```python
from cooethercat import load_topology, open_simulated

topology = load_topology("configs/csu-demo.yaml")
with open_simulated(topology) as axes:
    axis = axes.axis("CSU-0")
    axis.enable()
    axis.move_absolute(12.5, velocity=5.0)
    print(axis.status())
    axis.disable()
```

Configured travel and velocity limits are enforced before a move is commanded.
`move_absolute` and `move_relative` block until the move completes and raise
`AxisError` on a fault or a timeout; pass `wait=False` to return immediately and
call `wait()` later.

The field tables in `cooethercat/axis.py` are the interface contract, and the
libby keyword names derive from them. `sim`, `twincat` and `pysoem` are
available, and a script runs unchanged on any of them.

The `twincat` backend talks ADS to the PLC motion server (`GVL_Axes.Axis[N]`) and
needs the `twincat` extra (`pip install cooethercat[twincat]`). The `pysoem`
backend makes Python the EtherCAT master on a dedicated NIC, driving each
`MaxonEPOS4` through the soft-master stack below; it needs raw socket access
(`CAP_NET_RAW`). On both, `Axis.read_object` reads an EPOS4 CoE object, for drive
telemetry such as board temperature.

```python
from cooethercat import load_topology, open_soem

with open_soem(load_topology("configs/lab-137-pysoem.yaml")) as axes:
    axes.axis("EPOS1").enable()
```

## Soft-master stack

```python
from cooethercat import EtherCATBus, MaxonEPOS4, OperatingMode

with EtherCATBus("enp4s0") as bus:
    bus.scan()
    bus.configure()

    drive = MaxonEPOS4(bus, slave_index=0, name="EPOS1")
    drive.enable()
    drive.profile_position_move_sdo(
        position=10_000, velocity=4_000, acceleration=10_000
    )
    print(drive.info())
```

For cyclic control, register drives with a `PDOLoop` and queue commands into the
process-data image instead of writing SDOs:

```python
from cooethercat import PDOLoop

loop = PDOLoop(bus, period_s=0.002)
loop.add_device(drive)
loop.start()
drive.queue_profile_position_pdo(10_000, velocity=4_000, acceleration=10_000)
```

Profile position and homing modes tolerate host scheduling jitter because the
drive generates the trajectory internally. Cyclic synchronous modes need a
disciplined distributed clock and are not supported here.

## Layout

| Module | Purpose |
|---|---|
| `config.py` | topology config parsing and validation |
| `axis.py` | backend-neutral axis API and the field contract |
| `backends/` | one module per master implementation |
| `bus.py` | pysoem master wrapper: scan, SDO, PDO buffers, NMT states |
| `cia402.py` | vendor-neutral CiA 402 drive, states and modes |
| `epos4.py` | EPOS4 object dictionary, homing and diagnostics |
| `pdo.py` | PDO layouts and the cyclic loop |
| `maxon_errors.py` | EPOS4 error code lookup |
| `scripts/run_epos4.py` | single-drive command line tool |
| `gui/` | PyQt5 control panel |

## Tests

```sh
pytest
```

The suite runs against the simulator, a fake ADS connection and a fake pysoem
master, and needs no hardware.

## Notes

- Tested on the [EPOS4 Micro 24/5 EtherCAT](https://www.maxongroup.com/maxon/view/product/654731).
- CiA 402 gates actions behind a combination of device state, network state and
  operating mode. A [condensed CoE CiA 402 guide](https://caltech.sharepoint.com/:b:/r/sites/coo/LRIS-2/Shared%20Documents/LRIS-2%20-%20Subsystems%20%5BL3%5D/Maxon%20motion%20control/EPOS4%20Micro%2024%205%20EtherCAT%20Quick%20Start%20with%20Merged%20and%20Condensed%20Maxon%20Documentation.pdf?csf=1&web=1&e=m6LDQB)
  covers the EPOS4 specifics.
- EPOS4 tuning lives in the controller's non-volatile memory and is set once in
  EPOS Studio over USB. A master can push a full parameter set over SDO.
