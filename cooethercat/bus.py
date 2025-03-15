from collections import OrderedDict
from typing import Iterable

import pysoem
import struct
from enum import Enum
from logging import getLogger
import netifaces
from threading import RLock
import functools

from .helpers import STATUSWORD_STATE_BITMASK

#TODO I'm increasingly of the opintion that having both EthercatBus and EPOS4Bus does not make sense and is a ppoor strategy
class EthercatBus:

    @staticmethod
    def bus_locked(method):
        @functools.wraps(method)
        def wrapper(self, *args, **kwargs):
            with self.lock:
                return method(self, *args, **kwargs)

        return wrapper

    def __init__(self, ifname: str):
        self.ifname = ifname
        self.pysoem_master = pysoem.Master()
        self.lock = RLock()

    #TODO replace these with decorators that automate this, Bus user shall not need to worry about interface state.
    def open(self):
        """Opens the network interface with the given interface name."""
        try:
            if self.ifname in netifaces.interfaces():
                address_families = netifaces.ifaddresses(self.ifname)
                if not netifaces.AF_LINK in address_families:
                    raise RuntimeError(f"Interface {self.ifname} is not UP.")
            else:
                raise RuntimeError(f"Interface {self.ifname} not found.")
        except Exception as e:
            raise e
        self.pysoem_master.open(self.ifname)  # pysoem doesn't return anything, so we can't check if it was successful

    def close(self):
        """Closes the network interface."""
        self.pysoem_master.close()

    ### SDO methods ###
    @bus_locked
    def SDORead(self, slaveInstance, address: Iterable):
        """Reads a Service Data Object (SDO) from a slave."""
        slave = self.pysoem_master.slaves[slaveInstance.node]
        index, subIndex, packFormat, *_ = address
        response = struct.unpack('<' + packFormat, slave.sdo_read(index, subIndex))
        return response[0] if len(response) == 1 else response

    @bus_locked
    def SDOWrite(self, slaveInstance, address: Iterable, data, completeAccess=False):
        """Writes a Service Data Object (SDO) to a slave."""
        slave = self.pysoem_master.slaves[slaveInstance.node]
        index, subIndex, packFormat, *_ = address
        slave.sdo_write(index, subIndex, struct.pack('<' + packFormat, data), ca=completeAccess)

    ### Slave configuration methods ###
    @bus_locked
    def initialize_slaves(self):
        """Creates slave objects for each slave and assigns them to self.slaves. Returns the number of slaves."""
        n = self.pysoem_master.config_init()
        getLogger(__name__).info(self.slave_info(as_string=True))
        return n

    def slave_info(self, as_string=False):
        """Gathers detailed information for each slave."""
        keys = ('id', 'name', 'manufacturer', 'revision', 'state')
        attribs = ('id', 'name', 'man', 'rev', 'state')
        defaults = ('N/A', '""', 'N/A', 'N/A', 'N/A')
        data = OrderedDict()
        for slave_ndx, slave in enumerate(self.pysoem_master.slaves):
            # Inspect the available attributes using dir()
            data[slave_ndx] = OrderedDict()
            data[slave_ndx]['attributes'] = [x for x in dir(slave) if not x.startswith('__')]
            for key, attrib, default in zip(keys, attribs, defaults):
                try:
                    data[slave_ndx][key] = getattr(slave, attrib, default)  # Revision number
                except Exception as e:
                    data[slave_ndx][key] = f"<Error {e} for '{attrib}' attribute>"

        if as_string:
            fmt = ("Available attributes: {attributes}\n"
                   "ID: {id} - Name: {name}, Manufacturer ID: {manufacturer}, Revision: {revision}, State: {state}")
            string = ("Slave Information:"+
             '\n----\n'.join( [fmt.format(**rec) for rec in data.values()])+
             f'\nTotal slaves: {len(self.pysoem_master.slaves)}')

        return string if as_string else data

    @bus_locked
    def configureSlaves(self):
        """Configures the slaves"""
        self.pysoem_master.config_map()

    @bus_locked
    def setWatchDog(self, slaveInstance, timeout: float):
        """Sets the watchdog timeout for the slave.
        Inputs:
            slave: high level master.py.slave instance
            timeout: float
                The timeout in milliseconds
        """
        self.pysoem_master.slaves[slaveInstance.node].set_watchdog('pdi', timeout)
        self.pysoem_master.slaves[slaveInstance.node].set_watchdog('processdata', timeout)

    ### Network state methods ###
    # 1. Apply to all slaves
    @bus_locked
    def assertNetworkWideState(self, state: int) -> bool:
        return self.pysoem_master.state_check(state) == state

    @bus_locked
    def getNetworkWideState(self):
        self.pysoem_master.read_state()  # Cursed abstraction by pysoem, slaves can't refresh their own state :(
        states = []
        for i, slave in enumerate(self.pysoem_master.slaves):
            states += [slave.state]
        return states

    @bus_locked
    def setNetworkWideState(self, state: Enum| int):
        state = state.value if isinstance(state, Enum) else state
        self.pysoem_master.state = state
        self.pysoem_master.write_state()

    # 2. Apply to individual slaves
    def assertNetworkState(self, slaveInstance, state: int|Enum) -> bool:
        state = state.value if isinstance(state, Enum) else state
        return self.getNetworkState(slaveInstance) == state

    @bus_locked
    def getNetworkState(self, slaveInstance):
        self.pysoem_master.read_state()  # Cursed, the slaves can't refresh their own state
        return self.pysoem_master.slaves[slaveInstance.node].state

    @bus_locked
    def setNetworkState(self, slaveInstance, state: int | Enum):
        state = state.value if isinstance(state, Enum) else state
        self.pysoem_master.slaves[slaveInstance.node].state = state
        self.pysoem_master.slaves[slaveInstance.node].write_state()

    ### PDO methods ###
    @bus_locked
    def sendProcessData(self):
        self.pysoem_master.send_processdata()

    @bus_locked
    def receiveProcessData(self):
        self.pysoem_master.receive_processdata(timeout=2000)

    def addPDOMessage(self, slaveInstance, packFormat, data):
        """Adds a PDO message to the slave's PDO buffer."""
        self.pysoem_master.slaves[slaveInstance.node].output = struct.pack('<' + packFormat, *data)

    def __del__(self):
        self.pysoem_master.close()
