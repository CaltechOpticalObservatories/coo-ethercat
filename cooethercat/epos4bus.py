import struct
import threading
import time
from enum import Enum
from logging import getLogger

from cooethercat.bus import EthercatBus
from cooethercat.helpers import *

# from cooethercat.epos4 import EPOS4Motor

class EPOS4Bus:

    @staticmethod
    def pdo_mode_only(func):
        def wrapper(self, *args, **kwargs):
            if not self.pdo_mode_is_active:
                raise RuntimeError("PDO mode must be enabled before calling this method.")
            return func(self, *args, **kwargs)
        return wrapper

    def __init__(self, interface: str) -> None:
        """
        Inputs:
            configFuncs: list of functions or dictionary of functions
                The functions will be run in the 'Pre-Operational' NMT state and the 'Switch on disabled'
                device state.
        """
        self._bus = EthercatBus(interface)
        self.slaves: list["EPOS4Motor"] = []
        self._PDO_CYCLE_TIME = 0.002  # 10ms, official sync manager 2 cycle time is 2ms but I've run into issues
        self._pdo_shutdown = threading.Event()
        self._pdo_thread = None
        # self._pdo_lock = threading.Lock()

    def open(self):
        self._bus.open()

    def close(self):
        self._bus.close()

    def initialize_slaves(self, id_type_map=dict[int, "EPOS4Motor"]):
        """Creates slave objects in the HAL and in this instance. Sends some basic information to the
        actual hardware to do this.

        pass a dictionary of bus id types if passed no default will be used.
        """
        self.disable_pdo()
        self._bus.initialize_slaves()
        self.slaves = []
        for i, instance in enumerate(self._bus.pysoem_master.slaves):
            #TODO if the type can wrap or extend the pysoem cdefslave then we might be able to
            # patch into the underlying library and forgo having two things that are so tightly coupled
            device = id_type_map[i](self, i, instance.name)
            self.slaves.append(device)
            try:
                instance.config_func = device.config_func  # config func takes a node number
                # instance.setup_func = device.setup_func  # just gets the device
            except AttributeError:
                getLogger(__name__).warning(f'Device type {type(device)} does not provide a setup/config function.')

    def configure_slaves(self):
        """Configure slaves with specific constants, mode and PDO mappings."""
        if not self.assert_network_state(NetworkManagementStates.PRE_OP):
            self.set_network_state(NetworkManagementStates.PRE_OP)

        if not self.assert_device_states(StatuswordStates.SWITCH_ON_DISABLED):
            self.set_device_states(StatuswordStates.SWITCH_ON_DISABLED)

        self._bus.configure_slaves()

        if not self.assert_network_state(NetworkManagementStates.SAFE_OP):
            raise RuntimeError("Failed to transition to Safe-OP state after configuring slaves.")

        for slave in self.slaves:
            slave._initialize_pdo_vars()
            slave._create_pdo_message([0] * len(slave.currentRxPDOMap))

    def get_slaves_info(self, as_string=False)-> str | list[dict]:
        """Return a list of dictionaries containing information about each slave."""
        slave_info = []

        for slave in self.slaves:
            slave_data = slave.debug_info_sdo
            slave_info.append(slave_data)

        record = ("  Node: {node}\n" +
                  "  NetState: {network_state}\n" +
                  "  DevState: {state}\n" +
                  "  Current Rx PDO Map: {current_rx_pdo_map}\n" +
                  "  Current Tx PDO Map: {current_txx_pdo_map}\n" +
                  ("-" * 40))

        if as_string:
            return "Slave Information:\n" + '\n'.join([record.format_map(slave_data) for slave_data in slave_info])
        else:
            return slave_info

    def slave_alarms(self):
        self._bus.pysoem_master.read_state()
        return [s.al_status for s in  self._bus.pysoem_master.slaves]

    ### State Methods  SDO ###
    def assert_network_state(self, state: Enum) -> bool:
        return self._bus.assertNetworkWideState(state.value)

    def get_network_state(self) -> int:
        return self._bus.getNetworkWideState()

    def set_network_state(self, state: Enum):
        self._bus.setNetworkWideState(state.value)

    def assert_device_states(self, state: StatuswordStates, id=None) -> bool:
        slaves = [self.slaves[id]] if id is not None else self.slaves
        for slave in slaves:
            if not slave.assert_device_state(state):
                return False
        return True

    def get_device_states(self):
        return [slave.get_device_state() for slave in self.slaves]

    def set_device_states(self, state: Enum, id=None):
        slaves = [self.slaves[id]] if id is not None else self.slaves
        for slave in slaves:
            slave.set_device_state(state)

    ### PDO Methods ###
    def enable_pdo(self, shutdown_first=True):
        if self._pdo_thread is not None:
            if self._pdo_thread.is_alive():
                return
            raise RuntimeError('PDO thread must be terminated and joined')

        getLogger(__name__).debug("Enabling PDO")

        def pdo_sender():
            self._pdo_shutdown.clear()
            while not self._pdo_shutdown.is_set():

                with self._bus.lock:
                    start = time.perf_counter_ns()
                    sent_counter = self._send_pdo(sleep=False)
                    working_counter = self._receive_pdo(sleep=False)
                    if working_counter not in (72, 0):
                        getLogger(__name__).warning(f"PDO send/receive cycle sent {sent_counter}, "
                                                    f"got working counter {working_counter} ({working_counter/3}*3)")
                time.sleep(max(self._PDO_CYCLE_TIME - (time.perf_counter_ns() - start) * 1e-9, 0))

        self._pdo_thread = threading.Thread(name='Ethercat PDO thread', target=pdo_sender, daemon=True)

        # Bringing the devices into operational state may immediately trigger motion depending on the target
        #  position.
        if shutdown_first:
            self.set_device_states(StatuswordStates.READY_TO_SWITCH_ON)
        self.set_network_state(NetworkManagementStates.OPERATIONAL)
        self._pdo_thread.start()
        time.sleep(self._PDO_CYCLE_TIME * 8)

        if self.assert_network_state(NetworkManagementStates.OPERATIONAL):
            getLogger(__name__).debug("PDO Enabled")
            return True
        else:
            getLogger(__name__).error("Failed to enable PDO")
            self._pdo_shutdown.set()
            getLogger(__name__).debug("Joining on PDO thread")
            self._pdo_thread.join()
            self._pdo_thread = None
            return False

    def disable_pdo(self):
        if self._pdo_thread and self._pdo_thread.is_alive():
            self._pdo_shutdown.set()
            self._pdo_thread.join()
            self._pdo_thread = None
        self.set_network_state(NetworkManagementStates.SAFE_OP)

    @property
    def pdo_mode_is_active(self):
        #TODO this is a check on the thread, not the actual mode robustness would be enhanced if we add code to ensure
        # the thread dies if we leave pdo mode
        return self._pdo_thread and self._pdo_thread.is_alive()

    def _send_pdo(self, sleep=True):
        sent = 0
        with self._bus.lock:
            self._bus.pysoem_master.send_processdata()
            for s in self.slaves:
                if s.pdo_message_pending.is_set():
                    getLogger(__name__).info(f'Sent pending PDO messaage for slave {s.node}')
                    sent +=1
                s.pdo_message_pending.clear()
        if sleep:
            time.sleep(self._PDO_CYCLE_TIME)
        return sent

    def _receive_pdo(self, sleep=True, timeout=2000):
        with self._bus.lock:
            working_counter = self._bus.pysoem_master.receive_processdata(timeout=timeout)
            start = time.perf_counter_ns()
            for slave in self.slaves:
                # Put the low level HAL slave byte buffer into slave.PDOInput
                x = self._bus.pysoem_master.slaves[slave.node].input
                slave.pdo_input = struct.unpack('<' + slave.tx_pdo_pack_format, x)
            finish = time.perf_counter_ns()

        # Enforce the minimum PDO cycle time after performing all the above operations
        if sleep:
            time.sleep(max(self._PDO_CYCLE_TIME - (finish - start) * 1e-9, 0))
        return working_counter

    def _send_receive_pdo(self, sleep=True, timeout=2000):
        with self._bus.lock:
            self._send_pdo(sleep=False)
            self._receive_pdo(sleep=sleep, timeout=timeout)

    @pdo_mode_only
    def wait_for_device_states_pdo(self, state: StatuswordStates, timeout:float=10):
        """Wait until all slaves are in the desired state. This function will block until the desired state is reached."""
        getLogger(__name__).debug(f'Waiting for {state} with timeout {timeout} in PDO mode.')
        start = time.time()
        while True:
            if self.assert_device_states_pdo(state):
                getLogger(__name__).debug(f'Attained {state} in PDO mode.')
                break

            if time.time() - start > timeout:
                raise TimeoutError(f"Timeout waiting for slaves to reach state {state}")
            time.sleep(0.1)

    @pdo_mode_only
    def assert_device_states_pdo(self, state: StatuswordStates):
        # Check the states of all slaves
        for slave in self.slaves:
            if not assertStatuswordState(slave._statusword, state):
                getLogger(__name__).debug(f"Slave {slave.node} not in state {state} "
                                          f"(in {slave._statusword & STATUSWORD_STATE_BITMASK})")
                return False
            return True

    @pdo_mode_only
    def set_device_states_pdo(self, state: StatuswordStates, timeout:float=10):
        """Change the device state of all slaves to the desired state. Will only work if all slaves are in the same start state."""
        #TODO check that this overhauled function actually works now
        if self.assert_device_states_pdo(state):
            return

        endState = getStatuswordState(state)

        seq = {}
        for s in self.slaves:
            # TODO get rid of this 4-line nonsense, should consolidate with proper state classes and object methods
            startingState = getStatuswordState(s._statusword)
            state_control_sequence = getStateTransitions(startingState, endState)
            getLogger(__name__).debug(f'State transition control word sequence for node {s.node}: {state_control_sequence}')
            seq[s.node] = state_control_sequence

        while seq:
            for s in self.slaves:
                try:
                    word = seq[s.node].pop(0)
                except IndexError:
                    del seq[s.node]  #done with device
                    continue
                except KeyError:
                    continue
                s.rx_data[s._controlwordPDOIndex] = word
                s._create_pdo_message(s.rx_data)
            self.wait_for_pdo_transmit()

        if timeout:
            self.wait_for_device_states_pdo(state, timeout=timeout)

    @pdo_mode_only
    def _block_until_target(self, verbose=False, timeout=10):
        """Block the master until all slaves have reached their target positions."""
        if verbose:
            getLogger(__name__).info("Actual positions:")
            string = '  |  '.join([f'Slave {slave.node}' for slave in self.slaves])
            getLogger(__name__).info(string)

        time.sleep(self._PDO_CYCLE_TIME * 6)

        start_time = time.time()

        while True:
            moving = False
            strings = []
            for slave in self.slaves:
                # Print the actual position if asked
                strings.append(f'{slave.position_pdo}')
                moving |= slave.moving_pdo

            if verbose:
                getLogger(__name__).info('  |  '.join(strings))

            # If no slave is moving anymore, we are done
            if not moving:
                break

            time.sleep(self._PDO_CYCLE_TIME * 6)
            # Exit loop if timeout is reached
            if time.time() - start_time > timeout:
                raise TimeoutError("Timeout while waiting for slaves to reach target positions.")

    @pdo_mode_only
    def wait_for_pdo_transmit(self, timeout=10):
        """Wait until all slaves don't have PDO message pending."""
        start = time.time()
        while any([s.pdo_message_pending.is_set() for s in self.slaves]):
            time.sleep(0.01)
            if not self.pdo_mode_is_active:
                raise RuntimeError('Left PDO mode while waiting for messages.')
            if time.time() - start > timeout:
                raise TimeoutError("Timeout while waiting for PDO messages to be received.")

    # TODO this code, ported from the old library was never fit for purpose.
    # @pdo_mode_only
    # def home_motors(self, verbose=True):
    #     """Start the homing process of all slaves."""
    #
    #     if not self.assert_device_states_pdo(StatuswordStates.OPERATION_ENABLED):
    #         self.set_device_states_pdo(StatuswordStates.OPERATION_ENABLED)
    #
    #     for slave in self.slaves:
    #         slave.change_operating_mode(OperatingModes.HOMING_MODE)
    #
    #     self.wait_for_pdo_transmit()
    #
    #     for slave in self.slaves:
    #         data = slave.rx_data
    #         data[slave._controlwordPDOIndex] = ControlWord.COMMAND_START_HOMING.value
    #         slave._create_pdo_message(data)
    #
    #     self.wait_for_pdo_transmit()
    #
    #     for slave in self.slaves:
    #         data = slave.rx_data
    #         data[slave._controlwordPDOIndex] = ControlwordStateCommands.SWITCH_ON_AND_ENABLE.value
    #         slave._create_pdo_message(data)
    #
    #     self.wait_for_pdo_transmit()
    #
    #     self._block_until_target(verbose=verbose)

    def home_using_current_position(self, position_source:Enum, position:int|dict[int,int]=None):
        if self.pdo_mode_is_active:
            raise RuntimeError('PDO mode must be disabled before homing with this function')

        if isinstance(position, int):
            position = {s.node: position for s in self.slaves}

        for dev in self.slaves:
            if dev.node not in position:
                getLogger(__name__).error(f'No position for {dev.node} in {position}, skipping.')
                continue
            try:
                dev.home_to_actual_position_sdo(position_source, position=position[dev.node])
            except TimeoutError as e:
                getLogger(__name__).error(f'Homing of {dev.node} failed with error {e}')
                continue
            except RuntimeError as e:
                getLogger(__name__).error(f'Homing of {dev.node} failed with error {e}')
                continue

    @pdo_mode_only
    def stop_motors(self):
        for slave in self.slaves:
            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlwordStateCommands.QUICK_STOP.value
            slave._create_pdo_message(data)


    def home(self, method: HomingMethods, timeout=40, position_source: PositionSource=None, position:int=None,
             current_threshold:int=300, setup_only=False):

        self.disable_pdo()
        for s in self.slaves:
            s.home_via_method(method, position_source=position_source, position=position,
                              current_threshold=current_threshold, setup_only=True)
        if not setup_only:
            self.enable_pdo()
            self.execute_homing(timeout=timeout)

    @pdo_mode_only
    def execute_homing(self, timeout=40):
        self.enable_pdo()
        self.set_device_states_pdo(StatuswordStates.OPERATION_ENABLED, timeout=.5)

        for slave in self.slaves:
            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlWord.COMMAND_START_HOMING.value
            slave._create_pdo_message(data)
            self.wait_for_pdo_transmit()

        start = time.time()
        while True:
            sw = [s.statusword_pdo for s in self.slaves]
            homed = [StatuswordBits.HOMING_ATTAINED in s for s in sw]
            errors = [(StatuswordBits.HOMING_ERROR in s or StatuswordBits.FAULT in s) for s in sw]
            done = [e or h for e,h in zip(errors, homed)]

            if all(done) or (time.time()-start) > timeout:
                break

        if all(homed):
            getLogger(__name__).info('All devices have attained home.')

        if any(errors):
            getLogger(__name__).error('Homed devices have faulted or homing errors.')

        self.set_device_states_pdo(StatuswordStates.READY_TO_SWITCH_ON)

    @pdo_mode_only
    def move_to(self, positions: int | dict[int], acceleration=10000, deceleration=10000, speed=7000, blocking=True,
                verbose=False):
        """Send slaves to the given positions based on their node IDs."""

        # If no slave_ids are provided, assume all slaves get the same position
        if isinstance(positions, int):
            positions = {i:positions for i in range(len(self.slaves))}

        # Ensure the network is in operational mode
        if not self.assert_device_states_pdo(StatuswordStates.OPERATION_ENABLED):
            getLogger(__name__).debug('Setting network state to OPERATION_ENABLED')
            self.set_device_states_pdo(StatuswordStates.OPERATION_ENABLED)

        # for slave in self.slaves:
        #     slave.change_operating_mode(OperatingModes.PROFILE_POSITION_MODE)

        for slave in self.slaves:
            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlwordStateCommands.SHUTDOWN.value
            slave._create_pdo_message(data)
            self.wait_for_pdo_transmit()
        time.sleep(0.1)

        for slave in self.slaves:
            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlwordStateCommands.SWITCH_ON_AND_ENABLE.value
            data[5] = OperatingModes.PROFILE_POSITION_MODE.value
            slave._create_pdo_message(data)
            self.wait_for_pdo_transmit()
        time.sleep(0.1)

        for id, position in positions.items():
            if not isinstance(id, int) and 0<id<len(self.slaves):
                getLogger(__name__).warning(f'Skipping a position key {id} that does not denote a slave.')
                continue

            slave = self.slaves[id]

            # Print information about the slave and its target position
            getLogger(__name__).info(f"Moving Slave {slave.node} to position {position}")

            # Prepare the data for the slave
            data = slave.rx_data
            data[slave._controlwordPDOIndex] = ControlWord.COMMAND_ABSOLUTE_START_IMMEDIATELY.value
            data[1] = int(position)  # Target position
            data[2] = int(acceleration)  # Profile acceleration
            data[3] = int(deceleration)  # Profile deceleration
            data[4] = int(speed)  # Profile velocity
            slave._create_pdo_message(data)  # Create PDO message for this slave

            self.wait_for_pdo_transmit()

        #TODO figure this out, the controlword used corresponds to START_HOMING
        # # Remove the start PPM motion control word from all slave buffers (necessary to avoid faults)
        # for id in positions.keys():
        #     slave = self.slaves[id]
        #
        #     # Print information about the slave and its target position
        #     getLogger(__name__).debug(f"Remove the start PPM motion control word from {slave.node} too avoid faults")
        #
        #     data = slave.rx_data
        #     data[slave._controlwordPDOIndex] = 0b11111
        #     slave._create_pdo_message(data)
        #
        # self.wait_for_pdo_transmit()

        # If blocking, wait until all slaves have reached their target positions
        if blocking:
            self._block_until_target(verbose=verbose)

    def check_errors(self):
        for slave in self.slaves:
            slave.check_errors()

    def __del__(self):
        self.disable_pdo()
