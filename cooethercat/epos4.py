import time
import threading
import ctypes
from typing import Iterable

from .epos4bus import EPOS4Bus
from .helpers import *
from .bus import EthercatBus
from .epos4registers import EPOS4_ERRORS

#TODO Big picture: why is there both slave.set_device_state() and set_device_states_pdo (which uses PDO messages)


class EPOS4Motor:
    CONTROLWORD_DELAY_TIME = 0.01
    STATUSWORD_DELAY_TIME = 0.01
    ADDRESS = EPOS4Registers

    def __init__(self, master: EPOS4Bus, node: int):
        """Initializes a slave object with the given Finite State Automation and object dictionary.
        
        Args: 
            node (int): The node number of the slave
        
        Notes:
            - This object shouldn't be used directly, it should be created by the master class.
            - The object dictionary should be a string that corresponds to the object dictionary of the slave."""

        self.pdo_input = None
        self.HAL : EthercatBus = master._bus  #TODO make this simply master
        self.node = node
        self.currentRxPDOMap = None
        self.currentTxPDOMap = None
        self.rx_data = None
        self.pdo_message_pending = threading.Event()
        
        ### Default Operation Mode PDO Maps ###
        self.PPMRx = [self.ADDRESS.CONTROLWORD, self.ADDRESS.TARGET_POSITION,
                      self.ADDRESS.PROFILE_ACCELERATION, self.ADDRESS.PROFILE_DECELERATION,
                      self.ADDRESS.PROFILE_VELOCITY, self.ADDRESS.MODES_OF_OPERATION,
                      self.ADDRESS.PHYSICAL_OUTPUTS]
        self.PPMTx = [self.ADDRESS.STATUSWORD, self.ADDRESS.POSITION_ACTUAL_VALUE, self.ADDRESS.VELOCITY_ACTUAL_VALUE,
                      self.ADDRESS.FOLLOWING_ERROR_ACTUAL_VALUE, self.ADDRESS.MODES_OF_OPERATION_DISPLAY, self.ADDRESS.DIGITAL_INPUTS]

    def __repr__(self):
        """String representation of the slave."""
        return f"EPOS4Motor(masternode={self.node}, net_state={self._get_network_state()}, dev_state={self.get_device_state()})"

    @property
    def _statusword(self):
        return self.pdo_input[self._statuswordPDOIndex]

    def check_errors(self):
        print(f"Node {self.node} diagnostics:")
        resp = self._sdo_read(self.ADDRESS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_1)
        print(" Diagnosis message 1: ", resp)
        resp = self._sdo_read(self.ADDRESS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_2)
        print(" Diagnosis message 2: ", resp)
        resp = self._sdo_read(self.ADDRESS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_3)
        print(" Diagnosis message 3: ", resp)
        resp = self._sdo_read(self.ADDRESS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_4)
        print(" Diagnosis message 4: ", resp)
        resp = self._sdo_read(self.ADDRESS.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_5)
        print(" Diagnosis message 5: ", resp)

    @property
    def fault_state(self):
        return StatuswordBits.FAULT in StatuswordRegister(self._sdo_read(self.ADDRESS.STATUSWORD))

    def clear_faults(self):
        self._sdo_write(self.ADDRESS.CONTROLWORD, 1 << ControlwordBits.FAULT_RESET.value)

    def wait_for_statusword(self, state:StatuswordStates|Iterable[StatuswordBits]|StatuswordBits, any_bit:bool=True,
                            timeout:float=1, verbose:bool=False, monitor:EPOS4Obj|tuple[EPOS4Obj]=None):
        """Wait for a Statusword State or any or all of the specified bits to be set"""
        if isinstance(state, StatuswordBits):
            state = (state,)

        if isinstance(monitor, EPOS4Obj):
            monitor = (monitor,)
        elif monitor is None:
            monitor = tuple()

        collapse_func = any if any_bit else all

        start_time = time.time()
        while True:
            statusword = StatuswordRegister(self._sdo_read(self.ADDRESS.STATUSWORD))
            if isinstance(state, StatuswordStates):
                if statusword.state == state:
                    getLogger(__name__).info(f'Finished waiting for {statusword}')
                    return statusword
            elif collapse_func(bit in statusword for bit in state):
                getLogger(__name__).info(f'Finished waiting for {state}: {statusword}')
                return statusword
            for m in monitor:
                getLogger(__name__).debug(f'  {m}={self._sdo_read(m)}')
            if verbose:
                getLogger(__name__).info(f'{self.debug_info_sdo}')
            time.sleep(self.STATUSWORD_DELAY_TIME)
            if time.time() - start_time > timeout:
                raise TimeoutError(f'Timeout waiting for state {state}. Current statusword: {statusword}')

            time.sleep(self.STATUSWORD_DELAY_TIME)

    def stop(self):
        self.set_device_state(StatuswordStates.QUICK_STOP_ACTIVE)

    def halt(self):
        self.set_device_state(StatuswordStates.QUICK_STOP_ACTIVE)

    def abort(self):
        self.set_device_state(StatuswordStates.QUICK_STOP_ACTIVE)

    def reset(self):
        """Reset EPOS4"""
        # TODO if this is triggered the broader ecosystem of whats happening will be impacted. Figure out what needs to
        #  be handled and document it or add state management of class, bus, etc.
        getLogger(__name__).info(f"Resetting node {self.node}, reinitialization may be necessary")
        self._sdo_write(self.ADDRESS.PROGRAM_CONTROL, ProgramControlReg.INITIATE_DEVICE_RESET.value)
        return self._sdo_read(self.ADDRESS.PROGRAM_CONTROL)

    def fetch_config(self):
        """Fetch all parameters that are relevant to the baseline operationn of the EPOS, the goal of this is help
        facilitate programming without the EPOS Studio software."""
        register_list = ('AXIS_SENSORS_CONFIG',
                         'AXIS_CONTROL_STRUCTURE',
                         'AXIS_COMMUTATION_SENSORS',
                         'AXIS_CONFIG_MISC',
                         'POSITION_CONTROLLER_I_GAIN',
                         'POSITION_CONTROLLER_D_GAIN',
                         'POSITION_CONTROLLER_FF_VELOCITY_GAIN',
                         'POSITION_CONTROLLER_P_GAIN',
                         'POSITION_CONTROLLER_FF_ACCELERATION_GAIN',
                         'SSI_DATA_RATE_KBPS',
                         'SSI_NUMBER_OF_BITS',
                         'SSI_ENCODING_TYPE',
                         'SSI_TIMEOUT_TIME_US',
                         'SSI_SPECIAL_BITS_DATA',
                         'SSI_REFRESH_FREQ_HZ',
                         'SSI_POWER_UP_TIME_MS',
                         'SSI_POSITION_RAW_VALUE',
                         'SSI_COMMUTATION_OFFSET_VALUE',
                         'DIGITAL_INCREMENTAL_ENCODER_1',
                         'DIGITAL_INCREMENTAL_ENCODER_1_TYPE',
                         'GEAR_REDUCTION_NUMERATOR',
                         'GEAR_REDUCTION_DENOMINATOR',
                         'GEAR_MAX_INPUT_SPEED_RPM',
                         'GEAR_ORIENTATION',
                         'ANALOG_INCREMENTAL_ENCODER_TYPE',
                         'ANALOG_INCREMENTAL_ENCODER_RESOLUTION',
                         'ANALOG_INCREMENTAL_ENCODER_INDEX_POSITION',
                         'NOMINAL_CURRENT_MA',
                         'OUTPUT_CURRENT_LIMIT_MA',
                         'NUMBER_OF_POLE_PAIRS',
                         'THERMAL_TIME_CONSTANT_WINDING_DS',
                         'TORQUE_CONSTANT_UNM_A',
                         'CURRENT_CONTROLLER_P_GAIN',
                         'CURRENT_CONTROLLER_I_GAIN',
                         'VELOCITY_CONTROLLER_P_GAIN',
                         'VELOCITY_CONTROLLER_I_GAIN',
                         'VELOCITY_CONTROLLER_FF_VELOCITY_GAIN',
                         'VELOCITY_CONTROLLER_FF_ACCELERATION_GAIN',
                         'NODE_ID',
                         'SERIAL_NUMBER_COMPLETE',
                         )
        config = {}
        for r_name in register_list:
            config[r_name] = self._sdo_read(getattr(self.ADDRESS, r_name))
        return config

    def load_config(self, address_data:dict[str, int]):
        """ CAUTION this is very much without safety checks. Prototype to load in config instead of using EPOS Studio
        Use with fetch_config"""
        for k, v in address_data.items():
            self._sdo_write(getattr(self.ADDRESS, k), v)

    def temperature(self)->float:
        return self._sdo_read(self.ADDRESS.TEMPERATURE_DECICELSIUS)/10

    def home_via_method(self, method: HomingMethods, timeout=10, position_source: PositionSource=None, position:int=None,
                        current_threshold:int=400, monitor:EPOS4Obj=None, setup_only=False, wait_attained=False):

        if method==HomingMethods.ACTUAL_POSITION:
            if position_source is None:
                raise ValueError("Must provide position_source for ACTUAL_POSITION homing method.")

            if position_source==PositionSource.SSI:
                pos = self._sdo_read(self.ADDRESS.SSI_POSITION_RAW_VALUE)
                getLogger(__name__).info(f'Read an SSI position of {pos} for {self.node}, will home here.')
            elif position_source==PositionSource.USER:
                if position is None or abs(position) > 0x0FFFFFFF:
                    raise ValueError("Must provide position less than 2^31 (signed int32) to home "
                                     "using USER position source.")
                assert position is not None, "Must provide position to home using USER position source."
                assert abs(position) <= 0x0FFFFFFF, "Position must be less than 2^31 (signed int32)."
                pos = position
            elif position_source==PositionSource.CURRENT:
                pos = 0
            else:
                raise NotImplementedError
        else:
            pos = 0

        #make sure we are shutdown
        self._sdo_write(self.ADDRESS.CONTROLWORD, ControlwordStateCommands.SHUTDOWN)
        time.sleep(self.CONTROLWORD_DELAY_TIME)  # Needed before continuing or checking statusword
        self.wait_for_statusword(StatuswordStates.READY_TO_SWITCH_ON, timeout=.5)

        # Homing mode
        self._sdo_write(self.ADDRESS.MODES_OF_OPERATION, OperatingModes.HOMING_MODE)
        tic = time.time()
        while self._sdo_read(self.ADDRESS.MODES_OF_OPERATION_DISPLAY) != OperatingModes.HOMING_MODE.value:
            time.sleep(0.1)
            if time.time() - tic > .5:
                raise TimeoutError(f'Timeout waiting for homing mode on {self.node}.')

        home_pos = 0
        offset_distance = 100  #move away from limit for neg direction
        current_threshold = current_threshold  # threshold is in absolute value

        if method in (HomingMethods.CURRENT_THRESHOLD_POS_SPEED_AND_INDEX, HomingMethods.LIMIT_SWITCH_POSITIVE,
            HomingMethods.HOME_SWITCH_POSITIVE_SPEED, HomingMethods.INDEX_POSITIVE_SPEED):
            sign = -1
        else:
            sign = 1

        self._sdo_write(self.ADDRESS.HOME_POSITION, pos)
        # self._sdo_write(self.ADDRESS.FOLLOWING_ERROR_WINDOW, 1000)
        # self._sdo_write(self.ADDRESS.MAX_PROFILE_VELOCITY, 8000)
        # self._sdo_write(self.ADDRESS.QUICK_STOP_DECELERATION, 10000)
        self._sdo_write(self.ADDRESS.HOMING_ACCELERATION, 5000)
        self._sdo_write(self.ADDRESS.SPEED_FOR_SWITCH_SEARCH, 4000)
        self._sdo_write(self.ADDRESS.SPEED_FOR_ZERO_SEARCH, 2000)
        self._sdo_write(self.ADDRESS.HOMING_CURRENT_THRESHOLD, current_threshold)
        self._sdo_write(self.ADDRESS.HOME_OFFSET_MOVE_DISTANCE, offset_distance*sign)
        self._sdo_write(self.ADDRESS.HOME_POSITION, home_pos)
        self._sdo_write(self.ADDRESS.HOMING_METHOD, method)

        # self._sdo_write(self.ADDRESS.CONTROLWORD, ControlwordStateCommands.SHUTDOWN)
        # time.sleep(self.CONTROLWORD_DELAY_TIME)  # Needed before continuing or checking statusword
        # self.wait_for_statusword(StatuswordStates.READY_TO_SWITCH_ON, timeout=.5)

        if setup_only:
            return

        self._sdo_write(self.ADDRESS.CONTROLWORD, ControlwordStateCommands.SWITCH_ON_AND_ENABLE)
        time.sleep(self.CONTROLWORD_DELAY_TIME)  # Needed before continuing or checking statusword
        self.wait_for_statusword(StatuswordStates.OPERATION_ENABLED , timeout=.5)

        self._sdo_write(self.ADDRESS.CONTROLWORD, ControlWord.COMMAND_START_HOMING)
        time.sleep(self.CONTROLWORD_DELAY_TIME)  # Needed before continuing or checking statusword


        if wait_attained:
            statusword = self.wait_for_statusword((StatuswordBits.FAULT, StatuswordBits.FAULT.HOMING_ERROR,
                                                   StatuswordBits.FAULT.HOMING_ATTAINED), timeout=timeout,
                                                  monitor=monitor)

            if not StatuswordBits.HOMING_ATTAINED in statusword.bits_set:
                msg = f'Homing failed on {self.node} via {method}. Statusword: {statusword}, shutting down drive.'
                getLogger(__name__).error(msg)
                self._sdo_write(self.ADDRESS.CONTROLWORD, ControlwordStateCommands.SHUTDOWN)
                raise RuntimeError(msg)

            getLogger(__name__).info(f'Homed {self.node} via {method}.')

            self._sdo_write(self.ADDRESS.CONTROLWORD, ControlwordStateCommands.SHUTDOWN)
            time.sleep(self.CONTROLWORD_DELAY_TIME)  # Needed before continuing or checking statusword
            self.wait_for_statusword(StatuswordStates.READY_TO_SWITCH_ON, timeout=.5)

    def profile_position_move_sdo(self, position:int, speed:int, absolute:bool=True):

        self._sdo_write(self.ADDRESS.MODES_OF_OPERATION, OperatingModes.PROFILE_POSITION_MODE)
        tic = time.time()
        while self._sdo_read(self.ADDRESS.MODES_OF_OPERATION_DISPLAY) != OperatingModes.PROFILE_POSITION_MODE.value:
            time.sleep(0.1)
            if time.time() - tic > .5:
                raise TimeoutError(f'Timeout waiting for position mode on {self.node}.')

        # self._sdo_write(self.ADDRESS.FOLLOWING_ERROR_WINDOW, 1000)
        # self._sdo_write(self.ADDRESS.MAX_PROFILE_VELOCITY, 8000)
        self._sdo_write(self.ADDRESS.QUICK_STOP_DECELERATION, 10000)
        self._sdo_write(self.ADDRESS.PROFILE_ACCELERATION, 10000)
        self._sdo_write(self.ADDRESS.PROFILE_DECELERATION, 10000)
        self._sdo_write(self.ADDRESS.PROFILE_VELOCITY, int(abs(speed)))

        self._sdo_write(self.ADDRESS.CONTROLWORD, ControlwordStateCommands.SHUTDOWN)
        time.sleep(self.CONTROLWORD_DELAY_TIME)  # Needed before continuing or checking statusword
        self.wait_for_statusword(StatuswordStates.READY_TO_SWITCH_ON, timeout=.5)

        self._sdo_write(self.ADDRESS.CONTROLWORD, ControlwordStateCommands.SWITCH_ON_AND_ENABLE)
        time.sleep(self.CONTROLWORD_DELAY_TIME)  # Needed before continuing or checking statusword
        self.wait_for_statusword(StatuswordStates.OPERATION_ENABLED , timeout=.5)

        self._sdo_write(self.ADDRESS.TARGET_POSITION, int(position))

        if absolute:
            self._sdo_write(self.ADDRESS.CONTROLWORD, ControlWord.COMMAND_ABSOLUTE_START_IMMEDIATELY)
        else:
            self._sdo_write(self.ADDRESS.CONTROLWORD, ControlWord.COMMAND_RELATIVE_START_IMMEDIATELY)

        #TODO fault handling
        # if StatuswordBits.FAULT in statusword.bits_set:
        #     raise RuntimeError(f'Fault on {self.node} while moving to position {position}.')

        to_by = 'to' if absolute else 'by'
        getLogger(__name__).info(f'Moving {self.node} {to_by} {position}.')

    @property
    def position_pdo(self):
        return self.pdo_input[1]

    @property
    def velocity_pdo(self):
        return self.pdo_input[2]

    @property
    def moving_pdo(self):
        #TODO is merely be target position attained. it might be unattained but not moving for other reasons
        return not bool((self.pdo_input[self._statuswordPDOIndex] >> StatuswordBits.TARGET_REACHED.value) & 0b1)

    @property
    def following_error_pdo(self):
        return self.pdo_input[3]

    @property
    def statusword_pdo(self) -> StatuswordRegister:
        return StatuswordRegister(self.pdo_input[self._statuswordPDOIndex])

    @property
    def operation_mode_pdo(self):
        return OperatingModes(self.pdo_input[self._modesOfOperationDisplayPDOIndex])

    @property
    def controlword_sdo(self):
        return self._sdo_read(self.ADDRESS.CONTROLWORD)

    @property
    def debug_info_sdo(self):
        ec = self._sdo_read(self.ADDRESS.ERROR_CODE)
        return {'node': self.node,
                "network_state": self._get_network_state(),
                'position':self._sdo_read(self.ADDRESS.POSITION_ACTUAL_VALUE),
                'target_position': self._sdo_read(self.ADDRESS.TARGET_POSITION),
                'error_reg':self._sdo_read(self.ADDRESS.ERROR_REGISTER),
                'error_code': EPOS4_ERRORS.get(ec, f'Unknown error code ({ec})'),
                'mode_of_operation': self._sdo_read(self.ADDRESS.MODES_OF_OPERATION_DISPLAY),
                'velocity_demand' : self._sdo_read(self.ADDRESS.VELOCITY_DEMAND_VALUE),
                'velocity_actual': self._sdo_read(self.ADDRESS.VELOCITY_ACTUAL_VALUE),
                'velocity_profile': self._sdo_read(self.ADDRESS.PROFILE_VELOCITY),
                'velocity_target': self._sdo_read(self.ADDRESS.TARGET_VELOCITY),
                'torque_actual' : self._sdo_read(self.ADDRESS.TORQUE_ACTUAL_VALUE),
                'controlword': self._sdo_read(self.ADDRESS.CONTROLWORD),
                'statusword': StatuswordRegister(self._sdo_read(self.ADDRESS.STATUSWORD)),
                'temperatue': self.temperature(),
                # "current_rx_pdo_map": self.currentRxPDOMap,
                # "current_tx_pdo_map": self.currentTxPDOMap,
                }

    @property
    def info_sdo(self):
        ec = self._sdo_read(self.ADDRESS.ERROR_CODE)
        return {'node': self.node,
                'position':self._sdo_read(self.ADDRESS.POSITION_ACTUAL_VALUE),
                'target_position': self._sdo_read(self.ADDRESS.TARGET_POSITION),
                'error_reg':self._sdo_read(self.ADDRESS.ERROR_REGISTER),
                'error_code': EPOS4_ERRORS.get(ec, f'Unknown error code ({ec})'),
                }

    ### State methods ###
    def _assert_network_state(self, state: Enum) -> bool:
        return self.HAL.assertNetworkState(self, state)
    
    def _get_network_state(self):
        return self.HAL.getNetworkState(self)

    def _set_network_state(self, state: Enum):
        return self.HAL.setNetworkState(self, state)

    def assert_device_state(self, state: Enum) -> bool:
        state = state.value if isinstance(state, Enum) else state
        maskedWord = self.HAL.sdo_read(self, self.ADDRESS.STATUSWORD) & STATUSWORD_STATE_BITMASK
        return maskedWord == state

    def get_device_state(self):
        return self.HAL.sdo_read(self, self.ADDRESS.STATUSWORD)

    def set_device_state(self, state: Enum, mode ="automated"):
        """Set the device state of an individual slave. If the mode is default,
        try to set the state regardless of current state. If the mode is automated,
        automatically find the correct set of transitions and set the state."""

        if mode.lower() == 'default':
            self.HAL.sdo_write(self, self.ADDRESS.CONTROLWORD, state.value)

        elif mode.lower() == 'automated':
            statusword = self.get_device_state()
            device_state = getStatuswordState(statusword)
            desired_state = state

            controlwords = getStateTransitions(device_state, getStatuswordState(desired_state))
            getLogger(__name__).debug(f'State transition control word: {controlwords}')
            for controlword in controlwords:
                self.HAL.sdo_write(self, self.ADDRESS.CONTROLWORD, controlword)

        #TODO this is "Failing" as it is getting SWITCHED_ON, likely because the get state poll is too fast and it hasn't yet attained
        # OPERATION_ENABLED, it should optionally wait or at least not speciously warn
        time.sleep(0.1)
        statusword = self.get_device_state()
        if not assertStatuswordState(statusword, state):
            getLogger(__name__).warning(f"Failed to set device state, wanted {state}, got {statusword & STATUSWORD_STATE_BITMASK}")

    def _sdo_read(self, address: EPOS4Obj):
        return self.HAL.sdo_read(self, address)

    def _sdo_write(self, address: EPOS4Obj, value: int|Enum, completeAccess=False):
        value = value.value if isinstance(value, Enum) else value
        self.HAL.sdo_write(self, address, value, completeAccess)

    def _choose_pdo_map(self, syncManager, PDOAddress):
        raise NotImplementedError

    def _create_pdo_message(self, data: list[int]):
        """Create a PDO rx (outgoing) message to this slave with the given data. This will overwrite all data currently
        in RxData"""
        if self.pdo_message_pending.is_set():
            raise RuntimeError("PDO message is already pending.")
        packFormat = ''.join([address.packformat for address in self.currentRxPDOMap])
        getLogger(__name__).info(f'Queuing a PDO message for slave {self.node}')

        #TODO technically there is a race condition here add message and event set should be atomic
        self.HAL.addPDOMessage(self, packFormat, data)  #TODO make the HAL the EPOS4 BUS, this encapsulation is a mess
        self.pdo_message_pending.set()
        self.rx_data = data

    def _initialize_pdo_vars(self):
        """Finds the most important addresses of the Rx and Tx PDO and assigns them to the variables, as well as making the pack formats for the rx and tx pdos."""
        self._statuswordPDOIndex = None
        self._controlwordPDOIndex = None
        self._setOperationModePDOIndex = None
        self._modesOfOperationDisplayPDOIndex = None

        ### Find the most important addresses in the RxPDO ###
        for i, address in enumerate(self.currentRxPDOMap):
            
            match address:
                case self.ADDRESS.CONTROLWORD:
                    self._controlwordPDOIndex = i
                case self.ADDRESS.MODES_OF_OPERATION:
                    self._setOperationModePDOIndex = i

        ### Find the most important addresses in the TxPDO ###
        for i, address in enumerate(self.currentTxPDOMap):
                match address:
                    case self.ADDRESS.STATUSWORD:
                        self._statuswordPDOIndex = i
                    case self.ADDRESS.MODES_OF_OPERATION_DISPLAY:
                        self._modesOfOperationDisplayPDOIndex = i

    @property
    def tx_pdo_pack_format(self):
        return ''.join([x.packformat for x in self.currentTxPDOMap])

    @property
    def rx_pdo_pack_format(self):
        return ''.join([x.packformat for x in self.currentRxPDOMap])

    def change_operating_mode(self, mode: OperatingModes):
        """Change the RxPDO output to switch the operating mode of the slave on the next master.SendPDO() call."""

        if self.currentRxPDOMap is None:
            raise ValueError("No PDO map was assigned to the slave (at least at a software level).")

        # Catch edge case that happens if user changes operating mode without creating a PDO message first
        if self.rx_data is None:
            raise RuntimeError("Create a PDO message before changing operating mode.")
            # self.RxData = [0] * len(self.currentRxPDOMap)
            # This could be bad, I'm trusting that maxon has it setup such that PDOs with
            # all zeros or the lack of data results in no changes on the slave

        rx_ndx = None
        index, subindex, *_ = self.ADDRESS.MODES_OF_OPERATION
        for i, address in enumerate(self.currentRxPDOMap):
            if address.index == index and address.subindex == subindex:
                rx_ndx = i
        
        if rx_ndx is None:
            raise RuntimeError("Can't change operating mode with PDO because the current "
                               "RxPDO map doesn't contain the MODES_OF_OPERATION address.")

        #TODO why are we passing around an attribute? This is gross
        self.rx_data[rx_ndx] = mode.value
        self._create_pdo_message(self.rx_data)

    def profile_position_move(self, position:int, speed:int):
        #TODO update this with modern statusword/state functions
        if not self.assert_device_state(StatuswordStates.OPERATION_ENABLED):
            raise RuntimeError("Device must be in OPERATION_ENABLED state to use profile position move.")

        startingState = getStatuswordState(self._statusword)
        endState = getStatuswordState(StatuswordStates.OPERATION_ENABLED)
        stateTransitionControlwords = getStateTransitions(startingState, endState)
        getLogger(__name__).debug(f'State transition control word: {stateTransitionControlwords}')

        for controlword in stateTransitionControlwords:
            self.rx_data[self._controlwordPDOIndex] = controlword
            self._create_pdo_message(self.rx_data)
            while self.pdo_message_pending.is_set():
                time.sleep(.1)
            time.sleep(1)  # wait for transition # TODO make this work nicely

        self.change_operating_mode(OperatingModes.PROFILE_POSITION_MODE)
        time.sleep(1)

        data = self.rx_data
        data[self._controlwordPDOIndex] = ControlwordStateCommands.SWITCH_ON_AND_ENABLE.value
        data[1] = position  # Target position
        data[2] = 10000  # Profile acceleration
        data[3] = 10000  # Profile deceleration
        data[4] = speed  # Profile velocity
        # data[5] = OperatingModes.PROFILE_POSITION_MODE.value
        self._create_pdo_message(data)  # Create PDO message for this slave

        while self.pdo_message_pending.is_set():
            time.sleep(.1)

        #Change to QUICK_STOP_ACTIVE state
        startingState = getStatuswordState(self._statusword)
        endState = getStatuswordState(StatuswordStates.QUICK_STOP_ACTIVE)
        stateTransitionControlwords = getStateTransitions(startingState, endState)
        getLogger(__name__).debug(f'State transition control word: {stateTransitionControlwords}')

        for controlword in stateTransitionControlwords:
            self.rx_data[self._controlwordPDOIndex] = controlword
            self._create_pdo_message(self.rx_data)
            while self.pdo_message_pending.is_set():
                time.sleep(.1)
            time.sleep(1)  # wait for transition # TODO make this work nicely

    def watchdog(self, timeout_ms: float|None):
        """Set to None to disable the watchdog."""
        if timeout_ms is None:
            #TODO Disable the watchdog, is this even possible?
            raise NotImplementedError('Disabling watchdog not yet implemented')
        else:
            self.HAL.set_watchdog(self, timeout_ms)

    def config_func(self, node_id: int):
        raise RuntimeError('Must be implemented by subclass')