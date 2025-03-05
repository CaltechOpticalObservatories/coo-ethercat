import time
import struct
from typing import Callable
import logging
import functools

from .helpers import *
from .bus import EthercatBus


class EPOS4Bus:

    def __init__(self, networkInterfaceName: str) -> None:
        """
        Inputs:
            configFuncs: list of functions or dictionary of functions
                The functions will be run in the 'Pre-Operational' NMT state and the 'Switch on disabled'
                device state.
        """
        self.HAL = EthercatBus(networkInterfaceName)
        self.numSlaves = 0
        self.slaves: list[EPOS4Motor] = None
        self.PDOCycleTime = 0.010 # 10ms, official sync manager 2 cycle time is 2ms but I've run into issues

    ### Network interface methods ###
    def openNetworkInterface(self):
        self.HAL.openNetworkInterface()    

    def closeNetworkInterface(self):
        self.HAL.closeNetworkInterface()

    ### Slave configuration methods ###
    def initialize_slaves(self, id_type_map=None):
        """Creates slave objects in the HAL and in this instance. Sends some basic information to the 
        actual hardware to do this.

        pass a dictionary of bus id types if passed no default will be used. if None EPOS4Motor will be used

        #TODO VITAL! verify that the index a feature of the device and not the point on the bus
        """
        self.numSlaves = self.HAL.initialize_slaves()
        
        self.slaves = []
        for i, slaveInst in enumerate(self.HAL.slaves):
            self.slaves += [id_type_map[i](self, i, slaveInst.name)]

    def configure_slaves(self):
        """Configure slaves with specific constants, mode and PDO mappings."""

        if not self.assertNetworkWideState(NetworkManagementStates.PRE_OP):
            self.setNetworkWideState(NetworkManagementStates.PRE_OP)

        if not self.assertCollectiveDeviceState(StatuswordStates.SWITCH_ON_DISABLED):
            self.setCollectiveDeviceState(StatuswordStates.SWITCH_ON_DISABLED)

        self.HAL.configureSlaves()

        if not self.assertNetworkWideState(NetworkManagementStates.SAFE_OP):
            raise RuntimeError("Failed to transition to Safe-OP state after configuring slaves.")
        
        for slave in self.slaves:
            slave.initializePDOVars()
            slave.createPDOMessage([0] * len(slave.currentRxPDOMap))
    
    def get_slaves_info(self, as_string=False)-> str | list[dict]:
        """Return a list of dictionaries containing information about each slave."""
        slave_info = []
        
        for slave in self.slaves:
            slave_data = {
                "node": slave.node,
                "state": slave.state,
                "objectDictionary": slave.objectDictionary,
                "currentRxPDOMap": slave.currentRxPDOMap,
                "currentTxPDOMap": slave.currentTxPDOMap,
            }
            slave_info.append(slave_data)

        record = ("Node: {node}\n" +
                  "  State: {state}\n" +
                  "  Object Dictionary: {objectDictionary}\n" +
                  "  Current Rx PDO Map: {currentRxPDOMap}\n" +
                  "  Current Tx PDO Map: {currentTxPDOMap}\n" +
                  ("-" * 40))

        if as_string:
            return "Slave Information:\n" + '\n'.join([record.format_map(slave_data) for slave_data in slave_info])
        else:
            return slave_info

    ### State Methods  SDO ###
    def assertNetworkWideState(self, state: Enum| int) -> bool:

        if isinstance(state, Enum):
            state = state.value
    
        return self.HAL.assertNetworkWideState(state)
    
    def getNetworkWideState(self):
        return self.HAL.getNetworkWideState()
    
    def setNetworkWideState(self, state: Enum| int):
        if isinstance(state, Enum):
            state = state.value
        self.HAL.setNetworkWideState(state)
    
    def assertCollectiveDeviceState(self, state: StatuswordStates | int) -> bool:
        
        if isinstance(state, Enum): # Handle the pythonic class and int type
           state = state.value
        
        for slave in self.slaves:
            if not slave.assertDeviceState(state):
                return False
        
        return True
    
    def getCollectiveDeviceState(self):
        states = []
        for slave in self.slaves:
            states += [slave.getDeviceState()]
        return states

    def setCollectiveDeviceState(self, state: Enum | int | str):
        for slave in self.slaves:
            slave.setDeviceState(state)
    
    def get_slave_by_node(self, node_id):
        """Return the slave object corresponding to the given node ID."""
        for slave in self.slaves:
            if slave.node == node_id:
                return slave
        return None  # Return None if no slave with the given node ID is found

    ### PDO Methods ###
    """The PDO methods assume that all slaves are in the same state and that the PDOS all have the same base configuration, or at least that differences
    are handled at other abstraction levels"""
    def enablePDO(self):
        print("Enabling PDO")
        self.HAL.setNetworkWideState(NetworkManagementStates.OPERATIONAL)

        # Send PDO data to maintain the operational state
        for i in range(4): # Iterate 4 times to clear the three buffer sync managers
            self.sendPDO()
            self.receivePDO()

        print("PDO Enabled")
        return self.assertNetworkWideState(NetworkManagementStates.OPERATIONAL)
    
    def disablePDO(self):
        self.setNetworkWideState(NetworkManagementStates.SAFE_OP)
    
    def sendPDO(self):
        self.HAL.sendProcessData()
        #TODO This explicit sleep makes this block the current thread, consider reworking
        time.sleep(self.PDOCycleTime)

    def receivePDO(self):
        self.HAL.receiveProcessData()
        start = time.perf_counter_ns()
        for slave in self.slaves:
            slave.fetchPDOData() # Put the low level HAL slave byte buffer into slave.PDOInput 

            slave.PDOInput = struct.unpack('<' + slave.currentTxPDOPackFormat, slave.PDOInput)
        
        # Enforce the minimum PDO cycle time after performing all the above operations
        if time.perf_counter_ns() - start > self.PDOCycleTime * 1e9:
            # TODO This explicit sleep makes this block the current thread, consider reworking
            time.sleep(self.PDOCycleTime - (time.perf_counter_ns() - start) * 1e-9)
 
    def waitForStatePDO(self, desiredState: StatuswordStates):
        """Wait until all slaves are in the desired state. This function will block until the desired state is reached."""

        print("Waiting for state")

        self.sendPDO()
        self.receivePDO()
        self.sendPDO()
        self.receivePDO()
        self.sendPDO()
        self.receivePDO()

        waiting = True
        while waiting:
            self.sendPDO()
            self.receivePDO()

            oneSlaveNotInState = False
            for slave in self.slaves:
                if not assertStatuswordState(slave.PDOInput[slave._statuswordPDOIndex], desiredState):
                    oneSlaveNotInState = True
                    print(slave.PDOInput[slave._statuswordPDOIndex])
                    print("One slave not in state")
                    print(slave._statuswordPDOIndex)
                    break
            
            if not oneSlaveNotInState:
                print("All slaves in state")
                waiting = False
                break

    def assertStatuswordStatePDO(self, state: StatuswordStates):

        # Clear buffer
        for i in range(4):
            self.sendPDO()
            self.receivePDO()

        # Check the states of all slaves
        for slave in self.slaves:
            statusword = slave.PDOInput[slave._statuswordPDOIndex]
            if not assertStatuswordState(statusword, state):
                return False
        return True

    def changeDeviceStatesPDO(self, desiredState: StatuswordStates):
        """Change the device state of all slaves to the desired state. Will only work if all slaves are in the same start state."""

        self.receivePDO()
        firstSlave = self.slaves[0]
        statusword = firstSlave.PDOInput[firstSlave._statuswordPDOIndex]

        if assertStatuswordState(statusword, StatuswordStates.NOT_READY_TO_SWITCH_ON):
            print("Slave needs time to auto switch states")
            while True:
                self.sendPDO()
                self.receivePDO()

                statusword = firstSlave.PDOInput[firstSlave._statuswordPDOIndex]
                print(getStatuswordState(statusword), assertStatuswordState(statusword, StatuswordStates.NOT_READY_TO_SWITCH_ON), bin(statusword))
                if not assertStatuswordState(statusword, StatuswordStates.NOT_READY_TO_SWITCH_ON):
                    print("Reached switch on disabled")
                    break
        
        startingState = getStatuswordState(statusword)
        endState = getStatuswordState(desiredState)

        stateTransitionControlwords = getStateTransitions(startingState, endState)
        print(stateTransitionControlwords)

        for controlword in stateTransitionControlwords:
            for slave in self.slaves:
                slave.RxData[slave._controlwordPDOIndex] = controlword
                print(slave.RxData)
                slave.createPDOMessage(slave.RxData)
                self.sendPDO()
                self.receivePDO()

        self.waitForStatePDO(desiredState)

    def changeOperatingMode(self, operatingMode: int | OperatingModes):
        """Change the RxPDO output to switch the operating mode of the slave on the next master.SendPDO() call."""

        for slave in self.slaves:
            slave.changeOperatingMode(operatingMode)

        self.sendPDO()
        self.receivePDO()

    def performHoming(self, printActualPositions=True):
        """Start the homing process of all slaves."""

        if not self.assertStatuswordStatePDO(StatuswordStates.OPERATION_ENABLED):
            self.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)

        self.changeOperatingMode(OperatingModes.HOMING_MODE)

        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b11111 # Control word to start homing #TODO: Consider implementing controlword class
            slave.createPDOMessage(data)
        
        self.sendPDO()
        self.receivePDO()

        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b01111 #TODO: Consider implementing controlword class
            slave.createPDOMessage(data)

        # Send and receive PDOs 3 times so that the three buffer system is cleared of previous values to avoid false 'Target reached' signals
        self.sendPDO()
        self.receivePDO()

        self.sendPDO()
        self.receivePDO()

        self.sendPDO()
        self.receivePDO()

        if printActualPositions:
            print("Slave actual positions:")
            for slave in self.slaves:
                print(f"Slave {slave.node}:", end='  |  ')

        homing = True
        while homing:
            self.sendPDO()
            self.receivePDO()

            oneSlaveStillHoming = False

            for slave in self.slaves:
                statusword = slave.PDOInput[slave._statuswordPDOIndex]
                if statusword & (1 << 10) != 1 << 10:
                    oneSlaveStillHoming = True

                if printActualPositions:
                    print(slave.PDOInput[1], end='  |  ')
            
            if printActualPositions:
                print('')
            
            if not oneSlaveStillHoming:
                homing = False
                break

    def goToPositions(self, positions: list[int], profileAcceleration=10000, profileDeceleration=10000, profileVelocity=1000, blocking=True, printActualPosition=False, slave_ids=None):
        """Send slaves to the given positions based on their node IDs."""
        
        # Ensure the network is in operational mode
        if not self.assertStatuswordStatePDO(StatuswordStates.OPERATION_ENABLED):
            self.changeDeviceStatesPDO(StatuswordStates.OPERATION_ENABLED)

        self.changeOperatingMode(OperatingModes.PROFILE_POSITION_MODE)

        # If no slave_ids are provided, assume all slaves get the same position
        if slave_ids is None:
            slave_ids = [slave.node for slave in self.slaves]  # Use slave node IDs

        # Ensure we have enough positions for the number of slaves (or vice versa)
        if len(positions) != len(slave_ids):
            raise ValueError("The number of positions must match the number of slave IDs provided.")

        # Map slave node ID to target position
        for slave_node, position in zip(slave_ids, positions):
            slave = self.get_slave_by_node(slave_node)  # Get the slave by its node ID
            if slave is None:
                raise ValueError(f"Slave with node {slave_node} not found.")
            # Print information about the slave and its target position
            print(f"Moving Slave {slave.node} to position {position}")

            # Prepare the data for the slave
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b01111  # Control word to start movement
            data[1] = position  # Target position
            data[2] = profileAcceleration  # Profile acceleration
            data[3] = profileDeceleration  # Profile deceleration
            data[4] = profileVelocity  # Profile velocity
            slave.createPDOMessage(data)  # Create PDO message for this slave

        self.sendPDO()  # Send PDOs to slaves
        self.receivePDO()

        # Remove the start PPM motion control word from all slave buffers (necessary to avoid faults)
        for slave in self.slaves:
            data = slave.RxData
            data[slave._controlwordPDOIndex] = 0b11111  # Stop motion command
            slave.createPDOMessage(data)

        self.sendPDO()
        self.receivePDO()

        # If blocking, wait until all slaves have reached their target positions
        if blocking:
            moving = True

            if printActualPosition:
                print("Actual positions:")
                for slave in self.slaves:
                    print(f"Slave {slave.node}", end='  |  ')

            # Clear old statuswords to avoid false 'Target reached' signals
            self.sendPDO()
            self.receivePDO()

            self.sendPDO()
            self.receivePDO()

            while moving:
                self.sendPDO()
                self.receivePDO()

                oneSlaveMoving = False
                for slave in self.slaves:
                    # Print the actual position if asked
                    if printActualPosition:
                        print(slave.PDOInput[1], end='  |  ')

                    statusword = slave.PDOInput[slave._statuswordPDOIndex]
                    if statusword & (1 << 10) != 1 << 10:  # Checking if the target position is reached
                        oneSlaveMoving = True

                if printActualPosition:
                    print('')

                # If no slave is moving anymore, we are done
                if not oneSlaveMoving:
                    moving = False
                    break


    def __del__(self, checkErrorRegisters=True):

        #TODO this is almost certainly inappropriate for production,
        # assumes bus is still active at instance destruction and has no exception handling.
        if checkErrorRegisters:
            for slave in self.slaves:
                print("Slave one diagnostics:")
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_1)
                print("Diagnosis message 1: ", resp)
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_2)
                print("Diagnosis message 2: ", resp)
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_3)
                print("Diagnosis message 3: ", resp)
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_4)
                print("Diagnosis message 4: ", resp)
                resp = slave.SDORead(slave.objectDictionary.DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_5)
                print("Diagnosis message 5: ", resp)

        self.closeNetworkInterface()


class EPOS4Motor:

    def __init__(self, master: EPOS4Bus, node: int, objectDictionary: str):
        """Initializes a slave object with the given Finite State Automation and object dictionary.
        
        Args: 
            node (int): The node number of the slave
            objectDictionary (str): The object dictionary that the slave uses
        
        Notes:
            - This object shouldn't be used directly, it should be created by the master class.
            - The object dictionary should be a string that corresponds to the object dictionary of the slave."""

        self.HAL = master.HAL
        self.node = node
        self.state = None
        self.currentRxPDOMap = None
        self.currentRxPDOPackFormat = None
        self.currentTxPDOMap = None
        self.currentTxPDOPackFormat = None
        self.RxData = None

        match objectDictionary:
            case "EPOS4":
                self.objectDictionary = EPOS4ObjDict
        
        ### Default Operation Mode PDO Maps ###
        self.PPMRx = [self.objectDictionary.CONTROLWORD, self.objectDictionary.TARGET_POSITION,
                    self.objectDictionary.PROFILE_ACCELERATION, self.objectDictionary.PROFILE_DECELERATION,
                    self.objectDictionary.PROFILE_VELOCITY, self.objectDictionary.MODES_OF_OPERATION, 
                    self.objectDictionary.PHYSICAL_OUTPUTS]
        self.PPMTx = [self.objectDictionary.STATUSWORD, self.objectDictionary.POSITION_ACTUAL_VALUE, self.objectDictionary.VELOCITY_ACTUAL_VALUE, 
                      self.objectDictionary.FOLLOWING_ERROR_ACTUAL_VALUE, self.objectDictionary.MODES_OF_OPERATION_DISPLAY, self.objectDictionary.DIGITAL_INPUTS]

    def __repr__(self):
        """String representation of the slave."""
        return f"Slave(node={self.node}, state={self.state}, objectDictionary={self.objectDictionary})"

    # Example method to get slave-specific info
    def getInfo(self):
        """Returns key information about this slave."""
        return {
            "node": self.node,
            "state": self.state,
            "objectDictionary": self.objectDictionary,
            "currentRxPDOMap": self.currentRxPDOMap,
            "currentTxPDOMap": self.currentTxPDOMap,
        }

    ### State methods ###
    def assertNetworkState(self, state: int) -> bool:
        return self.HAL.assertNetworkState(self, state)
    
    def getNetworkState(self):
        return self.HAL.getNetworkState(self)

    def setNetworkState(self, state: int):

        if isinstance(state, Enum) and not isinstance(state, StatuswordStates):
            state = state.value 

        return self.HAL.setNetworkState(self, state)

    # b. Applies to device states
    def assertDeviceState(self, state: int) -> bool:
        return self.HAL.assertDeviceState(self, state)

    def getDeviceState(self):
        return self.HAL.getDeviceState(self)

    def setDeviceState(self, state: int, mode = "automated"):
        """Set the device state of an individual slave. If the mode is default,
        try to set the state regardless of current state. If the mode is automated,
        automatically find the correct set of transtiions and set the state."""

        if mode.lower() == 'default': 
            self.HAL.setDeviceState(self, state)
            return None
        
        elif mode.lower() == 'automated':
            statusword = self.getDeviceState()
            deviceState = getStatuswordState(statusword)
            desiredState = state

            controlwords = getStateTransitions(deviceState, getStatuswordState(desiredState))
            for controlword in controlwords:
                self.HAL.setDeviceState(self, controlword)
            
            if self.assertDeviceState(desiredState):
                return None
    
    ### Communication methods ###
    ## SDO Methods ##
    def SDORead(self, address: tuple):

        if isinstance(address, Enum):
            address = address.value.value

        return self.HAL.SDORead(self, address)

    def SDOWrite(self, address: Enum| tuple, value: int, completeAccess=False):
        self.HAL.SDOWrite(self, address, value, completeAccess)

    ## PDO Methods ##
    def choosePDOMap(self, syncManager, PDOAddress):
        raise NotImplementedError

    def createPDOMessage(self, data: list[int]):
        """"Create a PDO rx (outgoing) message to this slave with the given data. This will overwrite all data currently
        in RxData"""
        packFormat = ''
        for address in self.currentRxPDOMap:
            packFormat += address[2]
        
        self.HAL.addPDOMessage(self, packFormat, data)
        self.RxData = data

    def fetchPDOData(self):
        self.HAL.updateSoftSlavePDOData(self)

    def initializePDOVars(self):
        """Finds the most important addresses of the Rx and Tx PDO and assigns them to the variables, as well as making the pack formats for the rx and tx pdos."""

        self._statuswordPDOIndex = None
        self._controlwordPDOIndex = None
        self._setOperationModePDOIndex = None
        self._modesOfOperationDisplayPDOIndex = None

        ### Find the most important addresses in the RxPDO ###
        for i, address in enumerate(self.currentRxPDOMap):
            
            match address:

                case self.objectDictionary.CONTROLWORD.value.value:
                    self._controlwordPDOIndex = i
                case self.objectDictionary.MODES_OF_OPERATION.value.value:
                    self._setOperationModePDOIndex = i

        ### Find the most important addresses in the TxPDO ###
        for i, address in enumerate(self.currentTxPDOMap):
                match address:
                    case self.objectDictionary.STATUSWORD.value.value:
                        self._statuswordPDOIndex = i
                    case self.objectDictionary.MODES_OF_OPERATION_DISPLAY.value.value:
                        self._modesOfOperationDisplayPDOIndex = i
        
        ### Create the pack formats for the Rx and Tx PDOs ###
        self.currentRxPDOPackFormat = ''
        self.currentTxPDOPackFormat = ''
        for rxAddress in self.currentRxPDOMap:
            self.currentRxPDOPackFormat += rxAddress[2]
        for txAddress in self.currentTxPDOMap:
            self.currentTxPDOPackFormat += txAddress[2]

    def changeOperatingMode(self, operatingMode: int | OperatingModes):
        """Change the RxPDO output to switch the operating mode of the slave on the next master.SendPDO() call."""

        if self.currentRxPDOMap == None:
            raise ValueError("No PDO map was assigned to the slave (or at least at a software level).")

        if isinstance(operatingMode, Enum):
            operatingMode = operatingMode.value

        setOperatingModePDOIndex = None
        operationModeIndex, operationModeSubIndex, *_ = self.objectDictionary.MODES_OF_OPERATION.value.value
        for i, address in enumerate(self.currentRxPDOMap):
            if address[0] == operationModeIndex and address[1] == operationModeSubIndex:
                setOperatingModePDOIndex = i
        
        if setOperatingModePDOIndex == None:
            raise RuntimeError("Can't change operating mode with PDO because the current RxPDO map doesn't contain the MODES_OF_OPERATION address.")
        
        if self.RxData == None: # Catch edge case that happens if user wants to change the operating mode first without creating a PDO message first
            self.RxData = [0] * len(self.currentRxPDOMap)   # This could be bad, I'm trusting that maxon has it setup such that PDOs with all zeros or the lack of data results in no changes on the slave
        
        self.RxData[setOperatingModePDOIndex] = operatingMode
        self.createPDOMessage(self.RxData)
        
    # Homing Mode (HMM) #
    def performHoming(self):
        """Requires PDO, operating mode homing, and device state operation enabled. Change the RxPDO output to tell the slave to begin the homing
        operation on the next master.SendPDO() call."""
        

    # Profile Position Mode (PPM) #


    
    ### Configuration methods ###
    def setWatchDog(self, timeout_ms: float):
        self.HAL.setWatchDog(self, timeout_ms)


def EPOS4MicroTRB_12CC_Config(slaveNum:int, slaves: list[EPOS4Motor]):
    """ Configures an EPOS4 Micro TRB 12CC device """
    #TODO see TODO note in EPOS4Bus.configureSlaves()
    dev = slaves[slaveNum]
    logging.debug(f"Configuring device {dev} (EPOS4 Micro 24/5)")


    # Define the Process Data Objects for PPM (Rx and Tx)
    PPMRx = [
        dev.objectDictionary.CONTROLWORD,
        dev.objectDictionary.TARGET_POSITION,
        dev.objectDictionary.PROFILE_ACCELERATION,
        dev.objectDictionary.PROFILE_DECELERATION,
        dev.objectDictionary.PROFILE_VELOCITY,
        dev.objectDictionary.MODES_OF_OPERATION,
        dev.objectDictionary.PHYSICAL_OUTPUTS
    ]
    PPMTx = [
        dev.objectDictionary.STATUSWORD,
        dev.objectDictionary.POSITION_ACTUAL_VALUE,
        dev.objectDictionary.VELOCITY_ACTUAL_VALUE,
        dev.objectDictionary.FOLLOWING_ERROR_ACTUAL_VALUE,
        dev.objectDictionary.MODES_OF_OPERATION_DISPLAY,
        dev.objectDictionary.DIGITAL_INPUTS
    ]

    # Create rx and tx map integers
    rxAddressInts = makePDOMapping(PPMRx)
    txAddressInts = makePDOMapping(PPMTx)

    # Assign rx map
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1, 0)
    for i, addressInt in enumerate(rxAddressInts):
        dev.SDOWrite((0x1600, i + 1, 'I'), addressInt)
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1, len(PPMRx))

    # Assign tx map
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1, 0)
    for i, addressInt in enumerate(txAddressInts):
        dev.SDOWrite((0x1A00, i + 1, 'I'), addressInt)
    dev.SDOWrite(dev.objectDictionary.NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1, len(PPMTx))

    dev.currentRxPDOMap = PPMRx
    dev.currentTxPDOMap = PPMTx

    # Configure Digital Inputs (example)
    #TODO these write fucntions seem to expect a tuple not an Enum
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_1, 255)
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_2, 1)
    dev.SDOWrite(dev.objectDictionary.DIGITAL_INPUT_CONFIGURATION_DGIN_1, 0)

    # Set the home offset move distance
    dev.SDOWrite(dev.objectDictionary.HOME_OFFSET_MOVE_DISTANCE, -622080)
    logging.debug("Slave configuration complete.")