from enum import Enum
from logging import getLogger
from dataclasses import dataclass, astuple


STATUSWORD_STATE_BITMASK = 0b1101111

class IncorrectState(Exception):
    pass


class EPOS4ErrorBits(Enum):
    MOTION = 7
    RESERVED = 6  # Reserved, always 0
    DEVICE_PROFILE_SPECIFIC = 5  # Device profile-specific
    COMMUNICATION_ERROR = 4
    TEMPERATURE_ERROR = 3
    VOLTAGE_ERROR = 2
    CURRENT_ERROR = 1
    GENERIC_ERROR = 0  # Generic error

class ControlWord(Enum):
    COMMAND_SHUTDOWN = 0x0006
    COMMAND_SWITCH_ON_AND_ENABLE = 0x000F
    COMMAND_START_HOMING = 0x001F
    COMMAND_QUICK_STOP = 0x000B
    COMMAND_HALT_HOMING = 0x011F
    COMMAND_ABSOLUTE_START_IMMEDIATELY = 0x003F

class ControlwordBits(Enum):
    HALT = 8
    START_HOMING = 4
    QUICK_STOP = 2
    ENABLE_VOLTAGE = 1
    ENABLE_OPERATION = 3
    NEW_SETPOINT = 4
    CHANGE_SET_IMMEDIATELY = 5
    ABSOLUTE_RELATIVE = 6
    FAULT_RESET = 7

class StatuswordStates(Enum):
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED = 0b1000000
    READY_TO_SWITCH_ON = 0b0100001
    SWITCHED_ON = 0b0100011
    OPERATION_ENABLED = 0b0100111
    QUICK_STOP_ACTIVE = 0b0000111
    FAULT_REACTION_ACTIVE = 0b0001111
    FAULT = 0b0001000

class StatuswordBits(Enum):
    READY_TO_SWITCH_ON = 0
    SWITCHED_ON = 1
    OPERATION_ENABLED = 2
    FAULT = 3
    VOLTAGE_ENABLED = 4
    QUICK_STOP = 5
    SWITCH_ON_DISABLED = 6
    WARNING = 7
    REMOTE = 9
    TARGET_REACHED = 10
    INTERNAL_LIMIT_ACTIVE = 11
    HOMING_ATTAINED = 12
    HOMING_ERROR = 13
    POSITION_REFERENCE_TO_HOME = 15

class NetworkManagementStates(Enum):
    NONE = 0x00
    INIT = 0x01
    PRE_OP = 0x02
    STATE_BOOT = 0x03
    SAFE_OP = 0x04
    OPERATIONAL = 0x08
    ERROR = 0x10
    ACK = 0x10

class StateCommands(Enum):
    SHUTDOWN = 0b0000000000000110
    SWITCH_ON = 0b0000000000000111
    SWITCH_ON_AND_ENABLE = 0b0000000000001111
    DISABLE_VOLTAGE = 0b0000000000000000
    QUICK_STOP = 0b0000000000000010
    DISABLE_OPERATION = 0b0000000000000111
    ENABLE_OPERATION = 0b0000000000001111
    FAULT_RESET = 0b0000000010000000

class OperatingModes(Enum):
    PROFILE_POSITION_MODE = 1
    PROFILE_VELOCITY_MODE = 3
    HOMING_MODE = 6
    CYCLIC_SYNCHRONOUS_POSITION_MODE = 8
    CYCLIC_SYNCHRONOUS_VELOCITY_MODE = 9
    CYCLIC_SYNCHRONOUS_TORQUE_MODE = 10

class HomingMethods(Enum):
    ACTUAL_POSITION = 37
    INDEX_POSITIVE_SPEED = 34
    INDEX_NEGATIVE_SPEED = 33
    HOME_SWITCH_POSITIVE_SPEED = 23
    HOME_SWITCH_NEGATIVE_SPEED = 27
    LIMIT_SWITCH_POSITIVE = 18
    LIMIT_SWITCH_NEGATIVE = 17


# Current state                   States we can go to
EPOS4_STATE_MACHINE = {
    'NOT_READY_TO_SWITCH_ON':     {},
    'SWITCH_ON_DISABLED':         {'READY_TO_SWITCH_ON': StateCommands.SHUTDOWN},
    'READY_TO_SWITCH_ON':         {'SWITCH_ON_DISABLED': StateCommands.DISABLE_VOLTAGE, 'OPERATION_ENABLED': StateCommands.SWITCH_ON_AND_ENABLE, 'SWITCHED_ON': StateCommands.SWITCH_ON}, #'OPERATION_ENABLED': stateCommands.SWITCH_ON_AND_ENABLE, <- should be in second index (yes, dicts become iterable and order specific with the items() function)
    'SWITCHED_ON':                {'READY_TO_SWITCH_ON': StateCommands.SHUTDOWN, 'OPERATION_ENABLED': StateCommands.ENABLE_OPERATION, 'SWITCH_ON_DISABLED': StateCommands.DISABLE_VOLTAGE},
    'OPERATION_ENABLED':          {'SWITCHED_ON': StateCommands.DISABLE_OPERATION, 'QUICK_STOP_ACTIVE': StateCommands.QUICK_STOP, 'READY_TO_SWITCH_ON': StateCommands.SHUTDOWN, 'SWITCH_ON_DISABLED': StateCommands.DISABLE_VOLTAGE},
    'QUICK_STOP_ACTIVE':          {'OPERATION_ENABLED': StateCommands.ENABLE_OPERATION, 'SWITCH_ON_DISABLED': StateCommands.DISABLE_VOLTAGE},
    'FAULT_REACTION_ACTIVE':      {'FAULT': StateCommands.FAULT_RESET},
    'FAULT':                      {'SWITCH_ON_DISABLED': StateCommands.FAULT_RESET}
    }

@dataclass(frozen=True)
class EPOS4Obj:
    """Class used for type checking and storing information from the firmware spec guide object dictionary."""
    index: int
    subindex: int
    packformat: str
    length: int

    def __iter__(self):
        return iter(astuple(self))


@dataclass(frozen=True)
class EPOS4Registers:
    """Object dictionary containing the same information as the Firmware Specification guide
    for the EPOS4 Micro. See section 6.2 for more details"""

    ERROR_REGISTER = EPOS4Obj(0x1001, 0x00, 'B', 8) # 6.2.2
    ERROR_CODE = EPOS4Obj(0x603F, 0x00, 'H', 16) # 6.2.2
    SERIAL_NUMBER = EPOS4Obj(0x1018, 0x04, 'I', 32) # 6.2.11.4
    DIAGNOSIS_HISTORY_NEWEST_MESSAGE = EPOS4Obj(0x10F3, 0x02, 'B', 8) # 6.2.13.2
    DIAGNOSIS_HISTORY_NEW_MESSAGES_AVAILABLE = EPOS4Obj(0x10F3, 0x04, 'c', 1) # 6.2.13.4
    DIAGNOSIS_HISTORY_FLAGS = EPOS4Obj(0x10F3, 0x05, 'H', 16) # 6.2.13.5
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_1 = EPOS4Obj(0x10F3, 0x06, 'IHHQ', 128) # 6.2.13.6
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_2 = EPOS4Obj(0x10F3, 0x07, 'IHHQ', 128) # 6.2.13.6
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_3 = EPOS4Obj(0x10F3, 0x08, 'IHHQ', 128) # 6.2.13.6
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_4 = EPOS4Obj(0x10F3, 0x09, 'IHHQ', 128) # 6.2.13.6
    DIAGNOSIS_HISTORY_DIAGNOSIS_MESSAGE_5 = EPOS4Obj(0x10F3, 0x0A, 'IHHQ', 128) # 6.2.13.6
    NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x0, 'B', 8) # 6.2.19.1 % SEARCH MARKER: Receive PDO mapping 1
    FIRST_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x1, 'I', 32) # 6.2.19.2 % SEARCH MARKER: Receive PDO 1 mapping
    SECOND_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x2, 'I', 32) # 6.2.19.2
    THIRD_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x3, 'I', 32) # 6.2.19.2
    FOURTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x4, 'I', 32) # 6.2.19.2
    FITH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x5, 'I', 32) # 6.2.19.2
    SIXTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x6, 'I', 32) # 6.2.19.2
    SEVENTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x7, 'I', 32) # 6.2.19.2
    EIGHTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x8, 'I', 32) # 6.2.19.2
    NINETH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0x9, 'I', 32) # 6.2.19.2
    TENTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0xa, 'I', 32) # 6.2.19.2
    ELEVENTH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0xb, 'I', 32) # 6.2.19.2
    TWELVETH_MAPPED_OBJECT_IN_RXPDO_1 = EPOS4Obj(0x1600, 0xc, 'I', 32) # 6.2.19.2
    NUMBER_OF_MAPPED_OBJECTS_IN_RXPDO_2 = EPOS4Obj(0x1601, 0x0, 'B', 8) # 6.2.20.1 % SEARCH MARKER: Receive PDO 2 mapping
    FIRST_MAPPED_OBJECT_IN_RXPDO_2 = EPOS4Obj(0x1601, 0x1, 'I', 32) # 6.2.20.2 % SEARCH MARKER: Receive PDO 2 mapping
    SECOND_MAPPED_OBJECT_IN_RXPDO_2 = EPOS4Obj(0x1601, 0x2, 'I', 32) # 6.2.20.2 % SEARCH MARKER: Receive PDO 2 mapping
    NUMBER_OF_MAPPED_OBJECTS_IN_TXPDO_1 = EPOS4Obj(0x1A00, 0x0, 'B', 8) # 6.2.27
    NUMBER_OF_USED_SYNC_MANAGER_CHANNELS = EPOS4Obj(0x1c00, 0x0, 'B', 8) # 6.2.31.1
    COMMUNICATION_TYPE_SYNC_CHANNEL_0 = EPOS4Obj(0x1c00, 0x1, 'B', 8) # 6.2.31.2
    COMMUNICATION_TYPE_SYNC_CHANNEL_1 = EPOS4Obj(0x1c00, 0x2, 'B', 8) # 6.2.31.3
    COMMUNICATION_TYPE_SYNC_CHANNEL_2 = EPOS4Obj(0x1c00, 0x3, 'B', 8) # 6.2.31.4
    COMMUNICATION_TYPE_SYNC_CHANNEL_3 = EPOS4Obj(0x1c00, 0x4, 'B', 8) # 6.2.31.5
    NUMBER_OF_ASSIGNED_RXPDOS = EPOS4Obj(0x1c12, 0x0, 'B', 8) # 6.2.32.1 % SEARCH MARKER: SYNC manager 2 PDO assignment
    FIRST_ASSIGNED_RXPDO = EPOS4Obj(0x1c12, 0x1, 'H', 16) # 6.2.32.2 % SEARCH MARKER: SYNC manager 2 PDO assignment
    FIRST_ASSIGNED_TXPDO = EPOS4Obj(0x1C13, 0x01, "H", 16) # 6.2.33.2
    SERIAL_NUMBER_COMPLETE = EPOS4Obj(0x2100, 0x01, 'Q', 64)
    CURRENT_CONTROLLER_P_GAIN = EPOS4Obj(0x30A0, 0x01, 'I', 32) # 6.2.61
    CURRENT_CONTROLLER_I_GAIN = EPOS4Obj(0x30A0, 0x02, 'I', 32) # 6.2.62
    HOME_OFFSET_MOVE_DISTANCE = EPOS4Obj(0x30B1, 0x00, 'i', 32) # 6.2.67
    DIGITAL_INPUT_CONFIGURATION_DGIN_1 = EPOS4Obj(0x3142, 0x01, 'B', 8) # 6.2.75.1
    DIGITAL_INPUT_CONFIGURATION_DGIN_2 = EPOS4Obj(0x3142, 0x02, 'B', 8) # 6.2.75.1
    DIGITAL_INPUT_CONFIGURATION_DGIN_3 = EPOS4Obj(0x3142, 0x03, 'B', 8) # 6.2.75.1
    DIGITAL_INPUT_CONFIGURATION_DGIN_4 = EPOS4Obj(0x3142, 0x04, 'B', 8) # 6.2.75.1
    CONTROLWORD = EPOS4Obj(0x6040, 0x00, 'H', 16) # 6.2.94
    STATUSWORD = EPOS4Obj(0x6041, 0x00, 'H', 16) # 6.2.95
    MODES_OF_OPERATION = EPOS4Obj(0x6060, 0x00, 'b', 8) # 6.2.100
    MODES_OF_OPERATION_DISPLAY = EPOS4Obj(0x6061, 0x00, 'b', 8) # 6.2.101
    POSITION_ACTUAL_VALUE = EPOS4Obj(0x6064, 0x00, 'i', 32) # 6.2.103
    VELOCITY_ACTUAL_VALUE = EPOS4Obj(0x606C, 0x00, 'i', 32) # 6.2.109
    VELOCITY_DEMAND_VALUE = EPOS4Obj(0x606B, 0x00, 'i', 32)  # 6.2.10
    TORQUE_ACTUAL_VALUE = EPOS4Obj(0x6077, 0x00, 'H', 16) # 6.2.109# 9
    TARGET_POSITION = EPOS4Obj(0x607A, 0x00, 'i', 32) # 6.2.113
    PROFILE_VELOCITY = EPOS4Obj(0x6081, 0x00, 'i', 32) # 6.2.118
    PROFILE_ACCELERATION = EPOS4Obj(0x6083, 0x00, 'i', 32) # 6.2.119
    PROFILE_DECELERATION = EPOS4Obj(0x6084, 0x00, 'i', 32) # 6.2.120
    FOLLOWING_ERROR_ACTUAL_VALUE = EPOS4Obj(0x60F4, 0x00, 'i', 32) # 6.2.144
    DIGITAL_INPUTS = EPOS4Obj(0x60FD, 0x00, 'I', 32) # 6.2.145
    PHYSICAL_OUTPUTS = EPOS4Obj(0x60FE, 0x01, 'i', 32) # 6.2.146.1   % SEARCH MARKER: Digital outputs
    NBR_OF_CONFIGURED_MODULES = EPOS4Obj(0xf030, 0x0, 'B', 8) # 6.2.151.1 % SEARCH MARKER: Modular device profile
    MODULE_1 = EPOS4Obj(0xf030, 0x1, 'I', 32) # 6.2.151.2 % SEARCH MARKER: Modular device profile
    TARGET_VELOCITY = EPOS4Obj(0x60FF, 0x00, 'I', 32) # 6.2.147


def getInfo(identifier: str | int, ObjDict) -> None:
    """Search for a command and print all information associated with a particular command name or index.
    
    identifier: 
        can either be a string representing the command name in the firmware spec guide 
        or an int representing the index of a command in the firmware spec guide.
    """
    matchMode = False # False: match by int, True: match by index

    if isinstance(identifier, str):
        matchMode = True

    foundCMD = False
    for command in ObjDict:

        if matchMode and command.name == identifier:
            print(f"{command.name} = {command.value}")
            foundCMD = True
        elif not matchMode and command.value[0] == identifier:
            print(f"{command.name} = {command.value}")
            foundCMD = True

    if not foundCMD:
        print("No command with that identifier was found in the object dictionary. Not all\n commands from the firmware spec guide have been entered. Use the EPOSObject for help adding a command.")

def getStatuswordState(statusword: int|Enum) -> str:
    """Returns the current state name of the statusword."""
    #TODO this whole statusword and enum stuff needs a rework, why are we ints in some places, strings in others and
    # the enum in still others. thats the whole point of a type to handle that for you, sigh.
    if isinstance(statusword, tuple):
        statusword = statusword[0]
    if isinstance(statusword, Enum):
        statusword = statusword.value

    for e in StatuswordStates: # for each statusword state
        if e.value == (statusword & STATUSWORD_STATE_BITMASK): # if the state matches the statusword
            return e.name # return the name of the state
    return "UNKNOWN STATE"

def assertStatuswordState(statusword: int, state: StatuswordStates|Enum) -> bool:
    """Asserts that the current state of the statusword matches the desired state."""
    maskedWord = (statusword & STATUSWORD_STATE_BITMASK)
    if state == StatuswordStates.NOT_READY_TO_SWITCH_ON: # Catch edge case where the state is not ready to switch on. (Value is 0, represents a lack of a bitmask and therefore this func always evaluates to true)
        return maskedWord == state.value
    maskedWord = maskedWord & state.value # if its all the same nothing in the masked work should change
    return maskedWord == state.value # compare the masked and anded word to the state value

def assertNetworkManagementState(state: int) -> str:
    """Asserts that the current state of the network management state matches the desired state."""
    for e in NetworkManagementStates:
        if e.value == state:
            return e.name
    return "UNKNOWN STATE"

def concatenateBits(nums: list[int], dataLengths: list[int] | int) -> int:
    """Combines multiple numbers together through bit concatenation into a single number with a total
    number of bits equal to the sum of the lengths of each data type rounded to the nearest 2^n."""

    if isinstance(dataLengths, int):
        dataLengths = [dataLengths]

    result = 0
    for i, num in enumerate(nums): # for each number
        result <<= dataLengths[i] # shift the result to the left by the length of the current number
        result |= num # bitwise OR the current number with the result
    return result

def make_pdo_mapping(objects: EPOS4Obj | list[EPOS4Obj], raiseErrorWhenOverLengthLimit = True) -> list[int]:
    """Create a 32 bit unsigned integer that can be used to map an object to a PDO. Integer
    is created by concatenating bits from the index, subindex, and length of the data type."""

    # If only a single command is entered, put it in a 1 element list
    if isinstance(objects, EPOS4Obj):
        objects = [objects]

    mappingIntegers = []
    totalLength = 0
    for objectToMap in objects:
        index, subIndex, _, length, *_ = objectToMap # The *_ is future proofing so that additions to type EPOS4Obj don't break the function

        mappingIntegers += [concatenateBits([index, subIndex, length], [16, 8, 8])]
        totalLength += length

    if totalLength > 8 * 40 and raiseErrorWhenOverLengthLimit: # Maximum PDO length for EtherCAT is 40 bytes
        raise ValueError(f"Length of datatypes associated with requested addresses total {totalLength} bits, exceeding the 40 byte maximum for EtherCAT by {totalLength - 40 * 8} bits.") 
    
    return mappingIntegers

def getStateTransitions(startState: str, endState: str) -> list[int]:
    """
    Get a list of state transition controlwords to get from the device start state to 
    the device end state.

    Inputs:
        startState: str
            The initial state of the system.
        endState: str
            The desired state of the system.
    Outputs:
        transitions: list[int]
            A list of controlwords that each correspond to a state transition
    """

    getLogger(__name__).debug(f'Getting transition from {startState} to {endState}.')

    ### Catch simple transition cases ###
    if startState == endState:
        return [0]

    if endState in EPOS4_STATE_MACHINE[startState]:
        transitions = [EPOS4_STATE_MACHINE[startState][endState].value]
        return transitions

    ### Perform actual search ###
    nodeCost = {
        'NOT_READY_TO_SWITCH_ON': None,  # Not ready to switch on
        'SWITCH_ON_DISABLED': None,  # Switch on disabled
        'READY_TO_SWITCH_ON': None,  # Ready to switch on
        'SWITCHED_ON': None,  # Switched on
        'OPERATION_ENABLED': None,  # Operation enabled
        'QUICK_STOP_ACTIVE': None,  # Quick stop active
        'FAULT_REACTION_ACTIVE': None,  # Fault reaction active
        'FAULT': None,  # Fault
        startState: 0}

    searchedNodes = []
    searching = True
    currentNode = startState
    while searching:
    
        if currentNode not in searchedNodes:
            for neighbor, transition in EPOS4_STATE_MACHINE[currentNode].items():
            
                cost = nodeCost[currentNode] + 1
                if nodeCost[neighbor] == None or cost < nodeCost[neighbor]:
                    nodeCost[neighbor] = cost

                if endState == neighbor:
                    searching = False
                    break

            searchedNodes += [currentNode]
        
        # TODO: Cleanup, this is really long for what it does
        _cost = nodeCost.copy()
        toPop = []
        for key, value in _cost.items():
            if value == None:
                toPop += [key]
            
            if key in searchedNodes:
                toPop += [key]

        for key in toPop:
            _cost.pop(key)

        currentNode = min(_cost, key=_cost.get)
    
    ### Reconstruct path ###
    _cost = nodeCost.copy()
    toPop = []
    for key, value in _cost.items():
        if value == None:
            toPop += [key]

        if value != None and key != endState and value == _cost[endState]:
            toPop += [key] 

    for key in toPop:
        _cost.pop(key)

    path = sorted(_cost.items(), key = lambda item: item[1])
    transitions = []
    for i, node in enumerate(path):

        if i == len(path) - 1: # Don't index the last node as a node that can be transitioned out of
            break

        nodeName, _ = node
        nextNodeName, _ = path[i+1]
        transitions += [EPOS4_STATE_MACHINE[nodeName][nextNodeName].value]
    
    return transitions

