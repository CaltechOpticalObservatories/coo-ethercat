import ctypes
import pysoem
import logging


class InputPDO_PPM(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('statusword', ctypes.c_uint16),
        ('position_actual_value', ctypes.c_int32),
        ('velocity_actual_value', ctypes.c_int32),
        ('following_error_actual_value', ctypes.c_int32),
        ('modes_of_operation_display', ctypes.c_uint8),
        ('digital_inputs', ctypes.c_uint8),
    ]


class OutputPDO_PPM(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('controlword', ctypes.c_uint16),
        ('target_position', ctypes.c_int32),
        ('profile_acceleration', ctypes.c_uint32),
        ('profile_deceleration', ctypes.c_uint32),
        ('profile_velocity', ctypes.c_uint32),
        ('modes_of_operation', ctypes.c_uint8),
        ('digital_outputs', ctypes.c_uint8),
    ]


def configure_master_for_pdo_operation(master: pysoem.Master):
    """! Configure the master for PDO operation
    @param master: the master to configure for PDO operation
    """

    master.read_state()
    io_map_size = master.config_map()
    logging.info('configure_master_for_pdo_operation: io_map_size: {}'.format(io_map_size))

    if master.state_check(pysoem.SAFEOP_STATE, 500000) != pysoem.SAFEOP_STATE:
        master.read_state()
        fail=False
        for slave in master.slaves:
            if not slave.state == pysoem.SAFEOP_STATE:
                fail=True
                logging.warning(f'{slave.name} did not reach SAFEOP state\n'
                                f'al status code {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})')
        if fail:
            raise RuntimeError('not all slaves reached SAFEOP state')

        master.state = pysoem.OP_STATE
        master.write_state()

        master.state_check(pysoem.OP_STATE, 500000)
        if master.state != pysoem.OP_STATE:
            master.read_state()
            for slave in master.slaves:
                if not slave.state == pysoem.OP_STATE:
                    logging.warning(f'{slave.name} did not reach OP state\n'
                                    f'al status code {hex(slave.al_status)} ({pysoem.al_status_code_to_string(slave.al_status)})')
            raise RuntimeError('not all slaves reached OP state')


def configure_position_using_pdo(master, slaves, positions, velocities, accelerations):
    output_data = OutputPDO_PPM()
    for i in range(len(slaves)):
        output_data.controlword = ControlWord.COMMAND_SHUTDOWN.value
        output_data.target_position = positions[i]
        output_data.profile_velocity = velocities[i]
        output_data.profile_acceleration = accelerations[i]
        output_data.profile_deceleration = accelerations[i]
        output_data.modes_of_operation = OperatingModes.PROFILE_POSITION_MODE.value
        output_data.digital_outputs = 0
        slaves[i].output = output_data.pack()
    master.send_processdata()
    master.receive_processdata(1000)

    for i in range(len(slaves)):
        output_data.controlword = ControlWord.COMMAND_SWITCH_ON_AND_ENABLE.value
        output_data.target_position = positions[i]
        output_data.profile_velocity = velocities[i]
        output_data.profile_acceleration = accelerations[i]
        output_data.profile_deceleration = accelerations[i]
        output_data.modes_of_operation = OperatingModes.PROFILE_POSITION_MODE.value
        output_data.digital_outputs = 0
        slaves[i].output = output_data.pack()

    master.send_processdata()
    master.receive_processdata(1000)

    for i in range(len(slaves)):
        output_data.controlword = ControlWord.COMMAND_ABSOLUTE_START_IMMEDIATELY.value
        output_data.target_position = positions[i]
        output_data.profile_velocity = velocities[i]
        output_data.profile_acceleration = accelerations[i]
        output_data.profile_deceleration = accelerations[i]
        output_data.modes_of_operation = OperatingModes.PROFILE_POSITION_MODE.value
        output_data.digital_outputs = 0
        slaves[i].output = output_data.pack()

    master.send_processdata()
    master.receive_processdata(1000)


