from dataclasses import dataclass

@dataclass
class EPOS4Error:
    code: int
    error_reg: int
    description: str
    reaction_code: str
    pos_clear: bool
    def __str__(self):
        return f"{self.description} ({self.code}) {self.reaction_code} {self.pos_clear}"


EPOS4_ERRORS = {0x0: EPOS4Error(0x0, 0x0, "No Error", '', False),
                0x1000: EPOS4Error(0x1000, 0b1, "Generic error", 'd', False),
                0x1090: EPOS4Error(0x1090, 0b1, "Firmware incompatibility error", 'd', False),
                0x2310: EPOS4Error(0x2310, 0b10, "Overcurrent error", 'd', False),
                0x2320: EPOS4Error(0x2320, 0b10, "Power stage protection error", 'd', False),
                0x3210: EPOS4Error(0x3210, 0b100, "Overvoltage error", 'd', False),
                0x3220: EPOS4Error(0x3220, 0b100, "Undervoltage error", 'd', False),
                0x4210: EPOS4Error(0x4210, 0b1000, "Thermal overload error", 'd', False),
                0x4380: EPOS4Error(0x4380, 0b1000, "Thermal motor overload error", 'd', False),
                0x5113: EPOS4Error(0x5113, 0b100, "Logic supply voltage too low error", 'd', False),
                0x5280: EPOS4Error(0x5280, 0b1, "Hardware defect error", 'd', False),
                0x5281: EPOS4Error(0x5281, 0b1, "Hardware incompatibility error", 'd', False),
                0x6080: EPOS4Error(0x6080, 0b1, "Sign of life error", 'd', False),
                0x6081: EPOS4Error(0x6081, 0b1, "Extension 1 watchdog error", 'a', False),
                0x6320: EPOS4Error(0x6320, 0b1, "Software parameter error", 'f', False),
                0x6380: EPOS4Error(0x6380, 0b1, "Persistent parameter corrupt error", 'd', False),
                0x7320: EPOS4Error(0x7320, 0b100000, "Position sensor error", 'd', False),
                0x7380: EPOS4Error(0x7380, 0b100000, "Position sensor breach error", 'd', True),
                0x7381: EPOS4Error(0x7381, 0b100000, "Position sensor resolution error", 'd', True),
                0x7382: EPOS4Error(0x7382, 0b100000, "Position sensor index error", 'd', True),
                0x7388: EPOS4Error(0x7388, 0b100000, "Hall sensor error", 'd', True),
                0x7389: EPOS4Error(0x7389, 0b100000, "Hall sensor not found error", 'd', True),
                0x738A: EPOS4Error(0x738A, 0b100000, "Hall angle detection error", 'd', True),
                0x738C: EPOS4Error(0x738C, 0b100000, "SSI sensor error", 'd', False),
                0x7390: EPOS4Error(0x7390, 0b100000, "Missing main sensor error", 'd', False),
                0x7391: EPOS4Error(0x7391, 0b100000, "Missing commutation sensor error", 'd', False),
                0x7392: EPOS4Error(0x7392, 0b100000, "Main sensor direction error", 'd', True),
                0x8110: EPOS4Error(0x8110, 0b10000, "CAN overrun error (object lost)", 'a', False),
                0x8111: EPOS4Error(0x8111, 0b10000, "CAN overrun error", 'a', False),
                0x8120: EPOS4Error(0x8120, 0b10000, "CAN passive mode error", 'a', False),
                0x8130: EPOS4Error(0x8130, 0b10000, "CAN heartbeat error", 'a', False),
                0x8150: EPOS4Error(0x8150, 0b10000, "CAN PDO COB-ID collision", 'a', False),
                0x8180: EPOS4Error(0x8180, 0b10000, "EtherCAT communication error", 'a', False),
                0x8181: EPOS4Error(0x8181, 0b10000, "EtherCAT initialization error", 'd', False),
                0x8182: EPOS4Error(0x8182, 0b10000, "EtherCAT Rx queue overflow", 'a', False),
                0x8183: EPOS4Error(0x8183, 0b10000, "EtherCAT communication error (internal)", 'a', False),
                0x81FD: EPOS4Error(0x81FD, 0b10000, "CAN bus turned off", 'a', False),
                0x81FE: EPOS4Error(0x81FE, 0b10000, "CAN Rx queue overflow", 'a', False),
                0x81FF: EPOS4Error(0x81FF, 0b10000, "CAN Tx queue overflow", 'a', False),
                0x8210: EPOS4Error(0x8210, 0b10000, "CAN PDO length error", 'a', False),
                0x8250: EPOS4Error(0x8250, 0b10000, "RPDO timeout", 'a', False),
                0x8280: EPOS4Error(0x8280, 0b10000, "EtherCAT PDO communication error", 'a', False),
                0x8281: EPOS4Error(0x8281, 0b10000, "EtherCAT SDO communication error", 'a', False),
                0x8611: EPOS4Error(0x8611, 0b10000000, "Following error", 'f', False),
                0x8A80: EPOS4Error(0x8A80, 0b10000000, "Negative limit switch error", 'f', False),
                0x8A81: EPOS4Error(0x8A81, 0b10000000, "Positive limit switch error", 'f', False),
                0x8A82: EPOS4Error(0x8A82, 0b10000000, "Software position limit error", 'f', False),
                0x8A88: EPOS4Error(0x8A88, 0b1, "STO error", 'd', False),
                0xFF01: EPOS4Error(0xFF01, 0b1, "System overloaded error", 'd', False),
                0xFF02: EPOS4Error(0xFF02, 0b1, "Watchdog error", 'd', True),
                0xFF0B: EPOS4Error(0xFF0B, 0b1, "System peak overloaded error", 'd', False),
                0xFF10: EPOS4Error(0xFF10, 0b100000, "Controller gain error", 'f', False),
                0xFF12: EPOS4Error(0xFF12, 0b100000, "Auto tuning current limit error", 'd', False),
                0xFF13: EPOS4Error(0xFF13, 0b100000, "Auto tuning identification current error", 'd', False),
                0xFF14: EPOS4Error(0xFF14, 0b100000, "Auto tuning data sampling error", 'd', False),
                0xFF15: EPOS4Error(0xFF15, 0b100000, "Auto tuning sample mismatch error", 'd', False),
                0xFF16: EPOS4Error(0xFF16, 0b100000, "Auto tuning parameter error", 'd', False),
                0xFF17: EPOS4Error(0xFF17, 0b100000, "Auto tuning amplitude mismatch error", 'd', False),
                0xFF18: EPOS4Error(0xFF18, 0b100000, "Auto tuning period length error", 'd', False),
                0xFF19: EPOS4Error(0xFF19, 0b100000, "Auto tuning timeout error", 'd', False),
                0xFF20: EPOS4Error(0xFF20, 0b100000, "Auto tuning standstill error", 'd', False),
                0xFF21: EPOS4Error(0xFF21, 0b100000, "Auto tuning torque invalid error", 'd', False),
                }

for i in range(0x1080,0x1083+1):
    EPOS4_ERRORS[i]=EPOS4Error(i, 0b1, 'Generic initialization error', 'd', False)
for i in range(0x6180, 0x6190 + 1):
    EPOS4_ERRORS[i] = EPOS4Error(i, 0b1, 'Internal software error', 'd', False)
for i in range(0x5480, 0x5483 + 1):
    EPOS4_ERRORS[i] = EPOS4Error(i, 0b1, 'Hardware error', 'd', False)
