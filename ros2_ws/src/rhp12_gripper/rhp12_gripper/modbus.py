"""Modbus-RTU helpers for the ROBOTIS RH-P12-RN(A) gripper.

!!! UNVERIFIED ON HARDWARE !!!
Register numbers come from the ROBOTIS e-Manual (Modbus column of the control table):
    Torque Enable  : 40257 (Lo byte)  -> 0-based register 256
    Goal Position  : 40283 (4 bytes)  -> 0-based registers 282..283
    Present Position: 40291           -> 0-based register 290
The word order of 32-bit values and the effect of writing a whole register that
also holds a neighbouring 1-byte item are NOT confirmed. Test with the gripper
alone on the bench (not mounted on the arm) before using it on the robot.
"""

from typing import List

SLAVE_ID_DEFAULT = 1
REG_TORQUE_ENABLE = 256     # 40257, Lo byte
REG_GOAL_POSITION = 282     # 40283, 32-bit -> 2 registers
REG_PRESENT_POSITION = 290  # 40291, 32-bit -> 2 registers

POS_MIN = 0
POS_MAX = 1150              # pulses (e-Manual: Max Position Limit default)


def crc16(data: bytes) -> int:
    """Modbus CRC-16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def _with_crc(frame: bytes) -> bytes:
    crc = crc16(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # CRC low byte first


def write_single_register(slave: int, reg: int, value: int) -> bytes:
    """Function 0x06."""
    frame = bytes([slave, 0x06, (reg >> 8) & 0xFF, reg & 0xFF,
                   (value >> 8) & 0xFF, value & 0xFF])
    return _with_crc(frame)


def write_multiple_registers(slave: int, reg: int, values: List[int]) -> bytes:
    """Function 0x10."""
    n = len(values)
    frame = bytearray([slave, 0x10, (reg >> 8) & 0xFF, reg & 0xFF,
                       (n >> 8) & 0xFF, n & 0xFF, n * 2])
    for v in values:
        frame += bytes([(v >> 8) & 0xFF, v & 0xFF])
    return _with_crc(bytes(frame))


def torque_enable_frame(slave: int, on: bool) -> bytes:
    return write_single_register(slave, REG_TORQUE_ENABLE, 1 if on else 0)


def goal_position_frame(slave: int, pulse: int, low_word_first: bool = True) -> bytes:
    pulse = max(POS_MIN, min(POS_MAX, int(pulse)))
    lo, hi = pulse & 0xFFFF, (pulse >> 16) & 0xFFFF
    values = [lo, hi] if low_word_first else [hi, lo]
    return write_multiple_registers(slave, REG_GOAL_POSITION, values)


def ratio_to_pulse(ratio: float, pulse_open: int, pulse_closed: int) -> int:
    """ratio 0.0 = open ... 1.0 = closed (mapped onto the pulse range)."""
    ratio = max(0.0, min(1.0, ratio))
    return int(round(pulse_open + ratio * (pulse_closed - pulse_open)))


# DRL program sent through /dsr01/drl/drl_start (dsr_msgs2/srv/DrlStart).
# !!! UNVERIFIED: the flange_serial_* function names/arguments and the controller
# firmware support (2.10) must be checked against the Doosan DRL manual. !!!
DRL_TEMPLATE = """\
flange_serial_open(baudrate={baudrate}, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
{writes}
flange_serial_close()
"""
DRL_WRITE_LINE = "flange_serial_write(bytes([{byte_list}]))\nwait({wait})"


def build_drl(frames: List[bytes], baudrate: int = 57600, wait: float = 0.05) -> str:
    writes = "\n".join(
        DRL_WRITE_LINE.format(byte_list=", ".join(str(b) for b in f), wait=wait)
        for f in frames)
    return DRL_TEMPLATE.format(baudrate=baudrate, writes=writes)
