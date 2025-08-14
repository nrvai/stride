import time
import can
import math
import struct

from enum import IntEnum, auto
from typing import Iterable
from dataclasses import dataclass
from collections.abc import Buffer


CAN_PORT = "can0"
ZERO_DATA = [0, 0, 0, 0, 0, 0, 0, 0]


class CommunicationType(IntEnum):
    DeviceID = 0
    ControlMode = 1
    Feedback = 2
    Enable = 3
    Disable = 4
    MechanicalZero = 6
    ReadParameter = 17
    WriteParameter = 18
    FaultFeedback = 21


class Parameter(IntEnum):
    RunMode = 0x7005
    IqRef = 0x7006
    SpdRef = 0x700A
    LimitTorque = 0x700B
    CurKp = 0x7010
    CurKi = 0x7011
    CurFiltGain = 0x7014
    LocRef = 0x7016
    LimitSpd = 0x7017
    LimitCur = 0x7018
    MechPos = 0x7019
    IqFilt = 0x701A
    MechVel = 0x701B
    VBus = 0x701C
    LocKp = 0x701E
    SpdKp = 0x701F
    SpdKi = 0x7020
    SpdFiltGain = 0x7021
    AccRad = 0x7022
    VelMax = 0x7024
    AccSet = 0x7025
    EPScanTime = 0x7026
    CanTimeout = 0x7028
    ZeroSta = 0x7029


class MotorMode(IntEnum):
    Reset = 0
    Calibration = 1
    Run = 2
    Uknown = 3


@dataclass
class MotorStatus:
    mode: MotorMode
    has_error: bool
    unknown_mode: bool
    uncalibrated: bool
    gridlock_overload: bool
    magnetic_coding_fault: bool
    overtemperature: bool
    overcurrent: bool
    undervoltage: bool


@dataclass
class Feedback():
    motor_id: int
    status: MotorStatus
    angle: float
    velocity: float
    torque: float
    temp: float


class RobstrideBus:
    def __init__(self):
        self.can_bus = can.interface.Bus(interface='socketcan', channel=CAN_PORT)

    def recv(self):
        res = self.can_bus.recv()
        if not res or res.is_error_frame:
            Exception("response error")
        return res

    def send(self, comm_type: CommunicationType, id_field: int, data: bytes | bytearray | int | Iterable[int] | None = ZERO_DATA):
        arb_id = id_field + (comm_type << 24)
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
        self.can_bus.send(msg)
        print([msg])


class Actuator:
    def __init__(self, motor_id: int, host_id: int = 0xAA):
        self.bus = RobstrideBus()
        self.motor_id = motor_id
        self.host_id = host_id
        self.id_field = self.motor_id + (self.host_id << 8)

    def get_feedback(self):
        def _unpack(x: Buffer) -> int:
            return struct.unpack('>H', x)[0]

        def _unpack_range(x: Buffer, r: int) -> float:
            return _unpack(x) / 65535 * r - r / 2

        def _get_error_bit(error_bits: int, i: int) -> bool:
            return bool(error_bits & (1 << i))

        angle_range = 8 * math.pi
        vel_range = 88
        torque_range = 34

        res = self.bus.recv()

        angle = _unpack_range(res.data[0:2], angle_range)
        velocity = _unpack_range(res.data[2:4], vel_range)
        torque = _unpack_range(res.data[4:6], torque_range)

        temp = _unpack(res.data[6:8]) / 10

        arb_id = res.arbitration_id
        mode = MotorMode((arb_id >> 22) & 0x03)
        unknown_mode = mode == MotorMode.Uknown

        error_bits = (arb_id >> 16) & 0x3F
        status = MotorStatus(
            mode=mode,
            has_error=bool(error_bits) or unknown_mode,
            unknown_mode=unknown_mode,
            uncalibrated=_get_error_bit(error_bits, 5),
            gridlock_overload=_get_error_bit(error_bits, 4),
            magnetic_coding_fault=_get_error_bit(error_bits, 3),
            overtemperature=_get_error_bit(error_bits, 2),
            overcurrent=_get_error_bit(error_bits, 1),
            undervoltage=_get_error_bit(error_bits, 0)
        )

        feedback = Feedback(1, status, angle, velocity, torque, temp)
        print(feedback)

        return feedback

    def enable(self):
        self.bus.send(CommunicationType.Enable, self.id_field)
        return self.get_feedback()

    def disable(self):
        self.bus.send(CommunicationType.Disable, self.id_field)
        return self.get_feedback()

    def write_param(self, pid: Parameter, value: int | float):
        """
        data:
        - 0~1: index
        - 2~3: 00
        - 4~7: data (little-endian)
        """
        pid_bytes = struct.pack('<I', pid)[:2]

        if pid in [0x7005, 0x7026, 0x7028, 0x7029]:
            value_bytes = struct.pack('<I', value)
        else:
            value_bytes = struct.pack('<f', value)

        data = bytes([*pid_bytes, 0, 0, *value_bytes])
        self.bus.send(CommunicationType.WriteParameter, self.id_field, data)

        return self.get_feedback()

    def read_param(self, pid: Parameter) -> int:
        pid_bytes = struct.pack('<I', pid)[:2]
        data = bytes([*pid_bytes, 0, 0, 0, 0, 0, 0])

        self.bus.send(CommunicationType.ReadParameter, self.id_field, data)

        res = self.bus.recv()
        res_value = struct.unpack('<f', res.data[4:])[0]

        return res_value, self.get_feedback()

    def get_device_info(self):
        self.bus.send(CommunicationType.DeviceID, self.id_field, ZERO_DATA)
        res = self.bus.recv()
        return struct.unpack('<f', res.data)[0]

    def set_mechanical_zero(self):
        self.bus.send(CommunicationType.MechanicalZero, self.id_field, [1, 0, 0, 0, 0, 0, 0, 0])
        return self.get_feedback()

    def command(self):
        pass

    def shutdown(self):
        self.disable()
        self.bus.can_bus.shutdown()


def main():
    a.enable()
    a.write_param(Parameter.RunMode, 1)

    a.write_param(Parameter.LimitSpd, 50.0)
    a.write_param(Parameter.LocKp, 50.0)
    a.write_param(Parameter.SpdKp, 1.0)

    a.write_param(Parameter.LocRef, 0)
    time.sleep(2)

    for _ in range(2):
        a.write_param(Parameter.LocRef, -1)
        time.sleep(2)
        a.write_param(Parameter.LocRef, 1)
        time.sleep(2)

    a.write_param(Parameter.LocRef, 0.0)
    time.sleep(2)
    a.shutdown()


try:
    a = Actuator(127)
    main()
except:
    a.shutdown()
