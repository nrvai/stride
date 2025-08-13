import time
import can
import math
import struct

from enum import IntEnum
from typing import Iterable
from dataclasses import dataclass

CAN_PORT = "can0"
ZERO_BYTE = [0, 0, 0, 0, 0, 0, 0, 0]


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


class RobstrideBus:
    def __init__(self):
        # self.bus = None
        self.bus = can.interface.Bus(interface='socketcan', channel=CAN_PORT)
        pass

    def recv(self):
        res = self.bus.recv()
        if not res or res.is_error_frame:
            Exception("response error")
        return res

    def send(self, comm_type: CommunicationType, id_field: int, data: bytes | bytearray | int | Iterable[int] | None = ZERO_BYTE):
        arb_id = id_field + (comm_type << 24)
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
        self.bus.send(msg)
        if True:
            print([msg])


class MotorMode(IntEnum):
    Reset = 0
    Calibration = 1
    Run = 2


class MotorError(IntEnum):
    Undervoltage = 1
    Overcurrent = 2
    Overtemperature = 4
    MagneticEncodingFault = 8
    HallEncodingFault = 16
    Uncalibrated = 32


@dataclass
class Feedback():
    motor_id: int
    errors: list
    angle: float
    velocity: float
    torque: float
    temp: float


class Actuator:
    def __init__(self, motor_id: int, host_id: int = 0xAA):
        self.bus = RobstrideBus()
        self.motor_id = motor_id
        self.host_id = host_id
        self.id_field = self.motor_id + (self.host_id << 8)

    def get_feedback(self):
        res = self.bus.recv()
        # assert(res.data[0] == 0x2)
        unpack = struct.Struct('<H').unpack

        angle_range = 8 * math.pi
        vel_range = 44
        torque_range = 17

        errors = (res.arbitration_id & 0x1F0000) >> 16
        angle = unpack(res.data[0:2])[0]
        angle = (angle / 65535 * angle_range) - angle_range / 2

        velocity = unpack(res.data[2:4])[0]
        velocity = (velocity / 65535 * vel_range) - vel_range / 2

        torque = unpack(res.data[4:6])[0]
        torque = (torque / 65535 * torque_range) - torque_range / 2

        temp = unpack(res.data[6:8])[0] / 10

        print(errors, angle, velocity, torque, temp)

        return

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
        int_pack = struct.Struct('<I').pack
        pid_bytes = int_pack(pid)[:2]

        if pid in [0x7005, 0x7026, 0x7028, 0x7029]:
            value_bytes = int_pack(value)
        else:
            value_bytes = struct.Struct('<f').pack(value)

        data = bytes([*pid_bytes, 0, 0, *value_bytes])
        self.bus.send(CommunicationType.WriteParameter, self.id_field, data)

        return self.get_feedback()

    def command(self):
        pass

    def get_id(self):
        pass

    def shutdown(self):
        self.disable()
        self.bus.bus.shutdown()


def main():
    a.enable()
    a.write_param(Parameter.RunMode, 1)

    a.write_param(Parameter.LimitSpd, 50.0)
    a.write_param(Parameter.LocKp, 30.0)
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
