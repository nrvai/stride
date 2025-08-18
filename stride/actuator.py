import math
import struct
import time
from collections.abc import Buffer
from dataclasses import dataclass
from enum import IntEnum

from stride.communication import CommunicationType, Parameter, RobstrideBus


class ActuatorLimitException(ValueError):
    pass


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


class Actuator:
    ANGLE_LIMIT = 4 * math.pi
    VELOCITY_LIMIT = 44
    TORQUE_LIMIT = 17

    def __init__(self, motor_id: int, host_id: int = 0xAA):
        self.bus = RobstrideBus()
        self.motor_id = motor_id
        self.host_id = host_id
        self.id_field = self.motor_id + (self.host_id << 8)

    def get_feedback(self) -> Feedback:
        def _unpack(x: Buffer) -> int:
            return struct.unpack('>H', x)[0]

        def _unpack_range(x: Buffer, r: int | float) -> float:
            return _unpack(x) / 65535 * r - r / 2

        def _get_error_bit(error_bits: int, i: int) -> bool:
            return bool(error_bits & (1 << i))

        res = self.bus.recv()

        angle = _unpack_range(res.data[0:2], Actuator.ANGLE_LIMIT * 2)
        velocity = _unpack_range(res.data[2:4], Actuator.VELOCITY_LIMIT * 2)
        torque = _unpack_range(res.data[4:6], Actuator.TORQUE_LIMIT * 2)

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

        return Feedback(1, status, angle, velocity, torque, temp)

    def enable(self) -> Feedback:
        self.bus.send(CommunicationType.Enable, self.id_field)
        return self.get_feedback()

    def disable(self) -> Feedback:
        self.bus.send(CommunicationType.Disable, self.id_field)
        return self.get_feedback()

    def write_param(self, pid: Parameter, value: int | float) -> Feedback:
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
        return struct.unpack('<f', res.data[4:])[0]

    def get_device_info(self) -> int:
        self.bus.send(CommunicationType.DeviceID, self.id_field)
        res = self.bus.recv()
        return struct.unpack('<Q', res.data)[0]

    def set_mechanical_zero(self) -> Feedback:
        self.bus.send(CommunicationType.MechanicalZero, self.id_field, [1, 0, 0, 0, 0, 0, 0, 0])
        return self.get_feedback()

    def torque_command(self, torque: float) -> Feedback:
        if abs(torque) > Actuator.TORQUE_LIMIT:
            raise ActuatorLimitException("Torque command is over actuator limit")

        norm_torque = int((torque + Actuator.TORQUE_LIMIT) / (2 * Actuator.TORQUE_LIMIT) * 65535)
        id_field = self.motor_id + (norm_torque << 8)

        self.bus.send(CommunicationType.Control, id_field)
        return self.get_feedback()

    def shutdown(self):
        self.disable()
        self.bus.can_bus.shutdown()


def main():
    a.enable()
    a.write_param(Parameter.RunMode, 1)  # put motor into operational mode
    a.set_mechanical_zero()
    a.write_param(Parameter.LocKp, 40)
    a.write_param(Parameter.SpdKp, 6)
    a.write_param(Parameter.SpdKi, 0.02)
    a.write_param(Parameter.LimitSpd, 44)

    a.write_param(Parameter.LocRef, 0.6)

    # a.torque_command(1.0) # apply 1Nm torque

    time.sleep(2)
    # a.torque_command(-1.0) # apply 1Nm torque in counter-clockwise
    # time.sleep(1)
    q = a.read_param(Parameter.MechPos)
    print(q)

    a.shutdown()


if __name__ == "__main__":
    try:
        a = Actuator(127)
        main()
    except KeyboardInterrupt:
        a.shutdown()
