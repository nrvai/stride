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
    KP_LIMIT = 500
    KD_LIMIT = 5

    def __init__(self, bus: RobstrideBus, motor_id: int, host_id: int = 0xAA):
        self.bus = bus
        self.motor_id = motor_id
        self.host_id = host_id
        self.id_field = self.motor_id | (self.host_id << 8)

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

        return Feedback(self.motor_id, status, angle, velocity, torque, temp)

    def enable(self) -> Feedback:
        self.bus.send(CommunicationType.Enable, self.id_field)
        return self.get_feedback()

    def disable(self) -> Feedback:
        self.bus.send(CommunicationType.Disable, self.id_field)
        return self.get_feedback()

    def request_feedback(self) -> Feedback:
        self.bus.send(CommunicationType.Feedback, self.id_field)
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

    def set_can_id(self, new_can_id: int) -> int:
        self.disable()
        new_id_field = self.motor_id | (self.host_id << 8) | (new_can_id << 16)

        self.motor_id = new_can_id
        self.id_field = self.motor_id | (self.host_id << 8)

        self.bus.send(CommunicationType.SetCanID, new_id_field)
        res = self.bus.recv()
        data = struct.unpack('<Q', res.data)[0]
        self.enable()
        return data

    def run_calibration(self) -> int:
        self.disable()
        calibration_id = 0x05000000 | (self.host_id << 8) | self.motor_id
        self.bus.send_raw(calibration_id, [8, 0, 0, 0, 0, 0, 0, 0])
        init_res = self.bus.recv()
        cali_res = self.bus.recv()
        self.enable()
        return init_res, cali_res

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
        id_field = self.motor_id | (norm_torque << 8)

        self.bus.send(CommunicationType.Control, id_field)
        return self.get_feedback()

    def command(self, torque: float, angle: float, velocity, kp: float, kd: float):
        def _normalize_value(value: float, max_value: float, symmetric=True):
            value = value + max_value if symmetric else value
            return int(value / (2 * max_value) * 65535)

        if abs(torque) > Actuator.TORQUE_LIMIT:
            raise ActuatorLimitException("Torque command is over actuator limit")
        if abs(angle) > Actuator.ANGLE_LIMIT:
            raise ActuatorLimitException("Angle command is over actuator limit")
        if velocity > Actuator.VELOCITY_LIMIT:
            raise ActuatorLimitException("Velocity command is over actuator limit")

        norm_torque = _normalize_value(torque, Actuator.TORQUE_LIMIT)
        norm_angle = _normalize_value(angle, Actuator.ANGLE_LIMIT)
        norm_velocity = _normalize_value(velocity, Actuator.VELOCITY_LIMIT)

        norm_kp = _normalize_value(kp, Actuator.KP_LIMIT, False)
        norm_kd = _normalize_value(kd, Actuator.KD_LIMIT, False)

        id_field = self.motor_id | (norm_torque << 8)

        angle_bytes = struct.pack('>H', norm_angle)
        velocity_bytes = struct.pack('>H', norm_velocity)
        kp_bytes = struct.pack('>H', norm_kp)
        kd_bytes = struct.pack('>H', norm_kd)

        data = [*angle_bytes, *velocity_bytes, *kp_bytes, *kd_bytes]

        self.bus.send(CommunicationType.Control, id_field, data)
        return self.get_feedback()

    def get_all_params(self):
        return {param: self.read_param(Parameter[param]) for param in Parameter._member_names_}

    def shutdown(self):
        self.disable()
        self.bus.can_bus.shutdown()
