from enum import IntEnum
from typing import Iterable

import can

CAN_PORT = "can0"
ZERO_DATA = [0, 0, 0, 0, 0, 0, 0, 0]


class CommunicationType(IntEnum):
    DeviceID = 0
    Control = 1
    Feedback = 2
    Enable = 3
    Disable = 4
    MechanicalZero = 6
    SetCanID = 7
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

    def send_raw(self, arb_id: int, data: int, is_extended_id=True):
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=is_extended_id)
        self.can_bus.send(msg)
