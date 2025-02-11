import serial
from pynmeagps import NMEAReader, NMEAMessage
from enum import Enum
from ipc import IPC, IPCMessage, ThreadID

class MsgType(Enum):
    NO_SIGNAL = 0
    POSITION = 1

def gps_thread_exec():
    #  Socket to talk to server
    print("GPS IPC Connecting")
    ipc = IPC(ThreadID.GPS)

    gps_serial = serial.Serial('/dev/ttyS0')

    has_signal = False
    nmr = NMEAReader(gps_serial)
    while True:
        raw_data, parsed_data = nmr.read()
        if parsed_data is not None:
            msg: NMEAMessage = parsed_data
            if msg.msgID == 'GSV':
                has_signal = handle_gsv(msg, ipc)
            if msg.msgID == 'GGA' and has_signal:
                handle_gga(msg, ipc)

def handle_gsv(msg: NMEAMessage, ipc: IPC) -> bool:
    can_triangulate = msg.numSV > 3

    if not can_triangulate:
        ipc.send((), ThreadID.ALL, MsgType.NO_SIGNAL.value)

    return can_triangulate

def handle_gga(msg: NMEAMessage, ipc: IPC):
    lat = msg.lat
    lon = msg.lon
    ipc.send((lat, lon), ThreadID.FLIGHT_PLANNER, MsgType.POSITION.value)
