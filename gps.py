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

    while True:
        nmr = NMEAReader(gps_serial)
        while True:
            raw_data, parsed_data = nmr.read()
            if parsed_data is not None:
                msg: NMEAMessage = parsed_data
                if msg.msgID == 'GGA':
                    handle_gga(msg, ipc)

def handle_gga(msg: NMEAMessage, ipc: IPC):
    if not(msg.NS is int) or msg.NS == 0:
        ipc.send((1), ThreadID.ALL, MsgType.NO_SIGNAL)
    elif msg.lat is int:
        lat = msg.lat
        lon = msg.lon
        ipc.send((lat, lon), ThreadID.FLIGHT_PLANNER, MsgType.POSITION)
