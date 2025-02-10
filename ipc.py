from enum import Enum, auto
import socket
from json import dumps, loads

class ThreadID(Enum):
    MAIN = auto()
    GPS = auto()
    FLIGHT_CONTROLLER = auto()
    FLIGHT_PLANNER = auto()
    MAVLINK = auto()
    ALL = auto()


class IPCMessage:
    def __init__(self, from_id: ThreadID, to_id: ThreadID, msg_type: int, data: tuple):
        self.from_id = from_id
        self.to_id = to_id
        self.msg_type = msg_type
        self.data = data

    def serialize(self) -> tuple:
        return (self.from_id.value, self.to_id.value, self.msg_type.value, self.data)


class IPC:
    socket: socket.socket

    def __init__(self, id: ThreadID, host = False, port = 5555):
        self.id = id
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if host:
            self.socket.bind(('127.0.0.1', port))
        else:
            self.socket.connect(('127.0.0.1', port))

            self.send((), ThreadID.MAIN, -1)

    def send(self, data: tuple, to: ThreadID, msg_type: int):
        self.socket.send(
            bytes(
                dumps((self.id.value, to.value, msg_type, data)),
                "utf-8"
            )
        )

    def send_to(self, data: tuple, to: ThreadID, addr: str, msg_type: int):
        self.socket.sendto(
            bytes(
                dumps((self.id.value, to.value, msg_type, data)),
                "utf-8"
            ),
            addr
        )

    def recv(self):
        buf, addr = self.socket.recvfrom(1024)
        raw = loads(buf)
        return IPCMessage(ThreadID(raw[0]), ThreadID(raw[1]), raw[2], raw[3]), addr

