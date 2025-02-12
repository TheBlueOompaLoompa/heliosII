from enum import Enum, auto
import zmq

class ThreadID(Enum):
    MAIN = auto()
    GPS = auto()
    FLIGHT_CONTROLLER = auto()
    FLIGHT_PLANNER = auto()
    MAVLINK = auto()
    ALL = auto()

class IPCMessage:
    def __init__(self, from_id: ThreadID, to_id: ThreadID, msg_type: Enum, data: tuple):
        self.from_id = from_id
        self.to_id = to_id
        self.msg_type = msg_type
        self.data = data

    def serialize(self) -> tuple:
        return (self.from_id.value, self.to_id.value, self.msg_type.value, self.data)

class IPC:
    socket: zmq.Socket

    def __init__(self, id: ThreadID, host = False):
        self.id = id
        self.context = zmq.Context()
        if host:
            self.socket = self.context.socket(zmq.REP)
            self.socket.bind("tcp://*:5555")
        else:
            self.socket = self.context.socket(zmq.REQ)
            self.socket.connect("tcp://localhost:5555")

    def send(self, data: tuple, to: ThreadID, msg_type: Enum):
        self.socket.send_pyobj((self.id, to, msg_type, data))

    def recv(self):
        return self.socket.recv_pyobj()

