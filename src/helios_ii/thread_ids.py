import zmq
from enum import Enum

class ThreadID(Enum):
    MAIN = auto()
    GPS = auto()
    FLIGHT_CONTROLLER = auto()
    FLIGHT_PLANNER = auto()
    MAVLINK = auto()
    ALL = auto()

def create_msg(from_id: ThreadID, to_id: ThreadID, msg_type: int, data: tuple):
    return (from_id, to_id, msg_type, data)
