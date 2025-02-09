import board
from adafruit_servokit import ServoKit
from enum import Enum
from time import sleep

class State(Enum):
    STOPPED = 0
    START_CALIBRATION = 1
    CALIBRATING = 2
    READY = 3
    ARMED = 4
    ARMING = 5

class ESC:
    state = State.STOPPED
    kit: ServoKit
    _throttle = 0
    
    def __init__(self, kit: ServoKit, channel: int, min_value = 700, max_value = 2000):
        self.kit = kit
        self.channel = channel
        self.kit.continuous_servo[self.channel].set_pulse_width_range(0, max_value)
        self.throttle = 0
        self.min_value = min_value
        self.max_value = max_value

    @property
    def throttle(self) -> float:
        return self._throttle

    @throttle.setter
    def throttle(self, value: float) -> None:
        self._throttle = min(max(value, 0), 1)
        self.kit.continuous_servo[self.channel].throttle = self._throttle

    def start_calibration(self):
        self.throttle = 1
        self.state = State.START_CALIBRATION
        print('Starting Calibration')

    def finish_calibration(self):
        self.throttle = 0
        self.state = State.CALIBRATING
        sleep(12)
        self.kit.continuous_servo[self.channel].set_pulse_width_range(0, self.max_value)
        self.throttle = 0
        sleep(2)
        self.state = State.STOPPED

        return self.state

    def arm(self):
        if self.state != State.STOPPED:
            return None
        self.kit.continuous_servo[self.channel].set_pulse_width_range(self.min_value, self.max_value)
        self.state = State.ARMING
        self.throttle = 0
        sleep(1)
        self.throttle = 0
        sleep(5)
        self.state = State.ARMED

        return self.state

