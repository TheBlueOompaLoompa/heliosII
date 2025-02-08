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
    
    def __init__(self, kit: ServoKit, min_value = 700, max_value = 2000):
        self.kit = kit
        self.kit.continuous_servo[0].set_pulse_width_range(0, max_value)
        self.kit.continuous_servo[0].throttle = 0
        self.min_value = min_value
        self.max_value = max_value

    @property
    def throttle(self) -> float:
        return self._throttle

    @throttle.setter
    def throttle(self, value: float) -> None:
        self._throttle = min(max(value, 0), 1)
        self.kit.continuous_servo[0].throttle = self._throttle

    def start_calibration(self):
        self.kit.continuous_servo[0].throttle = 1
        self.state = State.START_CALIBRATION
        print('Starting Calibration')

    def finish_calibration(self):
        self.kit.continuous_servo[0].throttle = 0
        self.state = State.CALIBRATING
        sleep(12)
        self.kit.continuous_servo[0].set_pulse_width_range(0, self.max_value)
        self.kit.continuous_servo[0].throttle = 0
        sleep(2)
        self.state = State.STOPPED

        return self.state

    def arm(self):
        if self.state != State.STOPPED:
            return None
        self.kit.continuous_servo[0].set_pulse_width_range(self.min_value, self.max_value)
        self.state = State.ARMING
        self.kit.continuous_servo[0].throttle = 0
        sleep(1)
        self.kit.continuous_servo[0].throttle = 0
        sleep(5)
        self.state = State.ARMED

        return self.state

