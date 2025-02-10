from enum import Enum, auto
import board
from adafruit_servokit import ServoKit
import adafruit_mpu6050
import math
import time

from ipc import IPC, IPCMessage, ThreadID
from ESC import ESC

class FCMsg(Enum):
    IMU_DATA = auto()
    ARM = auto()
    SET_ESC_SPEED = auto()
    SET_SERVO_ANGLE = auto()

def flight_controller_thread_exec():
    i2c = board.I2C()

    kit = ServoKit(channels=16, i2c=i2c)
    esc = ESC(kit, 0)

    imu2 = adafruit_mpu6050.MPU6050(i2c) 

    print("FC IPC Connecting")
    ipc = IPC(ThreadID.FLIGHT_CONTROLLER)

    print("Running servo test")
    channel = 1
    kit.servo[channel].set_pulse_width_range(500, 2700)
    for x in range(5):
        if x % 2 == 0:
            kit.servo[channel].angle = 0
        else:
            kit.servo[channel].angle = 180
        time.sleep(.5)

    rx = 0
    ry = 0
    rz = 0
    print('Running IMU test')
    while True:
        ipc.send("Acceleration: X:{:.2f}, Y: {:.2f}, Z: {:.2f} m/s^2".format(*imu2.acceleration) + "Gyro X:{:.2f}, Y: {:.2f}, Z: {:.2f} rads/s".format(*imu2.gyro), ThreadID.FLIGHT_PLANNER, FCMsg.IMU_DATA.value)

        rx += imu2.gyro[0]/math.pi*180
        ry += imu2.gyro[1]/math.pi*180
        rz += imu2.gyro[2]/math.pi*180

        time.sleep(.01)

