import time
import math
import os
import asyncio
import threading

import board

from adafruit_servokit import ServoKit
import adafruit_mpu6050
from ESC import ESC
from pymavlink import mavutil

from ipc import IPC, IPCMessage, ThreadID
from gps import gps_thread_exec

conn = mavutil.mavlink_connection('udpout:' + os.environ['MAV_HOST'] + ':14540')

i2c = board.I2C()

kit = ServoKit(channels=16, i2c=i2c)
esc = ESC(kit, 0)

imu2 = adafruit_mpu6050.MPU6050(i2c) 


async def esc_test():
    print('Running ESC test')
    print('Arming')
    esc.arm()
    print('Armed')
    max_val = 200

    for x in range(max_val):
        esc.throttle = (math.sin(time.time())+1)/2/1
        time.sleep(.05)
    esc.throttle = 0


async def imu_test():
    rx = 0
    ry = 0
    rz = 0
    print('Running IMU test')
    while True:
        print("Acceleration: X:{:.2f}, Y: {:.2f}, Z: {:.2f} m/s^2".format(*imu2.acceleration))
        print("Gyro X:{:.2f}, Y: {:.2f}, Z: {:.2f} rads/s".format(*imu2.gyro))
        rx += imu2.gyro[0]/math.pi*180
        ry += imu2.gyro[1]/math.pi*180
        rz += imu2.gyro[2]/math.pi*180
        print(rx, ry , rz)
        print("")
        time.sleep(0.5)


async def servo_test(channel: int):
    print("Running servo test")
    kit.servo[channel].set_pulse_width_range(500, 2700)
    for x in range(5):
        if x % 2 == 0:
            kit.servo[channel].angle = 0
        else:
            kit.servo[channel].angle = 180
        time.sleep(.5)

#gps_serial_test()

async def wait_conn():
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    print('waiting')
    msg = None
    while not msg:
        conn.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = conn.recv_match()
        time.sleep(0.5)

async def mavhost():
    await wait_conn()

    while True:
        try:
            print(conn.recv_match().to_dict())
        except:
            pass
        time.sleep(0.1)


def main():
    print('Starting Helios II Avionics')
    ipc = IPC(ThreadID.MAIN, True)

    gps_thread = threading.Thread(target=gps_thread_exec)
    gps_thread.start()

    alive = True

    while True:
        print(ipc.recv())
        alive |= gps_thread.is_alive()
        if not alive:
            os.kill(os.getpid, -1)
            print('Death')
        time.sleep(.1)

main()
