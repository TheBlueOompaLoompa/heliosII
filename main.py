import board
from adafruit_servokit import ServoKit
import time
from ESC import ESC
import math
import adafruit_mpu6050
import serial
from pynmeagps import NMEAReader, NMEAMessage

i2c = board.I2C()

kit = ServoKit(channels=16, i2c=i2c)
esc = ESC(kit, 0)

imu2 = adafruit_mpu6050.MPU6050(i2c) 

gps_serial = serial.Serial('/dev/ttyS0')

def esc_test():
    print('Running ESC test')
    print('Arming')
    esc.arm()
    print('Armed')
    max_val = 200

    for x in range(max_val):
        esc.throttle = (math.sin(time.time())+1)/2/1
        time.sleep(.05)
    esc.throttle = 0


def imu_test():
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


def gps_serial_test():
    nmr = NMEAReader(gps_serial)
    while True:
        raw_data, parsed_data = nmr.read()
        if parsed_data is not None:
            msg: NMEAMessage = parsed_data
            #print(msg)
            if msg.msgID == 'GGA' and (not(msg.NS is int) or msg.NS == 0):
                print("No satellites")

def servo_test(channel: int):
    print("Running servo test")
    kit.servo[channel].set_pulse_width_range(500, 2700)
    for x in range(5):
        if x % 2 == 0:
            kit.servo[channel].angle = 0
        else:
            kit.servo[channel].angle = 180
        time.sleep(.5)

#gps_serial_test()

esc_test()
servo_test(1)
