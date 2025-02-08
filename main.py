import board
from adafruit_servokit import ServoKit
import time
from ESC import ESC
import math

i2c = board.I2C()
kit = ServoKit(channels=16, i2c=i2c)
esc = ESC(kit)

print('Arming')
esc.arm()
print('Armed')
max_val = 200

for x in range(max_val):
    esc.throttle = (math.sin(time.time())+1)/2/4
    time.sleep(.05)
esc.throttle = 0
