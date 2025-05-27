import time
from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c)
pca.frequency = 50

start = 0
stop = 3000
ser1 = servo.Servo(pca.channels[16], min_pulse = start, max_pulse = stop)
ser2 = servo.Servo(pca.channels[17], min_pulse = start, max_pulse = stop)

x = input("Press ENTER to Start.")
ser2.angle = 0
ser1.angle = 0
x = input("Press ENTER to Pause")
ser2.angle = 45
ser1.angle = 45
x = input("Press ENTER to return")

ser1.angle = 0
ser2.angle = 0



