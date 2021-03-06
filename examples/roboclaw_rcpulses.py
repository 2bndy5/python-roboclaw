""" On Roboclaw set switch 1 and 6 on. """
import time
from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685

from adafruit_motor import servo

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50

# The pulse range is [1250 (full forward), 1750 (full reverse)].
pulses[
    servo.ContinuousServo(pca.channels[7], min_pulse=1250, max_pulse=1750),
    servo.ContinuousServo(pca.channels[8], min_pulse=1250, max_pulse=1750)
]

pulses[0].throttle = 1
pulses[1].throttle = -1
time.sleep(2)

# stop
pulses[0].throttle = 0
pulses[1].throttle = 0

pca.deinit()
