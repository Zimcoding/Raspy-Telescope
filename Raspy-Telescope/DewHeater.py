# Copyright (c) 2016 Wim Rappe
# Author: Wim Rappe
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Import the sys for importing from another location
import sys
sys.path.insert(0, "../lib/Adafruit_Python_BME280")

import Adafruit_BME280
import Adafruit_MotorHAT as MtrHat
import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = MtrHat.Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(MtrHat.Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(MtrHat.Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(MtrHat.Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(MtrHat.Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

################ Actual program
DewHeater1 = mh.getMotor(1)
Dewheater2 = mh.getMotor(2)

sensor = Adafruit_BME280.BME280(mode=Adafruit_BME280.BME280_OSAMPLE_8)

#Read sensor and display
while (True):
    degrees = sensor.read_temperature()
    pascals = sensor.read_pressure()
    hectopascals = pascals / 100
    humidity = sensor.read_humidity()


    print 'Timestamp = {0:0.3f}'.format(sensor.t_fine)
    print 'Temp      = {0:0.3f} deg C'.format(degrees)
    print 'Pressure  = {0:0.2f} hPa'.format(hectopascals)
    print 'Humidity  = {0:0.2f} %'.format(humidity)

    #Start heater if humidity is to high
    if humidity > 80:
        print "Starting to heat..."
        DewHeater1.run(MtrHat.Adafruit_MotorHAT.FORWARD)
        DewHeater1.setSpeed(int((humidity-80)*10))
    else:
        DewHeater1.run(MtrHat.Adafruit_MotorHAT.RELEASE)
        print "Not heating"
    time.sleep(1)

    
    
