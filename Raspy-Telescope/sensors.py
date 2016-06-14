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
import time
import atexit
import csv
import os

class EnvSensor:
    def __init__(self):
        self.sensor = Adafruit_BME280.BME280(mode=Adafruit_BME280.BME280_OSAMPLE_8)
        self.temperature = 0
        self.pressure = 0
        self.humidity = 0
        self.time_fine = 0
        
    #Open file to log to
    def data_logger(self, data, path):
        if os.path.exists(path):
            log_file = open(path, "ab")
        else:
            log_file = open(path, "wb")
        datalogger = csv.writer(LogFile, dialect="excel-tab")
        datalogger.writerow(data)
        log_file.close()

    #Read sensor and display
    def read_sensor (self):
        self.time_fine = self.sensor.t_fine
        self.temperature = self.sensor.read_temperature()
        self.pressure = (self.sensor.read_pressure())/100
        self.humidity = self.sensor.read_humidity()


            #print 'Timestamp = {0:0.3f}'.format(sensor.t_fine)
            #print 'Temp      = {0:0.3f} deg C'.format(degrees)
            #print 'Pressure  = {0:0.2f} hPa'.format(hectopascals)
            #print 'Humidity  = {0:0.2f} %'.format(humidity)

        return (self.temperature, self.pressure, self.humidity) 

    
#niets = EnvSensor()
#print niets.read_sensor()

