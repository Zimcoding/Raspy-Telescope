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
#sys.path.insert(0, "../lib/Adafruit_Python_BME280")
#sys.path.insert(0, "../lib/Adafruit_MotorHAT_mod")

import sensors
import motor_controller
import time
import atexit
import csv
import os
import logging
import threading

def main():
    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG, format="(%(name)s %(threadName)-9s) %(message)s")

class DewHeater:
    def __init__(self, motor_ctrl_handle):
        self.status = "not running"
        self.temperature = 0
        self.DewHeater1 = motor_controller.DCMotorController(1)
        self.Dewheater2 = motor_controller.DCMotorController(2)
        self.sensor = sensors.EnvSensor()
        self.controller = motor_ctrl_handle
        self.shutdown = False


    #Read sensor and display
    def heat(self, interval=5):
        last_val = 0
        
        if interval < 0.1:
            interval = 1
        while not self.shutdown:
            self.sensor.read_sensor()
            self.temperature = self.sensor.temperature

            logging.debug('Timestamp = {0:0.3f}'.format(self.sensor.time_fine))
            logging.debug('Temp      = {0:0.3f} deg C'.format(self.sensor.temperature))
            logging.debug('Pressure  = {0:0.2f} hPa'.format(self.sensor.pressure))
            logging.debug('Humidity  = {0:0.2f} %'.format(self.sensor.humidity))

            #Start heater if humidity is to high
            if not (last_val - 1 < self.sensor.humidity < last_val + 1):
                last_val = self.sensor.humidity
                if self.sensor.humidity > 80:
                    logging.debug("Starting to heat...")
                    self.DewHeater1.start_motor(self.controller, int((self.sensor.humidity-80)*5), self.DewHeater1.mh.FORWARD)
                else:
                    self.DewHeater1.start_motor(self.controller, 0, self.DewHeater1.mh.RELEASE)
                    logging.debug("Not heating")
            time.sleep(interval)

if __name__ == '__main__':
    main()
    controller = motor_controller.StepperController(1)
    heater = DewHeater(controller)
    motor_thread = threading.Thread(target=controller.async_motor)
    #read_thread.setDaemon(True)
    motor_thread.start()

    heater.heat()
        
    
