#!/usr/bin/python

import threading
import Queue
import time
import logging
import sys
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor, Adafruit_PWM_Servo_Driver

def main():
    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG, format="(%(name)s %(threadName)-9s) %(message)s")


class StepperController:
        motor_pos = 0
        motor_pos_new = 0
        motor_speed = 0
        motor_half_step = True
        motor_running = False

        def __init__(self, num, logger=None):
                self._number = num
                self._ppr = 96                          # pulses per revolution for stepper motor
                self.mh = Adafruit_MotorHAT()           # create a default object, no changes to I2C address or frequency
                self.myStepper = self.mh.getStepper(self._ppr, self._number)   #96 steps per revolution. Stepper port num.
                self.mypwm = Adafruit_PWM_Servo_Driver.PWM(0x60, debug=False)   #I2C address 0x60

                self.logger = logger or logging.getLogger(__name__)

                self.speed_pps = 0      #Speed in pulses per second
                self.steps = 0
                self.step_type = 0      #SINGLE = 1     DOUBLE = 2      INTERLEAVE = 3  MICROSTEP = 4

                self.motorq = Queue.Queue(5)

                #Constants from motorhat library
                num -= 1

                if (num == 0):
                        self.PWMA = 8
                        self.AIN2 = 9
                        self.AIN1 = 10
                        self.PWMB = 13
                        self.BIN2 = 12
                        self.BIN1 = 11
                elif (num == 1):
                        self.PWMA = 2
                        self.AIN2 = 3
                        self.AIN1 = 4
                        self.PWMB = 7
                        self.BIN2 = 6
                        self.BIN1 = 5
                else:
                        self.logger.error("MotorHAT Stepper must be between 1 and 2 inclusive")

        # Disable motors/ no force on poles!
        def turn_off_all(self ):
                self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
                self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
                self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
                self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

        def motor_set_speed(self, speed):
                motor_spd = speed
                self.speed_pps = speed/60*self._ppr
                self.logger.info("speed {}".format(speed))
                if   self.speed_pps > 95 and  self.speed_pps < 193:
                        self.mypwm.setPWMFreq( self.speed_pps/4)       # Speed is 4 steps per frequency puls
                elif  self.speed_pps < 96:
                        self.myStepper.setSpeed(speed)
                else:
                        self.logger.error("Speed {} rpm is to high or otherwise wrong".format(speed))
                return

        def motor_set_step_type(self, half_step):
                motor_half_step = half_step
                if half_step:
                        self.step_type = 3
                else:
                        self.step_type = 2
                return

        def start_motor (self, position=motor_pos_new):
                motor_pos_new = position
                if self.speed_pps > 95 and  self.speed_pps < 193:       #High speed stepping
                        if self.step_type == 2:
                                self.mypwm.setPWM(self.AIN2, 0, 2048)
                                self.mypwm.setPWM(self.BIN1, 1024, 3072)
                                self.mypwm.setPWM(self.AIN1, 2048, 4095)
                                self.mypwm.setPWM(self.BIN2, 3072, 512)
                        elif self.step_type == 3:
                                self.mypwm.setPWM(self.AIN2, 0, 1536)
                                self.mypwm.setPWM(self.BIN1, 1024, 2560)
                                self.mypwm.setPWM(self.AIN1, 2048, 3584)
                                self.mypwm.setPWM(self.BIN2, 3072, 512)
                        else:
                                self.logger.error("Other step modes currently not supported")
                         
                        self.motorq.put(["fast", position])
                        self.logger.debug("Putting " + str(["fast", position]) + " : " + str(self.motorq.qsize()) + " items in queue")
                elif self.speed_pps < 96:
                        self.motorq.put(["slow", position])
                        self.logger.debug("Putting " + str(["slow", position]) + " : " + str(self.motorq.qsize()) + " items in queue")
                else:
                        self.logger.error("Speed {} rpm is to high or otherwise wrong".format(speed))
                return
        def stop_motor (self):
                self.motorq.put(["stop", 0])
                return
        
        def async_stop(self):
                self.motorq.put(["exit", 0])
                return

        def async_motor (self):
                steps = 0
                start_time = 0
                run_time = 0
                
                while True:
                        if not self.motorq.empty():
                                item = self.motorq.get()
                                command = item[0]
                                value = item[1]
                                direction = value > self.motor_pos   #True = going out.
                                steps = value - self.motor_pos       #Add steps if going out
                                self.logger.debug("Getting " + str(item) + " : " + str(self.motorq.qsize()) + " items in queue")
                                if command == "exit":
                                        #TODO: exec stop commands first
                                        self.turn_off_all()
                                        break
                                elif command == "fast":
                                        try:
                                              start_time = time.time()
                                              self.mypwm.setPWM(self.PWMA, 0, 4095)
                                              self.mypwm.setPWM(self.PWMB, 0, 4095)
                                              motor_running = True
                                              run_time = (float(steps)/float(self.speed_pps))  #t= steps/pulses per second
                                              #time.sleep(1-0.0025)
                                        except:
                                                self.logger.error("Start fast fault")
                                elif command == "slow":
                                        try:
                                            motor_running = True
                                            self.myStepper.step(steps, direction,  self.step_type)
                                        except:
                                                self.logger.error("Start slow fault")
                                elif command == "stop":
                                        run_time = time.time() - start_time + 1
                        if run_time and time.time() - start_time >= run_time:
                                stop_time = time.time()
                                self.logger.debug("Running time: {}, expected time: {}".format(stop_time - start_time, run_time))
                                run_time = 0
                                motor_running = False
                                if command == "fast":                               
                                        self.mypwm.setPWM(self.PWMA, 0, 0)
                                        self.mypwm.setPWM(self.PWMB, 0, 0)
                                        motor_pos =(start_time - stop_time)*self.speed_pps
                                        #print ("time differnce", str(time.time()-start))
                                elif command == "slow":
                                         self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
                                         motor_pos =(start_time - stop_time)*self.speed_pps
                                         self.logger.debug("Entering stop at slow speed")
     
                        time.sleep(0.1)
                        
                return

if __name__ == '__main__':
        main()

        controller = StepperController(2)

        motor_thread = threading.Thread(target=controller.async_motor)
        #read_thread.setDaemon(True)
        motor_thread.start()
        #controller.motorq.put(["exit", 0])
        time.sleep(2)
        while True: 
                try:
                        
                        controller.motor_speed(15)
                        controller.motor_step_type(True)
                        time.sleep(2)
                        controller.start_motor(200)

                        time.sleep(10)

                except KeyboardInterrupt:        
                        controller.async_stop()
                        controller.turn_off_all()
                        break
