#!/usr/bin/python

import threading
import Queue
import time
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor, Adafruit_PWM_Servo_Driver


class StepperController:
        motor_pos = 0

        #mypwm.setPWM(2, 0, 255*16)
        #mypwm.setPWM(7, 0, 255*16)

        def __init__(self, num):
                self._number = num
                self._ppr = 96                          # pulses per revolution for stepper motor
                self.mh = Adafruit_MotorHAT()           # create a default object, no changes to I2C address or frequency
                self.myStepper = self.mh.getStepper(self._ppr, self._number)   #96 steps per revolution. Stepper port num.
                self.mypwm = Adafruit_PWM_Servo_Driver.PWM(0x60, debug=False)   #I2C address 0x60

                self.speed_pps = 0      #Speed in pulses per second
                self.steps = 0
                self.step_type = 0      #SINGLE = 1     DOUBLE = 2      INTERLEAVE = 3  MICROSTEP = 4

                self.motorq = Queue.Queue()
                self._running = True

        # Disable motors/ no force on poles!
        def turn_off_all(self ):
                self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
                self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
                self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
                self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

        def motor_speed(self, speed):
                self.speed_pps = speed/60*self._ppr
                if   self.speed_pps > 95 and  self.speed_pps < 193:
                        self.mypwm.setPWMFreq( self.speed_pps/4)       # Speed is 4 steps per freauency puls
                elif  self.speed_pps < 96:
                        self.mystepper.setspeed(speed)
                else:
                        raise NameError("Speed {} rpm is to high or otherwise wrong".format(speed))
                return

        def motor_step_type(self, half_step):
                if half_step:
                        self.step_type = 3
                else:
                        self.step_type = 2
                return

        def start_motor (self, position):
                if self.speed_pps > 95 and  self.speed_pps < 193:       #High speed stepping
                        if self.step_type == 2:
                                self.mypwm.setPWM(3, 0, 2048)
                                self.mypwm.setPWM(5, 1024, 3072)
                                self.mypwm.setPWM(4, 2048, 4095)
                                self.mypwm.setPWM(6, 3072, 512)
                        elif self.step_type == 3:
                                self.mypwm.setPWM(3, 0, 1536)
                                self.mypwm.setPWM(5, 1024, 2560)
                                self.mypwm.setPWM(4, 2048, 3584)
                                self.mypwm.setPWM(6, 3072, 512)
                        else:
                                raise NameError("Other step modes currently not supported")
                        self.motorq.put(["fast", position])
                elif self.speed_ < 96:
                        self.motorq.put(["slow", position])
                else:
                        raise NameError("Speed {} rpm is to high or otherwise wrong".format(speed))
                return
        
        def async_stop(self):
                self.motorq.put(["exit", 0])
                return

        def async_motor (self):
                value = 0
                direction = True   #True = going out.
                steps = 0
                start = 0
                run_time = 0
                while True:
                        if not self.motorq.empty():
                                command = self.motorq.get()[0]
                                value = self.motorq.get()[1]
                                direction = value > self.motor_pos   #True = going out.
                                steps = value - self.motor_pos       #Add steps if going out
                                print "command {}, value {}".format(command, value)
                                if command == "exit":
                                        #TODO: exec stop commands first
                                        self.turn_off_all()
                                        break
                                elif command == "fast":
                                        try:
                                              start = time.time()
                                              self.mypwm.setPWM(2, 0, 4095)
                                              self.mypwm.setPWM(7, 0, 4095)

                                              run_time = (steps/self.speed_pps)  #t= steps/pulses per second
                                              time.sleep(1-0.0025)
                                        except:
                                                raise NameError("Start fast fault")
                                elif command == "slow":
                                        try:
                                                self.myStepper.step(steps, direction,  self.step_type)
                                        except:
                                                raise NameError("Start sloz fault")        
                        if start - time.time() >= run_time:
                                        if command == "fast":                               
                                                self.mypwm.setPWM(2, 0, 0)
                                                self.mypwm.setPWM(7, 0, 0)
                                                #print ("time differnce", str(time.time()-start))
                                        elif command == "slow":
                                                self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
                                                self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
                        #except:
                                #print "Values not initialized yet."
                        #        raise NameError("Values not initialised")
                                
                        time.sleep(0.05)
                        
                return

controller = StepperController(2)

#motor_thread = threading.Thread(target=controller.async_motor)
#read_thread.setDaemon(True)
#motor_thread.start()
#controller.motorq.put(["exit", 0])
while True: 
        try:
                
                controller.motor_speed(60)
                controller.motor_step_type(True)
                controller.start_motor(1000)

                time.sleep(10)

        except KeyboardInterrupt:        
                controller.motorq.put(["exit", 0])
                controller.turn_off_all()
                pass
