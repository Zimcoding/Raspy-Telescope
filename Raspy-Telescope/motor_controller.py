#!/usr/bin/python

import threading
import Queue
import time
import logging
import sys
sys.path.insert(0, "../lib/Adafruit_MotorHAT_mod")
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
import Adafruit_PWM_Servo_Driver

def main():
    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG, format="(%(name)s %(threadName)-9s) %(message)s")


class StepperController:

    def __init__(self, num, logger=None):
        self._number = num
        self._ppr = 96                          # pulses per revolution for stepper motor
        self.mh = Adafruit_MotorHAT()           # create a default object, no changes to I2C address or frequency
        self.myStepper = self.mh.getStepper(self._ppr, self._number)   #96 steps per revolution. Stepper port num.
        self.mypwm = Adafruit_PWM_Servo_Driver.PWM(0x60, debug=False)   #I2C address 0x60

        self.logger = logger or logging.getLogger(__name__)

        self.speed_pps = 20      #Speed in pulses per second
        self.steps = 0
        self.step_type = 0      #SINGLE = 1     DOUBLE = 2      INTERLEAVE = 3  MICROSTEP = 4

        self.motorq = Queue.Queue(5)

        self.motor_pos = 0
        self.motor_pos_new = 0
        self.motor_speed = 0
        self.motor_half_step = True
        self.motor_running = False
        self.inv_dir = True

        #Constants from motorhat library
        num -= 1

        self.FORWARD = 1
        self.BACKWARD = 2
        self.BRAKE = 3
        self.RELEASE = 4

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

    def write_motorq(self, command_tuple):  #Tuple: command, speed, direction, motor.handle
        self.motorq.put(command_tuple)
        self.logger.debug("Putting " + str(command_tuple) + " : " + str(self.motorq.qsize()) + " items in queue")

    # Disable motors/ no force on poles!
    def turn_off_all(self ):
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

    def motor_set_speed(self, speed):
        self.motor_speed = speed
        self.speed_pps = float(speed)/60*self._ppr
        self.logger.info("Speed: {} RPM".format(speed))
        if   self.speed_pps > 95 and  self.speed_pps < 193:
            self.mypwm.setPWMFreq( self.speed_pps/4)       # Speed is 4 steps per frequency puls
        elif  self.speed_pps < 96:
            self.mypwm.setPWMFreq(1600)
            self.myStepper.setSpeed(speed)
        else:
            self.logger.error("Speed {} rpm is to high or otherwise wrong".format(speed))
        return

    def motor_set_step_type(self, half_step):
        self.motor_half_step = half_step
        if half_step:
            self.step_type = 3
        else:
            self.step_type = 2
        return

    def start_motor (self, position=None):
        if self.step_type != 2 and self.step_type != 3:
            self.motor_set_step_type(self.motor_half_step)
        if not position == None:
            self.motor_pos_new = position
        direction = self.FORWARD if self.motor_pos_new > self.motor_pos else self.BACKWARD       #check direction (forward is True)
        if self.speed_pps > 95 and  self.speed_pps < 193:       #High speed stepping
            self.motorq.put(["fast", position, direction])
            self.logger.debug("Putting " + str(["fast", position, direction]) + " : " + str(self.motorq.qsize()) + " items in queue")
        elif self.speed_pps < 96:
            self.motorq.put(["slow", position, direction])
            self.logger.debug("speed_pps: {}".format(self.speed_pps))
            self.logger.debug("Putting " + str(["slow", position, direction]) + " : " + str(self.motorq.qsize()) + " items in queue")
        else:
            self.logger.error("Speed {} rpm is to high or otherwise wrong".format(self.motor_speed))
        return


    def release_motor (self):
        self.mh.setPin(self.AIN2, 0)
        self.mh.setPin(self.BIN1, 0)
        self.mh.setPin(self.AIN1, 0)
        self.mh.setPin(self.BIN2, 0)
        '''if self._number == 2:
            self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
            self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
        else :
            self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
            self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)'''

    def stop_motor (self):
        self.motorq.put(["stop", 0])
        self.myStepper.stop_stepper = True
        return

    def async_stop(self):
        self.myStepper.stop_stepper = True
        self.motorq.put(["exit", 0])
        return
    
    def calc_pos(self, direction, start_time, run_type, step_count):
        if run_type == "slow":
            #if step_count != None:
            if self.motor_pos_new > self.motor_pos:
                self.motor_pos = self.motor_pos + step_count
            else:
                self.motor_pos = self.motor_pos - step_count
        elif run_type == "fast":
            if self.motor_pos_new > self.motor_pos:
                self.motor_pos =self.motor_pos + int((time.time() - start_time)*self.speed_pps * (2 if self.motor_half_step else 1))
            else:
                self.motor_pos =self.motor_pos - int((time.time() - start_time)*self.speed_pps * (2 if self.motor_half_step else 1))

    def async_motor (self):
        steps = 0
        start_time = 0
        run_time = 0
        step_count = 0
        running_type = "fast"

        while True:
            if not self.motorq.empty():
                #Get items from Queue
                item = self.motorq.get()
                command = item[0]
                value = item[1]
                if len(item) == 3:
                    motor_dir = item[2]
                elif len(item) > 3:
                    motor_dir = item[2]
                    motor_handle = item[3]
                
                self.logger.info("Getting " + str(item) + " : " + str(self.motorq.qsize()) + " items in queue")
                #direction = self.motor_pos_new > self.motor_pos         #True = going out.
                steps = int(abs(self.motor_pos_new - self.motor_pos))   #Add steps if going out
                if command == "exit":
                    self.turn_off_all()
                    break
                elif command == "fast":
                    running_type = command
                    try:
                        if   self.motor_half_step:
                            #t= steps/pulses per second
                            run_time = (float(abs(steps))/float(self.speed_pps))/2
                        else:
                            run_time = (float(abs(steps))/float(self.speed_pps))
                        if run_time > 0.1:
                            if self.step_type == 2:
                                if (motor_dir != self.inv_dir):     
                                    self.mypwm.setPWM(self.AIN2, 0, 2048)
                                    self.mypwm.setPWM(self.BIN1, 1024, 3072)
                                    self.mypwm.setPWM(self.AIN1, 2048, 4095)
                                    self.mypwm.setPWM(self.BIN2, 3072, 512)
                                else:
                                    self.mypwm.setPWM(self.AIN2, 3072, 512)
                                    self.mypwm.setPWM(self.BIN1, 2048, 4095)
                                    self.mypwm.setPWM(self.AIN1, 1024, 3072)
                                    self.mypwm.setPWM(self.BIN2, 0, 2048)
                            elif self.step_type == 3:
                                if (motor_dir != self.inv_dir):
                                    self.mypwm.setPWM(self.AIN2, 0, 1536)
                                    self.mypwm.setPWM(self.BIN1, 1024, 2560)
                                    self.mypwm.setPWM(self.AIN1, 2048, 3584)
                                    self.mypwm.setPWM(self.BIN2, 3072, 512)
                                else:
                                    self.mypwm.setPWM(self.AIN2, 3072, 512)
                                    self.mypwm.setPWM(self.BIN1, 2048, 3584)
                                    self.mypwm.setPWM(self.AIN1, 1024, 2560)
                                    self.mypwm.setPWM(self.BIN2, 0, 1536)
                            else:
                                self.logger.error("Other step modes currently not supported")
                            start_time = time.time()
                            self.mypwm.setPWM(self.PWMA, 0, 4095)
                            self.mypwm.setPWM(self.PWMB, 0, 4095)
                            self.motor_running = True
                            self.logger.info("Going to position {}".format(self.motor_pos_new))
                        else:
                            pass
                    except:
                        self.logger.error("Start fast fault")
                elif command == "slow":
                    running_type = command
                    try:
                        if   self.step_type == 3:
                            #t= steps/pulses per second
                            run_time = (float(abs(steps))/float(self.speed_pps))/2
                        else:
                            run_time = (float(abs(steps))/float(self.speed_pps))
                        start_time = time.time()
                        self.motor_running = True
                        self.logger.info("Going to position {}".format(self.motor_pos_new))
                        step_count = self.myStepper.step(steps, (motor_dir != self.inv_dir),  self.step_type)
                        self.release_motor()
                    except:
                        self.logger.exception("Start slow fault")
                elif command == "DCMotor":
                    motor_handle.setSpeed(value)
                    motor_handle.run(motor_dir)
                    
                elif command == "stop" and self.motor_running:
                    self.release_motor()
                    self.motor_running = False

                    self.calc_pos(motor_dir, start_time, running_type, step_count)
                    run_time = 0
                    self.logger.info("Reached position {}, {}".format(self.motor_pos, motor_dir))
                else:
                    pass

            # Catch the time to stop the execution of fast moving motor.
            if self.motor_running and time.time() - start_time >= run_time:
                stop_time = time.time()
                self.logger.debug("Running time: {}, expected time: {}".format(stop_time - start_time, run_time))
                run_time = 0
                self.motor_running = False

                if running_type == "fast":
                    self.mypwm.setPWM(self.PWMA, 0, 0)
                    self.mypwm.setPWM(self.PWMB, 0, 0)
                    self.calc_pos(motor_dir, start_time, running_type, step_count)
                elif running_type == "slow":
                    self.calc_pos(motor_dir, start_time, running_type, step_count)

                self.logger.info("Reached position: {}, {}".format(self.motor_pos, motor_dir))
            time.sleep(0.05)

        return

class DCMotorController:

    def __init__(self, num):
        self._number = num
        self.mh = Adafruit_MotorHAT()   #Open motor hat instance
        self.motor = self.mh.getMotor(self._number)

    def stop_motor (self):
        self.motor.run(MtrHat.Adafruit_MotorHAT.RELEASE)

    def start_motor (self, contr_handle, duty_cycle, motor_dir):
        self.motor.setSpeed(duty_cycle)
        self.motor.run(motor_dir)
        logging.info("Heating at: {}% Duty Cycle".format(duty_cycle))
        #local_motor = self.motor
        #contr_handle.write_motorq(["DCMotor", duty_cycle, motor_dir, self.motor])


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

            controller.motor_set_speed(25)
            controller.motor_set_step_type(True)
            time.sleep(2)
            controller.start_motor(400)

            time.sleep(10)

        except KeyboardInterrupt:
            controller.async_stop()
            controller.turn_off_all()
            break
