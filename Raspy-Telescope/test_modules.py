import time
import sys
sys.path.insert(0, "../lib/Adafruit_MotorHAT_mod")
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
import Adafruit_PWM_Servo_Driver


mh = Adafruit_MotorHAT()           # create a default object, no changes to I2C address or frequency
myStepper = mh.getStepper(self._ppr, self._number)steppers = [ Adafruit_StepperMotor( 1), Adafruit_StepperMotor( 2) ]
print Adafruit_StepperMotor.AIN2
