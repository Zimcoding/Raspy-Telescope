import logging
import threading
import argparse
import sys
import time

import serial_ml
import motor_controller
import sensors

def init_main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-d', '--debug',
        help="Print lots of debugging statements",
        action="store_const", dest="loglevel", const=logging.DEBUG,
        default=logging.WARNING,
    )
    parser.add_argument(
        '-v', '--verbose',
        help="Be verbose",
        action="store_const", dest="loglevel", const=logging.INFO,
    )
    args = parser.parse_args()
    args.loglevel = logging.INFO #TODO: remove later

    logging.basicConfig(stream=sys.stdout, level=args.loglevel, format="(%(name)s %(threadName)-9s) %(message)s")

def serial_actions(motor_id, command, argument):
    log_cmd = True
    if command == 'GP': #Get Current Motor 1 Positon, Unsigned Hexadecimal
        log_cmd = False
        return "%0.4X" % controller.motor_pos #str(int(motor_pos)).zfill(4)
    elif command == 'GN': #Get the New Motor 1 Position, Unsigned Hexadecimal
        return "%0.4X" % controller.motor_pos_new
    elif command == 'GT': #Get the Current Temperature, Signed Hexadecimal
        log_cmd = False
        return "%0.4X" %((round(multisens.read_sensor()[0],0)+2**16)%2**16)
    elif command == 'GD': #Get the Motor 1 speed, valid options are "02, 04, 08, 10, 20"
        return "%0.2X" % (controller.motor_speed/6)
    elif command == 'GH': #"FF" if half step is set, otherwise "00"
        if controller.motor_half_step:
            return "FF"
        else:
            return "00"
    elif command == 'GI': #"01" if the motor is moving, otherwise "00"
        return str(int(controller.motor_running)).zfill(2)
    elif command == 'GB': #The current RED Led Backlight value, Unsigned Hexadecimal
        return "00"
    elif command == 'GV': #Code for current firmware version
        return "10"
    elif command == 'SP': #Set the Current Motor 1 Position, Unsigned Hexadecimal
        controller.motor_pos = int(argument, 16)
        return
    elif command == 'SN': #Set the New Motor 1 Position, Unsigned Hexadecimal
        controller.motor_pos_new = int(argument, 16)
        return 
    elif command == 'SF': #Set Motor 1 to Full Step
        controller.motor_set_step_type(False)
        return
    elif command == 'SH': #Set Motor 1 to Half Step
        controller.motor_set_step_type(True)
        return
    elif command == 'SD': #Set the Motor 1 speed, valid options are "02, 04, 08, 10, 20"
        controller.motor_set_speed(int(argument)*6)
        return 
    elif command == 'FG': #Start a Motor 1 move, moves the motor to the New Position.
        controller.start_motor()
        return 
    elif command == 'FQ': #Halt Motor 1 move, position is retained, motor is stopped.
        controller.stop_motor()
        return
    elif command == 'C#':
        log_cmd = False
        return
    else:
        logging.error("command not found")
    if log_cmd:
        logging.info("Command send: {}, {}".format(command, argument))




### Main ###

init_main()

controller = motor_controller.StepperController(2)
ser_port = serial_ml.Serial_ML()
try:
    multisens = sensors.EnvSensor()
except Exception as e:
    logging.exception("sensor not initialised: ")

motor_thread = threading.Thread(target=controller.async_motor)
motor_thread.start()

read_thread = threading.Thread(target=ser_port.read_from_serial, args=(ser_port.serial_port, ser_port.read_q))
read_thread.start()
write_thread = threading.Thread(target=ser_port.write_to_serial, args=(ser_port.serial_port, ser_port.write_q))
write_thread.start()

while True:
    try:
        if not ser_port.read_q.empty():
            decoded_cmd = ser_port.serial_decode(ser_port.read_q.get())
            logging.debug("Commands decoded: {}".format(decoded_cmd))
            str_to_serial = serial_actions(decoded_cmd[0],decoded_cmd[1],decoded_cmd[2])
            logging.debug("Return string: {}".format(str_to_serial)) 
            ser_port.write_q.put(str_to_serial)
    except KeyboardInterrupt:
        controller.async_stop()
        controller.turn_off_all()
        if read_thread.isAlive():
            ser_port.kill_thread = True
        break
    time.sleep(0.1)
            
ser_port.serial_port.close()
        
        
