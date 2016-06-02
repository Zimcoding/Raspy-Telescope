import time
import io
import serial
import threading
import sys
import Queue
import sensors
import logging

#TODO:
motor_pos = 0
motor_pos_new = 0
motor_speed = 0
motor_half_step = False
motor_moving = False

def main():
    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG, format="(%(name)s %(threadName)-9s) %(message)s")

class Serial_ML:
    def __init__(self, serialport='/dev/ttyS1', baudrate=9600, eol='#'):
        self.port = serialport
        self.baudrate = baudrate
        self.eol_character = eol
        self.kill_thread = False

        self.version = 1

        self.read_q =  Queue.Queue(10)
        self.write_q =  Queue.Queue(10)

        # configure the serial connections (the parameters differs on the device you are connecting to)
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate)
            logging.debug("Port opened: '{}'".format(self.serial_port.port))
        except serial.SerialException, e:
            logging.error("Could not open serial port '{}': {}".format(self.port, e))

        '''try:
            self.sensor = sensors.EnvSensor()
        except Exception as e:
            logging.exception("sensor not initialised: ")'''

    def read_serial_line(self, sercon, eol):
        
        #logging.debug(str(eol))
        leneol = len(eol)
        line = bytearray()
        while True:
            c = sercon.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)

    def read_from_serial(self, sercon, rx_q):
        while sercon.isOpen() :
            try:
                serial_line = self.read_serial_line(sercon, self.eol_character)
                if serial_line:
                    rx_q.put(serial_line)
                    logging.debug("Putting " + str(serial_line) + " : " + str(rx_q.qsize()) + " items in receive queue")
            except Exception as e:
                logging.exception("Read from serial went wrong: ")
            finally:
                time.sleep(0.05)
            if self.kill_thread:
                logging.debug("kill thread called")
                break


    def write_to_serial(self, sercon, tx_q):
        while sercon.isOpen():
            if not tx_q.empty():
                cmd_to_send = tx_q.get()
                if cmd_to_send != None:
                    logging.debug("Getting " + str(cmd_to_send) + " : " + str(tx_q.qsize()) + " items in send queue")
                    sercon.write("{}#".format(cmd_to_send.encode()))
                    tx_q.task_done()
            else:
                time.sleep(0.05)

    def serial_decode(self, ser_string):

        motor_pos = 0
        motor_pos_new = 0
        motor_speed = 0
        motor_half_step = False
        motor_moving = False

        version = 1
        length = len(ser_string)
        if ser_string[0] == ':' and ser_string[length-1] == '#':
            if ser_string[1] in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]:
                motor_id = ser_string[1]
                command = ser_string[2:4]
                argument = ser_string[4:(length-1)]
            else:
                motor_id = 1
                command = ser_string[1:3]
                argument = ser_string[3:(length-1)]
        else:
            motor_id = 0
            logging.error("Error: String not correctly formatted")
            
        return (motor_id, command, argument)  
        logging.debug("Command, Argument: {}, {}".format(command, argument))
        
        #Moved to main
        '''if command == 'GP': #Get Current Motor 1 Positon, Unsigned Hexadecimal
            return "%0.4X" % motor_pos #str(int(motor_pos)).zfill(4)
        elif command == 'GN': #Get the New Motor 1 Position, Unsigned Hexadecimal
            return "%0.4X" % motor_pos_new
        elif command == 'GT': #Get the Current Temperature, Signed Hexadecimal
            return "%0.4X" %((round(self.sensor.read_sensor()[0],0)+2**16)%2**16)
        elif command == 'GD': #Get the Motor 1 speed, valid options are "02, 04, 08, 10, 20"
            return "%0.4X" % motor_speed
        elif command == 'GH': #"FF" if half step is set, otherwise "00"
            if motor_half_step:
                return "FF"
            else:
                return "00"
        elif command == 'GI': #"01" if the motor is moving, otherwise "00"
            return str(int(motor_moving)).zfill(2)
        elif command == 'GB': #The current RED Led Backlight value, Unsigned Hexadecimal
            return "00"
        elif command == 'GV': #Code for current firmware version
            return str(version).zfill(2)
        elif command == 'SP': #Set the Current Motor 1 Position, Unsigned Hexadecimal
            motor_pos = int(argument, 16)
            return
        elif command == 'SN': #Set the New Motor 1 Position, Unsigned Hexadecimal
            motor_pos_new = int(argument, 16)
            return 
        elif command == 'SF': #Set Motor 1 to Full Step
            return str(motor_pos).zfill(4)
        elif command == 'SH': #Set Motor 1 to Half Step
            return str(motor_pos).zfill(4)
        elif command == 'SD': #Set the Motor 1 speed, valid options are "02, 04, 08, 10, 20"
            return str(motor_pos).zfill(4)
        elif command == 'FG': #Start a Motor 1 move, moves the motor to the New Position.
            return str(motor_pos).zfill(4)
        elif command == 'FQ': #Halt Motor 1 move, position is retained, motor is stopped.
            return str(motor_pos).zfill(4)
        elif command == 'C#':
            return
        else:
            logging.error("command not found")'''

### Program ###
if __name__ == '__main__':
    main()
    ser_port = Serial_ML()
    read_thread = threading.Thread(target=ser_port.read_from_serial, args=(ser_port.serial_port, ser_port.read_q))
    #read_thread.setDaemon(True)
    read_thread.start()
    write_thread = threading.Thread(target=ser_port.write_to_serial, args=(ser_port.serial_port, ser_port.write_q))
    #write_thread.setDaemon(True)
    write_thread.start() 

    try:
        while True:
            while not ser_port.read_q.empty():
                str_from_serial = ser_port.read_q.get()
                str_to_serial = ser_port.serial_decode(str_from_serial)
                ser_port.write_q.put(str_to_serial)
                logging.debug('From , to serial: {}, {} '.format(str_from_serial, str_to_serial))
                ser_port.read_q.task_done()
            time.sleep(1)
            #print 'thread is alive %d' %read_q.qsize()

    except KeyboardInterrupt:
        if read_thread.isAlive():
            ser_port.kill_thread = True
            
    ser_port.serial_port.close()

