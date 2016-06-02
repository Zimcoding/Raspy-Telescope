import time
import io
import serial
import sys
import sensors
import logging

def write_to_serial(sercon, data):
        if sercon.isOpen():
            if data != None:
                sercon.write(":{}#".format(data))

serport = serial.Serial(port='/dev/ttyS0', baudrate=9600)

write_data = ["SH", "SN0100", "SD02", "FG"]

try:
    for cmd in write_data:
        print cmd
        write_to_serial(serport, cmd)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
serport.close()
        
            
#ser_port.serial_port.close()
