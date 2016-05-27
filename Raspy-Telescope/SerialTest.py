import time
import io
import serial
import threading
import sys
import Queue

port = '/dev/ttyS1'
baudrate = 9600
eol_character = '#'
kill_thread = False

motor_pos = 0

read_q =  Queue.Queue()
#rx_q =  Queue.Queue()
write_q =  Queue.Queue()
#tx_q =  Queue.Queue()

# configure the serial connections (the parameters differs on the device you are connecting to)
try:
    serial_port = serial.Serial(port, baudrate)
    print "Port opened: '{}'".format(serial_port.port)
except serial.SerialException, e:
    print "Could not open serial port '{}': {}".format(port, e)

### Definitions of fundtions ###

def read_serial_line(sercon, eol):
    
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

def read_from_serial(sercon, rx_q):
    while sercon.isOpen() :
        try:
            serial_line = read_serial_line(sercon, eol_character)
            if serial_line:
                rx_q.put(serial_line)
                #print serial_line
                sys.stdout.flush()
        except kill_thread:
            break


def write_to_serial(sercon, tx_q):
    while sercon.isOpen():
        while not tx_q.empty():
            print 'in write thread'
            sys.stdout.flush()
            cmd_to_send = tx_q.get()

            sercon.write(cmd_to_send.encode())
    
            tx_q.task_done()

def serial_decode(ser_string):
    ser_string.strip('#')
    #Split string after 2
    command = ser_string[0:2]
    argument = ser_string[2:6]
    print 'Command, Argument: {}, {}'.format(command, argument)

    if command == 'GP':
        return str(motor_pos).zfill(4)
    else:
        return 'command not found'

### Program ###

read_thread = threading.Thread(target=read_from_serial, args=(serial_port, read_q))
#read_thread.setDaemon(True)
read_thread.start()
write_thread = threading.Thread(target=write_to_serial, args=(serial_port, write_q))
#write_thread.setDaemon(True)
write_thread.start() 

try:
    while True:
        while not read_q.empty():
            str_from_serial = read_q.get()
            str_to_serial = serial_decode(str_from_serial)
            write_q.put(str_to_serial)
            print 'From , to serial: {}, {} '.format(str_from_serial, str_to_serial)
            read_q.task_done()
        time.sleep(1)
        #print 'thread is alive %d' %read_q.qsize()

except KeyboardInterrupt:
    if read_thread.isAlive():
        kill_thread = True
        
serial_port.close()

