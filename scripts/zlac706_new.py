#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import serial
import time
import struct

motor_speed_mode = b'\x02\x00\xC4\xC6'
motor_status = b'\x80\x00\x80'
motor_start = b'\x00\x00\x01\x01'
motor_stop = b'\x00\x00\x00\x00'

class SpeedMotor:
    def __init__(self, device):
        # 
        self.rel_speed = 0
        #Set the speed
        self.set_speed = 0
        self.x_or_y_position = 0 
        # Operating status 
        self.run = False
        # Fault status
        self.fault = None
        self.voltage = 0
        self.current = 0
        #Set serial communication
        self.serial = serial.Serial(device, 57600)
        self.serial.timeout = 0

        # Set to speed mode
        self.serial.write(motor_speed_mode)
        #time.sleep(0.1)
                 #Set acceleration and deceleration
        self.serial.write(b'\x0A\x14\x14\x32')
        #time.sleep(0.1)

    def motor_speed_set(self):
        #print("set motor speed", self.set_speed)
        a1 = 6
        m_speed = self.set_speed * 16384 / 6000
        a4 = check_code(a1, m_speed)
        self.serial.write(struct.pack(">BhB", int(a1), int(m_speed), int(a4)))

    def set_status(self):
            self.serial.write(motor_status)
           # time.sleep(0.2)

    def motor_start(self):
        self.serial.write(motor_start)
        self.run = True

    def motor_stop(self):
        self.run = False
        self.serial.write(motor_stop)

    def read_motor(self):
        n = self.serial.inWaiting() # Wait for the arrival of the data and get the length of the data
        if n: # if there is data
            n = self.serial.read(n) # read n bits of data
            s = [ord(x) for x in bytes(n)]
            #print(s)
            #if len(s) == 32:
            for i in range(int(len(s) / 4)):
               addr = s[4 * i]
               if addr == 228:
                   high_data = s[4 * i + 1]
                   low_data = s[4 * i + 2]
                   #print(high_data, low_data)
                   if high_data == 255 or high_data == 199:
                       low_data -= 256
                   if high_data == 1:
                       low_data += 256
                   if high_data == 254:
                       low_data -= 512
                   #print(high_data, low_data)
                   self.rel_speed = (low_data) * 6000 / 16384
                   #  print(self.rel_speed)
        '''
            for i in range(len(s)):
                s[i] = s[i]
 
            if len(s) == 32:
                #print("CHECK",s)
                for i in range(int(len(s) / 4)):
                    addr = s[4 * i]
                    if addr == 128:
                        if s[2] == 0:
                            print("stop state")
                        elif s[2] == 1:
                            print("Startup Status")
                        elif s[2] == 2:
                            print("overcurrent")
                        elif s[2] == 4:
                            print("overvoltage")
                        elif s[2] == 8:
                            print("Encoder Fault")
                        elif s[2] == 16:
                            print("overheat")
                        elif s[2] == 32:
                            print("undervoltage")
                        elif s[2] == 64:
                            print("overload")
                    elif addr == 225:
                        low_data = s[4 * i + 2]
                        self.voltage = low_data
                        #print("Voltage:" + str(self.voltage))
                    elif addr == 226:
                        low_data = s[4 * i + 2]
                        self.current = low_data/100
                        #print("current:" + str(self.current))
                    elif addr == 228:
                        # Output rotate speed
                        #high_data = s[4 * i + 1]
                        low_data = s[4 * i + 2]
                        self.rel_speed = low_data * 6000 / 16384
                        print(str(self.rel_speed))
                        #print("Speed:" + str(self.rel_speed))
                    elif addr == 230:
                        # Not working now
                        None                         
                    elif addr == 231:
                        # Not working now
                        None
                    elif addr == 232:
                        # Position Feedback High 16bits
                        upper_low_data = s[4 * i + 2]
                        self.x_or_y_position_up = upper_low_data * 256
                    elif addr == 233:
                        lower_low_data = s[4 * i + 2]
                        self.x_or_y_position_low = lower_low_data
                        #print("Position:" + str(self.x_or_y_position_up + self.x_or_y_position_low)) 
                        # Position Feedback Low 16bits
                        
            elif len(s) == 2:
                if s[0] == 6:
                    None
                    #print("Speed setting succeeded")
        #return str(self.rel_speed)
        '''
    def send_motor(self):
        if self.run:
            self.set_status()
            self.motor_speed_set()

'''
 Calculate the checksum code by three codes a1, a2, and a3
'''

def check_code(a1, a2):
    buffer = struct.pack(">bh", int(a1), int(a2))

    # python3 code
    # buffer = buffer[0] + buffer[1] + buffer[2]
    # check_num = struct.pack(">l", bytes(buffer)[-1])

    buffer2 = ord(buffer[0]) + ord(buffer[1]) + ord(buffer[2])
    check_num = buffer2 & 0xff
    return check_num

'''
 test car
'''
def test_car(port, speed):
    m = SpeedMotor(port)
    m.motor_start()
    for i in range(speed):
        m.set_speed = i * -1
        m.send_motor()
    m.motor_stop()

if __name__ == '__main__':
    test_car('/dev/ttyUSB0', 10)
    # test_car('/dev/ttyUSB1', 10)
    
