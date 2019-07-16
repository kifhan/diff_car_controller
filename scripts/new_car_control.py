#!/usr/bin/env python
# -*- coding:utf-8 -*-

from zlac706 import SpeedMotor
from math import *
import time

import threading
import time
try:
    import _thread
except:
    import thread as _thread
    
mutex = threading.Lock()

def motorThread (motor_obj):
    while True:
        motor_obj.send_motor()
        time.sleep(0.07)

def motorRaedThread (motor1, motor2, motor3, motor4, car):
    while True:
        motor1.read_motor()
        motor2.read_motor()
        motor3.read_motor()
        motor4.read_motor()
        car.set_odom()
        time.sleep(0.1)

class car(object):
    def __init__(self,wheel_diameter,wheel_distance):
        self.diameter = wheel_diameter
        self.distance = wheel_distance
        self.odom = {'x':0,'y':0,'theta':0,'v':0,'w':0}
        self.isRunMode = False
        self.isSending = False
        self.current_time = time.time()
        self.last_time = self.current_time
        self.motor = []
        self.motor.append(SpeedMotor('/dev/ttyUSB0'))
        self.motor.append(SpeedMotor('/dev/ttyUSB1'))
        self.motor.append(SpeedMotor('/dev/ttyUSB2'))
        self.motor.append(SpeedMotor('/dev/ttyUSB3'))
        self.enable()

        try:
            motor1_thread = _thread.start_new(motorThread, (self.motor[0],))
            motor2_thread = _thread.start_new(motorThread, (self.motor[1],))
            motor3_thread = _thread.start_new(motorThread, (self.motor[2],))
            motor4_thread = _thread.start_new(motorThread, (self.motor[3],))
            motor_read_thread = _thread.start_new(motorRaedThread, (self.motor[0],self.motor[1],self.motor[2],self.motor[3],self))
        except:
            print("thread creation failed! program exit!")
            exit(0)
        finally:
            print("motor thread ready!")

    def enable(self):
        self.motor[0].motor_start()
        self.motor[1].motor_start()
        self.motor[2].motor_start()
        self.motor[3].motor_start()
        return True

    def disable(self):
        self.motor[0].motor_stop()
        self.motor[1].motor_stop()
        self.motor[2].motor_stop()
        self.motor[3].motor_stop()
        return True

    # According to the speed of the car, calculate the speed of the wheel
    def set_car_vel(self,v,w):
        w_r, w_l = self.cal_wheel_vel(v,w)
        self.motor[0].set_speed = w_r
        self.motor[1].set_speed = w_l
        self.motor[2].set_speed = w_r
        self.motor[3].set_speed = w_l
        return True
   
    # Calculate wheel speed
    def cal_wheel_vel(self,v,w):
        w_r = 2*v/(2*self.diameter) + w*self.distance/(2*self.diameter)
        w_l = -(2*v/(2*self.diameter) - w*self.distance/(2*self.diameter))
        return [w_r, w_l]
    
    # Get the speed and speed of the car
    def get_car_status(self):
        odom_w_r = self.motor[0].rel_speed
        odom_w_l = self.motor[1].rel_speed
        #odom_w_r = self.motor[2].rel_speed
        #odom_w_l = self.motor[3].rel_speed
        print("odom check", odom_w_r, odom_w_l)
        v = (odom_w_r+odom_w_l)*(self.diameter/2)
        w = (odom_w_r-odom_w_l)*(self.diameter/self.distance)
        return [v,w]

    # Set vehicle odom information
    def set_odom(self):
        self.current_time = time.time()
        dt = self.current_time - self.last_time
        #print("DT!!!!!!!!", dt)
        v,w = self.get_car_status()
        self.odom['x']= self.odom['x'] + v*dt*cos(self.odom['theta'])
        self.odom['y']= self.odom['y'] + v*dt*sin(self.odom['theta'])
        self.odom['theta'] = self.odom['theta'] + w*dt
        self.odom['v'] = v
        self.odom['w'] = w
        #print("set odom", self.odom)

        self.last_time = self.current_time

    # Enter config mode, close bus
    def config_mode(self):
        self.motor[0].motor_stop()
        self.motor[1].motor_stop()
        self.isRunMode = False

    # Enter run_mode, you can speed control
    def run_mode(self):
        self.motor[0].motor_start()
        self.motor[1].motor_start()
        self.motor[2].motor_start()
        self.motor[3].motor_start()
        self.isRunMode = True
        print("diff_car go into the run mode")

    # With new wheel information and car information
    def update_status(self):
        None
        #self.set_odom()
                
def test_set_car_vel(v,w):
    wheel_diameter = 0.06
    wheel_distance = 0.3
    diff_car = car(wheel_diameter,wheel_distance)
    print("HI")
    diff_car.run_mode()
    start = time.time()
    #a = SpeedMotor('/dev/ttyUSB0')
    #b = a.read_motor
    while True:
        diff_car.set_car_vel(v,w)
        #print("WOW", b)
        if (time.time() - start > 80):
            break

def test_car_run_mode():
    wheel_diameter = 0.100
    wheel_distance = 0.100
    diff_car = car(wheel_diameter,wheel_distance)
    diff_car.run_mode()
    
def test_car_config_mode():
    wheel_diameter = 0.100
    wheel_distance = 0.100
    diff_car = car(wheel_diameter,wheel_distance)
    diff_car.config_mode()

if __name__ == '__main__':
    #test_car_config_mode()
    #test_car_run_mode()
    test_set_car_vel(7.5,0)
