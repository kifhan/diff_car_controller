#!/usr/bin/env python
# -*- coding:utf-8 -*-

from zlac706 import SpeedMotor
from math import *
import time
import rospy

import threading
import time
try:
    import _thread
except:
    import thread as _thread
    
mutex = threading.Lock()

def moterThread (motor_obj):
    while True:
        motor_obj.send_motor()
        time.sleep(0.5)

# def moterRaedThread (motor1, motor2):
#     while True:
#         motor1.read_motor()
#         motor2.read_motor()
#         time.sleep(0.1)

class car(object):
    def __init__(self,wheel_diameter,wheel_distance):
        self.diameter = wheel_diameter
        self.distance = wheel_distance
        self.odom = {'x':0,'y':0,'th':0,'vx':0,'vy':0,'vth':0}
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.isRunMode = False
        self.isSending = False
        self.motor = []
        self.motor.append(SpeedMotor('/dev/ttyUSB0'))
        self.motor.append(SpeedMotor('/dev/ttyUSB1'))
        # self.motor[2] = SpeedMotor('/dev/ttyUSB2')
        # self.motor[3] = SpeedMotor('/dev/ttyUSB3')
        self.enable()

        try:
            motor1_thread = _thread.start_new(moterThread, (self.motor[0],))
            motor2_thread = _thread.start_new(moterThread, (self.motor[1],))
            # motor_read_thread = _thread.start_new(moterRaedThread, (self.motor[0],self.motor[1]))
        except:
            print("thread creation failed! program exit!")
            exit(0)
        finally:
            print("motor thread ready!")

    def enable(self):
        self.motor[0].motor_start()
        self.motor[1].motor_start()
        return True

    def disable(self):
        self.motor[0].motor_stop()
        self.motor[1].motor_stop()
        return True

    # According to the speed of the car, calculate the speed of the wheel
    def set_car_vel(self,v,w):
        w1, w2 = self.cal_wheel_vel(v,w)
        self.motor[0].set_speed = w1
        self.motor[1].set_speed = w2
        return True

    # Calculate wheel speed
    def cal_wheel_vel(self,v,w):
        w1 = 2*v/self.diameter - w*self.distance/self.diameter
        w2 = -(2*v/self.diameter + w*self.distance/self.diameter)
        return [w1, w2]
    
    # Get the speed and speed of the car
    def get_car_status(self):
        self.motor[0].read_motor()
        self.motor[1].read_motor()
        w1 = self.motor[0].rel_speed
        w2 = self.motor[1].rel_speed
        #print("odom check", w1, w2)
        vx = ((w1 + w2) / 2)
        vy = 0
        vth = ((w1 - w2)/self.distance)
        return [vx,vy,vth]
    
    # Enter config mode, close bus
    def config_mode(self):
        self.motor[0].motor_stop()
        self.motor[1].motor_stop()
        self.isRunMode = False

    # Enter run_mode, you can speed control
    def run_mode(self):
        self.motor[0].motor_start()
        self.motor[1].motor_start()
        self.isRunMode = True
        print("diff_car go into the run mode")

    # With new wheel information and car information
    def update_status(self):
        self.motor[0].read_motor()
        self.motor[1].read_motor()
                
def test_set_car_vel(v,w):
    wheel_diameter = 0.100
    wheel_distance = 0.100
    diff_car = car(wheel_diameter,wheel_distance)
    diff_car.run_mode()
    start = time.time()
    while True:
        diff_car.set_car_vel(v,w)
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
    test_set_car_vel(15,15)
