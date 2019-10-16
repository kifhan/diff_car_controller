#! /usr/bin/env python
# -*- coding: utf-8 -*-

from car_control import Car
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import tf
import math
from diff_car_controller.srv import *

import threading
import time
try:
    import _thread
except:
    import thread as _thread

mutex = threading.Lock()

def odom_puber(odom_info,puber):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg.header.seq = msg.header.seq + 1
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = odom_info['x']
        msg.pose.pose.position.y = odom_info['y']
        msg.pose.pose.position.z = 0
        odom_qua = tf.transformations.quaternion_from_euler(0, 0, odom_info['theta'])
        msg.pose.pose.orientation.x = odom_qua[0]
        msg.pose.pose.orientation.y = odom_qua[1]
        msg.pose.pose.orientation.z = odom_qua[2]
        msg.pose.pose.orientation.w = odom_qua[3]
        msg.twist.twist.linear.x = odom_info['vx']
        msg.twist.twist.linear.y = odom_info['vy']
        msg.twist.twist.angular.z = odom_info['w']
        puber.publish(msg)
        if diff_car.publish_tf:
            br.sendTransform((odom_info['x'],odom_info['y'],0),odom_qua,rospy.Time.now(),'base_footprint','odom')
        rate.sleep()

def vel_callback(msg,arg):
    diff_car = arg[0]
    mode = arg[1]
    if diff_car.modeTopic != diff_car.prevmode:
        print "vel: %s mode: %s" % (mode,diff_car.modeTopic)
        diff_car.prevmode = diff_car.modeTopic
    # Processing speed, depending on the release speed of vel topic.
    if diff_car.isRunMode and diff_car.modeTopic == mode:
        v = msg.linear.x
        w = msg.angular.z
        #if abs(diff_car.motor[0].rel_speed) > 100:
        #    pub_emergency = rospy.Publisher('/emergency_stop', String, queue_size=10)
        #    print("emergency: car moving too fast. ", abs(diff_car.motor[0].rel_speed))
        #    pub_emergency.publish('1')
        #    rospy.signal_shutdown("emergency: car moving too fast.")
        diff_car.set_car_vel(v*1.12,w)

def scan_callback(msg, arg):
    diff_car = arg[0]
    if not diff_car.isRunMode:
        return
    printvalue = ' '
    i = 0
    fall_flag = False
    for point in msg.ranges:
        if i > 5 and i <= 180:
            trad = math.radians(45 - 0 - 0.25 * (i-0))
            tdist = 0.23 / math.cos(trad)
            if point > 0.1 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif  i <= 290:
            trad = math.radians(72.5 - 72.5 + 0.25 * (i-181))
            tdist = 0.23 / math.cos(trad)
            if point > 0.1 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif i <= 540:
            trad = math.radians(135 - 72.5 - 0.25 * (i-291))
            tdist = 0.12 / math.cos(trad)
            if point > 0.1 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif  i <= 790:
            trad = math.radians(207.5 - 207.5 + 0.25 * (i-541))
            tdist = 0.12 / math.cos(trad)
            if point > 0.1 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif  i <= 900:
            trad = math.radians(235 - 207.5 - 0.25 * (i-791))
            tdist = 0.23 / math.cos(trad)
            if point > 0.1 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif i <= 1075:
            trad = math.radians(270 - 270 + 0.25 * (i-901))
            tdist = 0.23 / math.cos(trad)
            if point > 0.1 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        i = i + 1
    if fall_flag:
        diff_car.set_car_vel(0,0)
        diff_car.modeTopic = 'config'
        diff_car.config_mode()
        print 'CONFIG MODE! point: ' + printvalue

def mode_callback(msg,arg):
    diff_car = arg[0]
    if msg.data == 'cmd':
        diff_car.modeTopic = 'cmd'
        if not diff_car.isRunMode:
            diff_car.run_mode()
    elif msg.data == 'manual':
        diff_car.modeTopic = 'manual'
        if not diff_car.isRunMode:
            diff_car.run_mode()
    else:
        diff_car.modeTopic = 'config'
        diff_car.config_mode()

if __name__ == '__main__':
    # Initialize node
    rospy.init_node("car_server",anonymous=True)

    # car params
    if not rospy.has_param("~wheel_diameters"):
        rospy.set_param("~wheel_diameter", 0.06)
    if not rospy.has_param("~wheel_distance"):
        rospy.set_param("~wheel_distance", 0.3)
    if not rospy.has_param("~publish_tf"):
        rospy.set_param("~publish_tf", False)
    if not rospy.has_param("~wheel_distance"):
        rospy.set_param("~wheel_distance", 0.3)

    wheel_diameter = rospy.get_param("~wheel_diameter")
    wheel_distance = rospy.get_param("~wheel_distance")
    publish_tf = rospy.get_param("~publish_tf")

    diff_car = Car(wheel_diameter,wheel_distance, publish_tf)
    diff_car.run_mode()

    # Create an odom release thread
    odom_publisher = rospy.Publisher('/odom',Odometry,queue_size=10)

    # Create a subscriber for cmd_vel
    rospy.Subscriber('/manual_vel',Twist,vel_callback,(diff_car,'manual',))
    rospy.Subscriber('/cmd_vel',Twist,vel_callback,(diff_car,'cmd',))

    rospy.Subscriber('/car_mode',String,mode_callback,(diff_car,))
    rospy.Subscriber("/scan", LaserScan, scan_callback,(diff_car,))

    try:
        odom_thread = _thread.start_new(odom_puber, (diff_car.odom, odom_publisher))
    except :
        rospy.loginfo("thread creation failed! program exit!")
        exit(0)
    finally:
        rospy.loginfo("odom thread created！ begin to pub odom info and transform.")

    # Determine if car is in run_mode or config_mode
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if diff_car.isRunMode:
            #mutex.acquire()
            diff_car.update_status()
            #mutex.release()
            #rospy.spinOnce()
        else:
            diff_car.update_status()
        rate.sleep()

    rospy.spin()
