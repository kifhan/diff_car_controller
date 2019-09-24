#! /usr/bin/env python
# -*- coding: utf-8 -*-

from car_control import Car
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tf
import math

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
    #start = time.time()
    diff_car = arg[0]
    # Processing speed, depending on the release speed of /cmd_vel.
    if diff_car.isRunMode:
        v = msg.linear.x
        w = msg.angular.z
        # print(diff_car.motor[0].rel_speed, diff_car.motor[1].rel_speed)
        if abs(diff_car.motor[0].rel_speed) > 100:
            pub_emergency = rospy.Publisher('/emergency_stop', String, queue_size=10)
            print("emergency: car moving too fast. ", abs(diff_car.motor[0].rel_speed))
            pub_emergency.publish('1')
            # rospy.signal_shutdown("emergency: car moving too fast.")
        diff_car.set_car_vel(v*1.12,w)
    #timepass = start - time.time()
    #print("time pass is:",timepass)

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
    rospy.Subscriber('/manual_vel',Twist,vel_callback,(diff_car,))

    try:
        odom_thread = _thread.start_new(odom_puber, (diff_car.odom, odom_publisher))
    except :
        rospy.loginfo("thread creation failed! program exit!")
        exit(0)
    finally:
        rospy.loginfo("odom thread created！ begin to pub odom info and transform.")

    # Determine if car is in run_mode or config_mode
    while not rospy.is_shutdown():
        if diff_car.isRunMode:
            #mutex.acquire()
            diff_car.update_status()
            #mutex.release()
            #rospy.spinOnce()
        else:
            diff_car.update_status()

    rospy.spin()
