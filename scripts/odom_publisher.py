#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Sensor fusion Test
# It is too complecated.

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import tf
import math

import threading
import time
try:
    import _thread
except:
    import thread as _thread

mutex = threading.Lock()
odom_info = {'x':0,'y':0,'theta':0,'vx':0,'vy':0,'w':0}

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
        br.sendTransform((odom_info['x'],odom_info['y'],0),odom_qua,rospy.Time.now(),'base_footprint','odom')
        rate.sleep()

def laser_callback(data):
    distances = data.ranges
    for distance in distances:
        if distance > 0.05 and distance < 0.35:
#            print("stop : laserscan : obstacle is too close.",time.time())
            pub_emergency = rospy.Publisher('/emergency_stop', String, queue_size=10)
            pub_emergency.publish('1')
            break

def pose_callback(msg,arg):
    odom_info = arg[0]
    odom_info['vx'] = msg.x - odom_info['x']
    odom_info['vy'] = msg.y - odom_info['y']
    odom_info['w'] = msg.theta - odom_info['theta']
    odom_info['x'] = msg.x
    odom_info['y'] = msg.y
    odom_info['theta'] = msg.theta

if __name__ == '__main__':
    # Initialize node
    rospy.init_node("car_server",anonymous=True)

    # car params
    if not rospy.has_param("~imu_topic"):
        rospy.set_param("~imu_topic", "/imu/data")
    if not rospy.has_param("~scan_topic"):
        rospy.set_param("~scan_topic", "/scan")
    if not rospy.has_param("~pose_topic"):
        rospy.set_param("~pose_topic", "/pose2D")

    imu_topic = rospy.get_param("~imu_topic")
    scan_topic = rospy.get_param("~scan_topic")
    pose_topic = rospy.get_param("~pose_topic")

    # Create an odom release thread
    odom_publisher = rospy.Publisher('/odom',Odometry,queue_size=10)

    # Create a subscriber
    rospy.Subscriber("/scan",LaserScan,laser_callback)
    rospy.Subscriber("/pose2D",Pose2D,pose_callback,(odom_info,))

    try:
        odom_thread = _thread.start_new(odom_puber, (odom_info, odom_publisher))
    except :
        rospy.loginfo("thread creation failed! program exit!")
        exit(0)
    finally:
        rospy.loginfo("odom thread created！ begin to pub odom info and transform.")

    while not rospy.is_shutdown():
        # if diff_car.isRunMode:
        #     diff_car.update_status()
        # else:
        #     diff_car.update_status()
        None
