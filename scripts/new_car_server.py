#!/usr/bin/env python
# -*-coding:UTF-8 -*-
"""
The car server file is the client class of the car class and performs the following functions:
     - Open bus
     - Pass bus to car class
     - in car's run_mode mode
         * listen for cmd_vel messages and execute
         * odom information with new car : bus Get wheel information car According to wheel information, with new vehicle odom
     - in car's config_mode mode
         * Waiting state, no command is executed

     The wheel is controlled using cmd_vel without action_server; therefore using a simple topic and service mechanism
"""

from new_car_control import car
from zlac706 import SpeedMotor
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tf

import threading
import time
try:
    import _thread
except:
    import thread as _thread
    
mutex = threading.Lock()

def odom_puber(odom_info,puber):
    msg = Odometry()
    msg.header.frame_id = 'odom_link'
    msg.child_frame_id = 'base_link'
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20)
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
        msg.twist.twist.linear.x = odom_info['v']
        msg.twist.twist.angular.z = odom_info['w']
        puber.publish(msg)
        br.sendTransform((odom_info['x'],odom_info['y'],0),odom_qua,rospy.Time.now(),"base_link","odom_link")
        rate.sleep()

def vel_callback(msg,arg):
    start = time.time()
    diff_car = arg[0]
    #print(msg)
    # Processing speed, depending on the release speed of /cmd_vel.
    if diff_car.isRunMode:
        v = msg.linear.x
        #v = 5  #rad/s
        w = msg.angular.z
        #w = 5
        # Use isSending to avoid the main thread calling the update function when the thread sends data, destroying the thread and calling the update function.
        # Since I don’t care, the sending is successful, so I decided here.
        #diff_car.isSending = True
        #if  v !=0 or w !=0:
            #print("let's move the car")
            #mutex.acquire()
        #print("v",v,"w",w)
        diff_car.set_car_vel(v,w)
            #mutex.release()
        #diff_car.isSending = False
    #timepass = start - time.time()
    #print("time pass is:",timepass)
        
 

    # SetMode  未定义srv类型：其内容包括：
    # ---req    int request : run_mode:1   config_mode:0
    # ---Res    Bool   answer :  True       Fasle
def set_mode_callback(req,diffcar):
    if req.request == 'run_mode':
        diffcar.run_mode()
        rospy.loginfo("Enter the run mode!")
        return SetModeResponse(True)
    else:
        diffcar.config_mode()
        rospy.loginfo("Enter configuration mode!")
        return SetModeResponse(True)

def mode_server(args):
    diffcar = args
    s = rospy.Service("set_mode_server",SetMode,set_mode_callback,(diffcar))
    rospy.loginfo("mode server opened!")
    rospy.spin()

if __name__ == '__main__':
    # Initialize node
    rospy.init_node("car_server",anonymous=True)

    # car params
    if not rospy.has_param("~wheel_diameters"):
        rospy.set_param("~wheel_diameter", 0.06)
    if not rospy.has_param("~wheel_distance"):
        rospy.set_param("~wheel_distance", 0.3)

    wheel_diameter = rospy.get_param("~wheel_diameter")
    wheel_distance = rospy.get_param("~wheel_distance")

    diff_car = car(wheel_diameter,wheel_distance)
    diff_car.run_mode()

    # Create an odom release thread
    odom_publisher = rospy.Publisher('/odom',Odometry,queue_size=10)

    # Create a subscriber for cmd_vel
    rospy.Subscriber('/cmd_vel',Twist,vel_callback,(diff_car,))
    
    try:
        odom_thread = _thread.start_new(odom_puber, (diff_car.odom, odom_publisher))
    except :
        rospy.loginfo("thread creation failed! program exit!")
        exit(0)
    finally:
        rospy.loginfo("odom thread created！ begin to pub odom info and transform from odom_link to base_link")

    # mode service Thread
    # Create a service for external program to control the runmode or config mode of the program
    _thread.start_new(mode_server,(diff_car,))

    i = 0
    
    # Determine if car is in run_mode or config_mode
    while not rospy.is_shutdown():
        if diff_car.isRunMode:
            #if i == 0 :
                #rospy.loginfo("car is in run mode!")
                # Avoid reading too frequently (when the buffer has no readable content, there will be problems.)            #rospy.Rate(10)
            # Determine if there is a message that needs to be processed
            #rospy.spinOnce()
            #if not diff_car.isSending:
            #mutex.acquire()
            #i = 1
            diff_car.update_status()  
            #print(diff_car.bus.status)
            #print("update times:",i)
            #i = i + 1
            #mutex.release()
        else:
            # do notiong just update status
            #if i == 1:
                #rospy.loginfo("car is in configure mode!")
            diff_car.update_status()
            #i = 0

    
