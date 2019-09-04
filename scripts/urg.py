import time
import rospy
from sensor_msgs.msg import LaserScan	

def callback(data):
    #rospy.loginfo(data.ranges)
    a = data.ranges
    b = 0
    for points in a:
        if points > 0.25 and points < 0.35:
             b += 1
             print(b, time.time())
    b = 0
        #if points == 0.0010000000474974513:   
    

def listener():
    rospy.init_node('urg', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()   
while True:
    listener()
