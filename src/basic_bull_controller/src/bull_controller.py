#!/usr/bin/env python

# from __future__ import print_function

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import String

from geometry_msgs.msg import Twist

from std_msgs.msg import Float32

import sys, select, termios, tty

def init():
    global pub, sub, xMinOutlier, xMaxOutlier, xMinCenter, xMaxCenter

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    
    sub =  rospy.Subscriber("ballX", Float32, callback)

    rospy.init_node('controller')
    
    width = rospy.search_param('width')
    width = rospy.get_param(width, 410)

    # Compute action areas based on width camera parameter
    xMin = int(- width/2)
    xMax = int(width/2)

    # Outlier areas, to be discarded
    xMinOutlier = xMin + 0.05*width
    xMaxOutlier = xMax - 0.05*width

    # Center area, when ball is ahead
    xMinCenter = -0.2*width
    xMaxCenter = 0.2*width

    
def goAhead():
    twist = Twist()

    twist.linear.x = 5; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 1; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

def leftAlign():
    twist = Twist()

    twist.linear.x = 5; twist.linear.y = -5; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = 0  # counter-clock wise rotation
    pub.publish(twist)

def rightAlign():
    twist = Twist()

    twist.linear.x = 5; twist.linear.y = 5; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = 0 # clock wise roation
    pub.publish(twist)

def rotateLeft():
    twist = Twist()

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = 5  # counter-clock wise rotation
    pub.publish(twist)

def rotateRight():
    twist = Twist()

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = -5  # clock wise rotation
    pub.publish(twist)


def callback(data):
    global ballX
    ballX = data.data 
    rospy.loginfo("Data is %f", data.data)
    
def planner():
    global pub, sub, xMinOutlier, xMaxOutlier, xMinCenter, xMaxCenter

    global ballX

    if ballX < xMinOutlier | ballX > xMaxOutlier:
        if ballX > 0:
            rotateRight()
        else:
            rotateLeft()
        # The ball is in an outlying area -> spin to search ball
    elif ballX > xMinCenter & ballX < xMaxCenter:
        # We're in the middle area -> go ahead
        goAhead()
    elif ballX > xMinOutlier & ballX < xMinCenter:
        # The ball is on the left of the image -> go left
        leftAlign()
    elif ballX < xMaxOutlier & ballX > xMaxCenter:
        # The ball is on the right of the image -> go right
        rightAlign()
    else:
        print("Some error in areas computation occured")
        pass

        

    
    
    
    



# _____________________________________________________________________


def main(args):
    '''Initializes and cleanup ros node'''
    print("Starting ROS bull controller node")
    
    global pub

    global twist
    
    

    try:
        init()
        r = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            planner()
            rospy.spin()
            r.sleep()
    except KeyboardInterrupt:
        print("Shutting down ROS bull controller node")

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    

    
    

if __name__ == "__main__":
    main(sys.argv)