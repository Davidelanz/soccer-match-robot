#! /usr/bin/env python

import rospy

# import ros messages
# from geometry_msgs.msg import Point, Twist
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from tf import transformations

# import ros service
# from std_srvs.srv import *

# other imports
import cv2
import numpy as np

# _____________________________________________________________________

# Global variables
# position_ = Point()
# yaw_ = 0
# state_ = 0
# state_desc_ = ['Go to node', 'Obstacle avoidance']
# regions_ = {
#    'right': 10,
#    'fright': 10,
#    'front': 10,
#    'fleft': 10,
#    'left': 10,
# }

# Services
# srv_client_go_to_point_ = None

# _____________________________________________________________________
# Callbacks

# def clbk_laser(msg):
#    global regions_
#    regions_ = {
#        'right':  min(min(msg.ranges[0:143]), 10),
#        'fright': min(min(msg.ranges[144:287]), 10),
#        'front':  min(min(msg.ranges[288:431]), 10),
#        'fleft':  min(min(msg.ranges[432:575]), 10),
#        'left':   min(min(msg.ranges[576:713]), 10),
#    }

# def clbk_odom(msg):
#    global position_, yaw_

# position
#    position_ = msg.pose.pose.position

# yaw
#    quaternion = (
#        msg.pose.pose.orientation.x,
#        msg.pose.pose.orientation.y,
#        msg.pose.pose.orientation.z,
#        msg.pose.pose.orientation.w)
#    euler = transformations.euler_from_quaternion(quaternion)
#    yaw_ = euler[2]


# _____________________________________________________________________
# Functions

# def change_state(state):
#    global state_, state_desc_, robot_ID
#    global srv_client_go_to_point_
#    state_ = state
#    rospy.loginfo("Controller node - Robot %s - State [%s] %s", robot_ID, state, state_desc_[state])
#    if state_ == 0:
#        resp = srv_client_go_to_point_(True)
# resp = srv_client_wall_follower_(False)
#    if state_ == 1:
#        resp = srv_client_go_to_point_(False)
# resp = srv_client_wall_follower_(True)


# _____________________________________________________________________

def main():
    #    global regions_, position_, state_, yaw_, robot_ID
    #    global srv_client_go_to_point_

    rospy.init_node('perception_node')
    rospy.loginfo("Robot perveption node started")

    # Get params
#    robot_ID = rospy.get_param("robot_ID")

    # Subs
#    sub_laser = rospy.Subscriber('front_laser/scan', LaserScan, clbk_laser)
#    sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)

    # Client services
#    srv_client_go_to_point_ = rospy.ServiceProxy('go_to_point_switch', SetBool)

    # Pubs
#    obst_pub = rospy.Publisher('/obstacle_node', Point, queue_size=10)
#    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Initialize
#    rospy.loginfo("Controller node - Waiting for service")
#    rospy.wait_for_service('go_to_point_switch')
#    change_state(0)
#    rospy.loginfo("Controller node - Waiting for service")
#    rospy.wait_for_service('go_to_point_switch')
#    change_state(0)
    # initialize the videocapture
    cap = cv2.VideoCapture(0)
    # define color ranges in HSV
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])
    lower_red = np.array([160, 140, 50])
    upper_red = np.array([180, 255, 255])
    lower_green = np.array([45, 140, 50])
    upper_green = np.array([75, 255, 255])
    
    
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # Take each frame
        _, frame = cap.read()
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image to get only desired colors
        mask = cv2.inRange(hsv, lower_red, upper_red)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        # Show the result
        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        cv2.imshow('res', res)
        # k = cv2.waitKey(5) & 0xFF
        # if k == 27:
        #    break
        rate.sleep()
    
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
