#!/usr/bin/env python
# from __future__ import print_function
# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy, sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

ballX = 0

def callback(data):
    global ballX
    ballX = data.data 
    rospy.loginfo("Callback data is %f", data.data)
    
def planner(pub, xMinCenter, xMaxCenter, xMinOutlier, xMaxOutlier):
    if ballX > xMinCenter and ballX < xMaxCenter:
        print("We're in the middle area -> go ahead")
        goAhead(pub)
    elif ballX > xMinOutlier and ballX < xMinCenter:
        print("The ball is on the left of the image -> go left")
        leftAlign(pub)
    elif ballX < xMaxOutlier and ballX > xMaxCenter:
        print("The ball is on the right of the image -> go right")
        rightAlign(pub)
    elif ballX < xMinOutlier or ballX > xMaxOutlier:
        print("The ball is in an outlying area -> spin to search ball")
        rotateLeft(pub)
    else:
        print("Some error in areas computation occured")
        pass


def goAhead(pub):
    twist = Twist()
    twist.linear.x = 5; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 1; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

def leftAlign(pub):
    twist = Twist()
    twist.linear.x = 5; twist.linear.y = 5; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = 0  # counter-clock wise rotation
    pub.publish(twist)

def rightAlign(pub):
    twist = Twist()
    twist.linear.x = 5; twist.linear.y = -5; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = 0 # clock wise roation
    pub.publish(twist)

def rotateLeft(pub):
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = 5  # counter-clock wise rotation
    pub.publish(twist)

def rotateRight(pub):
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = -5  # clock wise rotation
    pub.publish(twist)

# _____________________________________________________________________

def main(args):
    '''Initializes and cleanup ros node'''
    print("Starting ROS bull controller node")
    rospy.init_node('controller')
    
    sub =  rospy.Subscriber('redX', Float32, callback)
    print("Subscribed to redX")

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    print("Publisher on cmd_vel")
    
    width = rospy.get_param('width', 410)
    # Compute action areas based on rotateRightwidth camera parameter
    xMin =  width * -0.5
    xMax = width * 0.5
    print("Camera width = " + str(width) 
        + "  ,  xMin = " + str(xMin) 
        + "  ,  xMax = " + str(xMax) )
    
    # Outlier areas, to be discarded
    xMinOutlier = xMin + 0.05*width
    xMaxOutlier = xMax - 0.05*width
    # Center area, when ball is ahead
    xMinCenter = -0.2*width
    xMaxCenter = 0.2*width

    r = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        planner(pub, xMinCenter, xMaxCenter, xMinOutlier, xMaxOutlier)
        #rospy.spin()
        r.sleep()

    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)


if __name__ == "__main__":
    main(sys.argv)
