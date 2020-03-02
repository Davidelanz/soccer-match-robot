#!/usr/bin/env python

# This is the controller able to detect the ball, reach a certain distance from it and align itself and the ball with 
# the target
import rospy, sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


ballX = 0       # Ball
blueX = 0       # Goal
dist = 999      # Distance
area = 1
# sign = -1


def callbackBall(data):
    global ballX
    ballX = data.data 
    # rospy.loginfo("Ball callback data is %f", data.data)

def callbackGoal(data):
    global blueX
    blueX = data.data 
    # rospy.loginfo("Goal callback data is %f", data.data)

# def callbackDist(data):
#     global dist
#     dist = data.data 
#     # rospy.loginfo("Distance callback data is %f", data.data)

def callbackArea(data):
    global area
    area = data.data 
    # rospy.loginfo("Distance callback data is %f", data.data)
    
def planner(pub, xMinCenter, xMaxCenter, xMinOutlier, xMaxOutlier, xMinGoal, xMaxGoal, sign, xRightGoal, xLeftGoal):
    if area > 50000:
        #goBack(pub)
        distOk = False
    elif area < 50001 and area > 25000:
        distOk = True
    else:
        distOk = False

    if not distOk: # We're still far from the ball -> Get closer
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
            rotateRight(pub)
        else:
            print("Some error in areas computation occured")
            pass
    else:   # We're close to the ball -> rotate around it
        # if blueX > xMinCenter and blueX < xMaxCenter:
        #     print("We see the goal in the middle -> go ahead")
        #     goAhead(pub)
        # elif blueX > xMinOutlier and blueX < xMinCenter:
        #     print("We see the goal in the left -> go left")
        #     leftAlign(pub)
        # elif blueX < xMaxOutlier and blueX > xMaxCenter:
        #     print("We see the goal in the right -> go right")
        #     rightAlign(pub)
        # elif blueX < xMinOutlier or blueX > xMaxOutlier:
        #     print("We don't see the goal -> rotating around")
        #     rotateAround(pub)
        # else:
        #     print("Some error in areas computation occured")
        #     pass
        if blueX > xMinGoal  and blueX < xMaxGoal:
            print("We see the goal in the middle -> go ahead")
            goAhead(pub)
        elif blueX > xMaxGoal and blueX < xRightGoal :
            print("We see the goal on the right -> go right")
            rightAlign(pub)
        elif blueX < xMinGoal and blueX > xLeftGoal:
            print("We see the goal on the left -> go left")
            leftAlign(pub)
        else:
            rotateAround(pub,sign)
            print("We don't see the goal -> rotating around")
        

def rotateAround(pub, sign):
    # global sign
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = -5 * sign; twist.linear.z = 0  
    # sign = sign * -1          
    twist.angular.x = 0.375  # This gain is used to decouple the front and rear wheels
    twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

def goAhead(pub):
    twist = Twist()
    twist.linear.x = 5; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 1; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

def goBack(pub):
    twist = Twist()
    twist.linear.x = -5; twist.linear.y = 0; twist.linear.z = 0
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
    twist.angular.y = 0; twist.angular.z = 1  # counter-clock wise rotation
    pub.publish(twist)

def rotateRight(pub):
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
    twist.angular.y = 0; twist.angular.z = -1  # clock wise rotation
    pub.publish(twist)

# _____________________________________________________________________

def main(args):
    '''Initializes and cleanup ros node'''
    print("Starting ROS bull controller node")
    rospy.init_node('controller')
 
    # Subscribe to topic for detecting red (the ball)
    subBall =  rospy.Subscriber('redX', Float32, callbackBall)
    print("Subscribed to redX")

    # Subscribe to topic for detecting green (the goal)
    subGoal =  rospy.Subscriber('blueX', Float32, callbackGoal)
    print("Subscribed to blueX")

    #  # Subscribe to topic for getting distance 
    # subDist = rospy.Subscriber('dist', Float32, callbackDist)
    # print("Subscribed to dist")

     # Subscribe to topic for getting distance 
    subDist = rospy.Subscriber('areaX', Float32, callbackArea)
    print("Subscribed to area")

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
    
    # Centre detection area for goal
    xMinGoal = xMin + 0.3*width 
    xMaxGoal = xMax - 0.3*width
    # Right and left detection area for goal
    xRightGoal = xMaxGoal + 0.2*width 
    xLeftGoal  = xMinGoal - 0.2*width
    
    sign = 1
    counter = 0
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        planner(pub, xMinCenter, xMaxCenter, xMinOutlier, xMaxOutlier, xMinGoal, xMaxGoal, sign, xRightGoal, xLeftGoal)
        counter = counter + 1
        if counter == 100:
            sign = sign * -1
            counter = 0
        #rospy.spin()
        r.sleep()

    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)


if __name__ == "__main__":
    main(sys.argv)
