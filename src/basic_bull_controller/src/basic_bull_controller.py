#!/usr/bin/env python

from __future__ import print_function

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('driver_node')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    sign = -1       # used for direction commutation
    try:
        while(1):
            twist = Twist()
            sign = sign * -1
     
            
            # Robot-Ball alignment -> Angular phase (counter-clockwise rotation only)
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 1 # This gain is used to decouple the front and rear wheels, therefore, it has to be set to 1 if not decoupling 
            twist.angular.y = 0; twist.angular.z = 5 #* sign
            pub.publish(twist)
            rospy.sleep(1.5)

            # Ball approach -> get in proximity of the ball ()
            twist.linear.x = 5; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 1; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
            rospy.sleep(1.5)

            # Robot-Ball-Target alignment -> rotation around target (both translation and rotation)
            twist.linear.x = 0; twist.linear.y = 5; twist.linear.z = 0            
            twist.angular.x = 0.33  # This gain is used to decouple the front and rear wheels
            twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
            rospy.sleep(1)


            # Kick ball -> go ahead and hit ball
            twist.linear.x = 5; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 1; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
            rospy.sleep(0.2)

            # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0            
            # pub.publish(twist)
            # rospy.sleep(2)



    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
