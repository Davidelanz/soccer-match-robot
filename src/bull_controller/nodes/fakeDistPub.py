#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import sys, select, termios, tty

msg = """
Reading from the keyboard 

g - sends a good distance

b - sends a bad distance

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def talker():
    dist_pub = rospy.Publisher("dist", Float32, queue_size=1)
    rospy.init_node('fake_dist')

    # counter = 1
    # dist = 100
        

    
    print(msg) 
    while(1):
        key = getKey()   
        if key == 'g':
            fakeDist = 15
        elif key == 'b':
            fakeDist = 75

        dist_pub.publish(fakeDist)
    # r = rospy.Rate(5) # 5hz
    # while not rospy.is_shutdown():
    # fakeDist = dist - counter
    # dist_pub.publish(fakeDist)
    # counter = counter + 1
    # if counter == 101:
    #     counter = 0

      # r.sleep()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass