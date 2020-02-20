#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32


def talker():
    dist_pub = rospy.Publisher("dist", Float32, queue_size=1)
    rospy.init_node('fake_dist')

    counter = 1
    dist = 100
    
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       fakeDist = dist - counter
       dist_pub.publish(fakeDist)
       counter = counter + 1
       if counter == 101:
           counter = 0

       r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass