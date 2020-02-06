#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial


class driver:
    def __init__(self):
        # init ros
        rospy.init_node('car_driver', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.get_cmd_vel)
#        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
	self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.get_arduino_message()

    # get cmd_vel message, and get linear velocity and angular velocity
    def get_cmd_vel(self, data):
        x = data.linear.x
	y = data.linear.y
        angular = data.angular.z
        self.send_cmd_to_arduino(x, y, angular)

    # translate x, and angular velocity to PWM signal of each wheels, and send to arduino
    def send_cmd_to_arduino(self, x, y, angular):
        # calculate right and left wheels' signal

	front_left = int((x + y)*75)
	front_right = int((x - y)*75)

	rear_left = int((x - y)*75)
	rear_right = int((x + y )*75)
	
 #      right = int((x) * 50 + (y) * 50 )
 #      left = int((x) * 50 + (y) * 50)
        # format for arduino
#        message = "{},{},{},{},{},{}*".format(right, left, right, left, right, left)
	message = "{},{},{},{}*".format(front_left, front_right, rear_left, rear_right)
        print message
        # send by serial 
        self.ser.write(message)

    # receive serial text from arduino and publish it to '/arduino' message
    def get_arduino_message(self):
        pub = rospy.Publisher('arduino', String, queue_size=10)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            message = self.ser.readline()
            pub.publish(message)
            r.sleep()

if __name__ == '__main__':
    try:
        d = driver()
    except rospy.ROSInterruptException: 
        pass
