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
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.get_arduino_message()

    # get cmd_vel message, and get linear velocity and angular velocity
    def get_cmd_vel(self, data):
        x = data.linear.x
        y = data.linear.y
        angular = data.angular.z
        rotation = data.angular.x  # used for front/rear decoupling  
        self.send_cmd_to_arduino(x, y, angular, rotation)

    def get_sign(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        elif x == 0:
            return 0
        else:
            return x

    # Translate x, y and angular velocity to PWM signal of each wheels, and send to arduino
    def send_cmd_to_arduino(self, x, y, angular, rotation):
        # Calculate right and left wheels' signal
        # if (angular = 0):
        front_left = int((x + y)  + angular)*75
        front_right = int((x - y) - angular)*75
        # rotation is an additional gain to decouple front and rear wheels to rotate
        rear_left = int(((x - y)   + angular)*75 * rotation)  
        rear_right = int(((x + y)  - angular)*75 * rotation)
        # elif( angular > 0):
        #     front_left = int((x + y)*75)
        #     front_right = int((x - y)*75)
        #     rear_left = int((x - y)*75)
        #     rear_right = int((x + y )*75)
        # else:
        #     front_left = int((x + y)*75)
        #     front_right = int((x - y)*75)
        #     rear_left = int((x - y)*75)
        #     rear_right = int((x + y )*75)
        
    	# Saturate velocities if over 255
        if abs(front_left) > 255:
            front_left =  (self.get_sign(front_left)) * 255
        if abs(front_right) > 255:
            front_right = self.get_sign(front_right) * 255
        if abs(rear_left) > 255:
            rear_left = self.get_sign(rear_left) * 255
        if abs(rear_right) > 255:
            rear_right = self.get_sign(rear_right) * 255

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


