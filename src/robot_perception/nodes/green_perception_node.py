#! /usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
from cv2 import cv2

# Image processing
import matplotlib.pyplot as plt
from skimage import io, filters, measure, color, external

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32


# _____________________________________________________________________

# Global variables
VERBOSE=False
DISPLAY=True

# _____________________________________________________________________
# "Callback class"

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        #self.image_pub = rospy.Publisher("/output/image_raw/compressed",
        #    CompressedImage, queue_size=1)
        self.greenX_pub = rospy.Publisher("greenX", Float32, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print("subscribed to /raspicam_node/image/compressed")

        # define color ranges in HSV
        self.lower_red = np.array([160, 140, 50])
        self.upper_red = np.array([180, 255, 255])
        self.lower_blue = np.array([110, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        self.lower_green = np.array([45, 140, 50])
        self.upper_green = np.array([75, 255, 255])

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # Direct conversion to CV2
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        # Get frame dimensions
        frame_height = frame.shape[0]
        frame_width = frame.shape[1]
        frame_channels = frame.shape[2]
        # Image processing
        res = self.green_filtering(frame)
        # Detect centrode
        cX, cY = self.detect_centrode(res)
        # Write the point (cX,xY) on "res" image
        cv2.circle(res, (int(cX),int(cY)), 5, (255, 0, 0), -1)
        # Normalizing w.r.t the center
        cX = int(cX-frame_width/2) 
        cY = int(cY-frame_height/2)
        self.greenX_pub.publish(cX)

        # Print the center of mass coordinates w.r.t the center of image and diplay it
        if VERBOSE:
            print (cX, cY)
            cmd = self.extraction(cX)
            print(cmd)
        # Display the result
        if DISPLAY:
            cv2.imshow('frame', frame)
            cv2.imshow("res_center",res)
            cv2.waitKey(2)


    def green_filtering(self,frame):        
        # Adapted from 
        # https://stackoverflow.com/questions/54425093/
        # /how-can-i-find-the-center-of-the-pattern-and-the-distribution-of-a-color-around)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image to get only desired colors
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        # Smooth and blur "res" image
        res = cv2.medianBlur(res,15)
            # Or also:
            # res = cv2.GaussianBlur(res,(15,15),0)
            # res = cv2.bilateralFilter(res,15,75,75)
        # Erode and convert it for noise reduction
        kernel = np.ones((2,2),np.uint8)
        res = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
        res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        res = cv2.convertScaleAbs(res)
        return res

    def detect_centrode(self,res):        
        # Adapted from 
        # https://stackoverflow.com/questions/54425093/
        # /how-can-i-find-the-center-of-the-pattern-and-the-distribution-of-a-color-around)
        contours, hierarchy = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        areas = []
        centersX = []
        centersY = []
        for cnt in contours:
            areas.append(cv2.contourArea(cnt))
            M = cv2.moments(cnt)
            try:
                centersX.append(int(M["m10"] / M["m00"]))
                centersY.append(int(M["m01"] / M["m00"]))    
            except ZeroDivisionError as error:
                # Output expected ZeroDivisionErrors.
                centersX.append(int(M["m10"]))
                centersY.append(int(M["m01"]))   
                pass    
        full_areas = np.sum(areas)
        acc_X = 0
        acc_Y = 0
        for i in range(len(areas)):
            acc_X += centersX[i] * (areas[i]/full_areas) 
            acc_Y += centersY[i] * (areas[i]/full_areas)
        return acc_X,acc_Y

    
    def extraction(self, cX):
        #  cX  >    0 -> ball on the right
        #  cX  >  100 -> ball on the FAR right
        #  cX  <    0 -> ball on the left
        #  cX  < -100 -> ball on the FAR right
        # -100<cX<100 -> ball ok

        if abs(cX) < 190:
            if cX >  80:
                return 'R'
            elif cX < -100: 
                return 'L'
            else:
                return True    
        else:
            return False 




# _____________________________________________________________________


def main(args):
    '''Initializes and cleanup ros node'''
    print("Starting ROS Image feature detector module")
    ic = image_feature()
    rospy.init_node('green_perception_node')
    r = rospy.Rate(1) # 1hz
    try:
        rospy.spin()
        r.sleep()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main(sys.argv)
