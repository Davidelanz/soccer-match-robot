#! /usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Image processing
import matplotlib.pyplot as plt
from skimage import io, filters, measure, color, external

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage


# _____________________________________________________________________

# Global variables
VERBOSE=False

# _____________________________________________________________________
# "Callback class"

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage, queue_size=1)

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
        # Direct conversion to CV2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        frame_height = frame.shape[0]
        frame_width = frame.shape[1]
        frame_channels = frame.shape[2]
        cv2.imshow('frame', frame)
        cv2.waitKey(2)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image to get only desired colors
        mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        # Smooth and blur "res" image
        res = cv2.medianBlur(res,15)
        #res = cv2.GaussianBlur(res,(15,15),0)
        #res = cv2.bilateralFilter(res,15,75,75)

        # Then do the elaboration
        # (https://stackoverflow.com/questions/54425093/how-can-i-find-the-center-of-the-pattern-and-the-distribution-of-a-color-around)
        # 1 - Erode and convert it for noise reduction
        kernel = np.ones((2,2),np.uint8)
        opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
        opening = cv2.cvtColor(opening, cv2.COLOR_BGR2GRAY)
        opening = cv2.convertScaleAbs(opening)
        # 2 - Compute the center of mass
        contours, hierarchy = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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

        # Print the center of mass coordinates w.r.t the center of image and diplay it
        print (int(acc_X-frame_width/2), int(acc_Y-frame_height/2))
        cv2.circle(res, (int(acc_X), int(acc_Y)), 5, (255, 0, 0), -1)
        cv2.imshow("res_center",res)

        #self.subscriber.unregister()


# _____________________________________________________________________


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main(sys.argv)
