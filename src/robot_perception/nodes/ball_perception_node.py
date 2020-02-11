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
        self.redX_pub = rospy.Publisher("redX", Float32, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print("subscribed to /raspicam_node/image/compressed")

        # define color ranges in HSV
        self.bright_red_lower_bounds = (0, 100, 100)
        self.bright_red_upper_bounds = (10, 255, 255)
        self.dark_red_lower_bounds = (160, 100, 100)
        self.dark_red_upper_bounds = (179, 255, 255)

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
        res = self.red_filtering(frame)
        # Detect centrode
        frame = self.detect_ball(frame,res)
        
        # Display the result
        if DISPLAY:
            cv2.imshow('frame', frame)
            cv2.imshow("res_center",res)
            cv2.waitKey(2)


    def red_filtering(self,frame):        
        # Adapted from 
        # https://stackoverflow.com/questions/42840526/opencv-python-red-ball-detection-and-tracking
        #
        # convert the input stream into HSV color space
        hsv_conv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # then the image is blurred
        hsv_blurred_img = cv2.medianBlur(hsv_conv_img,9,3)
        # because hue wraps up and to extract as many "red objects" as possible,
        # we define lower and upper boundaries for brighter and for darker red shades
        bright_red_mask = cv2.inRange(hsv_blurred_img,self.bright_red_lower_bounds,self.bright_red_upper_bounds)
        dark_red_mask   = cv2.inRange(hsv_blurred_img,self.dark_red_lower_bounds,  self.dark_red_upper_bounds)
        # after masking the red shades out, I add the two images 
        weighted_mask = cv2.addWeighted(bright_red_mask, 1.0, dark_red_mask, 1.0, 0.0)
        # then the result is blurred
        blurred_mask = cv2.GaussianBlur(weighted_mask,(9,9),3,3)
        # some morphological operations (closing) to remove small blobs 
        erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
        eroded_mask = cv2.erode(blurred_mask,erode_element)
        dilated_mask = cv2.dilate(eroded_mask,dilate_element)
        return dilated_mask 

    def detect_ball(self,frame,res):        
        # Adapted from 
        # https://stackoverflow.com/questions/42840526/opencv-python-red-ball-detection-and-tracking
        #
        # on the color-masked, blurred and morphed image I apply the cv2.HoughCircles-method to detect circle-shaped objects 
        detected_circles = cv2.HoughCircles(res, cv2.HOUGH_GRADIENT, 1, 150, param1=100, param2=20, minRadius=20, maxRadius=200)
        if detected_circles is not None:
            for circle in detected_circles[0, :]:
                circled_orig = cv2.circle(frame, (circle[0], circle[1]), circle[2], (0,255,0),thickness=3)
            return circled_orig
        else:
            return frame


    
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
    rospy.init_node('perception_node')
    ic = image_feature()
    r = rospy.Rate(1) # 1hz
    try:
        rospy.spin()
        r.sleep()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main(sys.argv)
