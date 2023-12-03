#!/usr/bin/env python3

import cv2
import numpy as np


import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def nothing(x):
    pass

class ObjectDetection:
    def __init__(self) -> None:
        rospy.init_node('object_detector', anonymous=True)
        
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
        self.ball_pos_pub = rospy.Publisher('ball_position', Point, queue_size=10)
        self.ball_img_pub = rospy.Publisher('ball_image', Image, queue_size=10)
        self.bridge = CvBridge()
        
        self.h_min, self.h_max, self.s_min, self.s_max, self.v_min, self.v_max = 64, 83, 166, 255, 35, 255
        
        self.setup_calibration()
        
    def setup_calibration(self):
        cv2.namedWindow("Original Frame")
        cv2.namedWindow("HSV Mask")
        cv2.namedWindow("Detected Ball")
        
        cv2.createTrackbar('HMin', 'HSV Mask', self.h_min, 179, nothing)
        cv2.createTrackbar('HMax', 'HSV Mask', self.h_max, 179, nothing)
        cv2.createTrackbar('SMin', 'HSV Mask', self.s_min, 255, nothing)
        cv2.createTrackbar('SMax', 'HSV Mask', self.s_max, 255, nothing)
        cv2.createTrackbar('VMin', 'HSV Mask', self.v_min, 255, nothing)
        cv2.createTrackbar('VMax', 'HSV Mask', self.v_max, 255, nothing)
        
    def update_calibration(self):
        self.h_min = cv2.getTrackbarPos('HMin', 'HSV Mask')
        self.h_max = cv2.getTrackbarPos('HMax', 'HSV Mask')
        self.s_min = cv2.getTrackbarPos('SMin', 'HSV Mask')
        self.s_max = cv2.getTrackbarPos('SMax', 'HSV Mask')
        self.v_min = cv2.getTrackbarPos('VMin', 'HSV Mask')
        self.v_max = cv2.getTrackbarPos('VMax', 'HSV Mask')
        
    def image_callback(self, image):
        self.update_calibration()
        
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        
        cv2.imshow("Original Frame", image)
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
        if self.h_min > self.h_max:
            lower_red = np.array([0, self.s_min, self.v_min])
            upper_red = np.array([self.h_max, self.s_max, self.v_max])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            
            lower_yellow = np.array([self.h_min, self.s_min, self.v_min])
            upper_yellow = np.array([179, self.s_max, self.v_max])
            mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower = np.array([self.h_min, self.s_min, self.v_min])
            upper = np.array([self.h_max, self.s_max, self.v_max])
            mask = cv2.inRange(hsv, lower, upper)
            
        cv2.imshow("HSV Mask", mask)
        
        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # img_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        # self.ball_img_pub.publish(img_msg)
    
    def run(self):
        rospy.spin()
    

if __name__ == "__main__":
    objDetect = ObjectDetection()
    objDetect.run()
    
    