#!/usr/bin/env python3

import cv2
import numpy as np


import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ObjectDetection:
    def __init__(self) -> None:
        rospy.init_node('object_detector', anonymous=True)
        
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
        self.ball_pos_pub = rospy.Publisher('ball_position', Point, queue_size=10)
        self.ball_img_pub = rospy.Publisher('ball_image', Image, queue_size=10)
        self.bridge = CvBridge()
        
    def image_callback(self, image):
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        h_min, h_max, s_min, s_max, v_min, v_max = 64, 83, 166, 255, 35, 255
        
        if h_min > h_max:
            lower_red = np.array([0, s_min, v_min])
            upper_red = np.array([h_max, s_max, v_max])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            
            lower_yellow = np.array([h_min, s_min, v_min])
            upper_yellow = np.array([179, s_max, v_max])
            mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)
            
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        img_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.ball_img_pub.publish(img_msg)
    
    def run(self):
        rospy.spin()
    

if __name__ == "__main__":
    objDetect = ObjectDetection()
    objDetect.run()
    
    