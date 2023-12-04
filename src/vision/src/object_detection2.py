#!/usr/bin/env python3

import cv2
import numpy as np


import rospy
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
import tf2_geometry_msgs
import tf2_ros

def nothing(x):
    pass

class ObjectDetection:
    def __init__(self) -> None:
        rospy.init_node('object_detector', anonymous=True)
        
        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.image_callback)
        rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.camera_info_callback)

        self.ball_pos_pub = rospy.Publisher('ball_position', PointStamped, queue_size=10)
        self.ball_img_pub = rospy.Publisher('ball_image', Image, queue_size=10)
        self.bridge = CvBridge()
        
        self.h_min, self.h_max, self.s_min, self.s_max, self.v_min, self.v_max = 38, 57, 58, 214, 44, 188
        
        self.calibrate = True
        self.trackbars_created = False

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.tf_buffer = tf2_ros.Buffer()                    # base_point = self.tf_listener.transformPoint("/base", camera_point)

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def camera_info_callback(self, msg):
        self.fx = msg.P[0]
        self.fy = msg.P[5]
        self.cx = msg.P[2]
        self.cy = msg.P[6]

    def pixel_to_point(self, u, v, depth):
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z
                
    def update_calibration(self, image, mask, ball_image):
        cv2.imshow("Original Frame", image)
        cv2.imshow("HSV Mask", mask)
        cv2.imshow("Tracked Ball", ball_image)

        if not self.trackbars_created:
            cv2.namedWindow("Original Frame")
            cv2.namedWindow("HSV Mask")

            cv2.createTrackbar('HMin', 'HSV Mask', self.h_min, 179, nothing)
            cv2.createTrackbar('HMax', 'HSV Mask', self.h_max, 179, nothing)
            cv2.createTrackbar('SMin', 'HSV Mask', self.s_min, 255, nothing)
            cv2.createTrackbar('SMax', 'HSV Mask', self.s_max, 255, nothing)
            cv2.createTrackbar('VMin', 'HSV Mask', self.v_min, 255, nothing)
            cv2.createTrackbar('VMax', 'HSV Mask', self.v_max, 255, nothing)
            self.trackbars_created = True

        self.h_min = cv2.getTrackbarPos('HMin', 'HSV Mask')
        self.h_max = cv2.getTrackbarPos('HMax', 'HSV Mask')
        self.s_min = cv2.getTrackbarPos('SMin', 'HSV Mask')
        self.s_max = cv2.getTrackbarPos('SMax', 'HSV Mask')
        self.v_min = cv2.getTrackbarPos('VMin', 'HSV Mask')
        self.v_max = cv2.getTrackbarPos('VMax', 'HSV Mask')

        cv2.waitKey(10)
        
    def image_callback(self, image):        
        image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
                
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
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if self.calibrate:
            ball_image = image.copy()


        if contours:
            contour = max(contours, key = cv2.contourArea)
            
            if cv2.contourArea(contour):
                (u, v), radius = cv2.minEnclosingCircle(contour)

                if radius > 10:

                    depth = 80 / radius
                    
                    # Draw the circle and center
                    if self.calibrate:
                        print("rad ball: ", radius)
                        center = (int(u), int(v))
                        cv2.circle(ball_image, center, int(radius), (255,0,0), 2)
                        cv2.circle(ball_image, center, 5, (255,0,0), -1)
                    
                    if self.fx is not None:
                        x, y, z = self.pixel_to_point(u, v, depth)

                        try:
                            trans = self.tf_buffer.lookup_transform("base", "usb_cam", rospy.Time())

                            camera_point = PointStamped()
                            camera_point.point.x = x
                            camera_point.point.y = y
                            camera_point.point.z = z

                            base_point = tf2_geometry_msgs.do_transform_point(camera_point, trans)

                            self.ball_pos_pub.publish(base_point)
                        except Exception as e:
                            # Writes an error message to the ROS log but does not raise an exception
                            rospy.logerr("Could not extract pose from TF.")
                            rospy.logerr(e)
                            return
                        
        if self.calibrate:
            self.update_calibration(image, mask, ball_image)
            
    def run(self):
        rospy.spin()
    

if __name__ == "__main__":
    objDetect = ObjectDetection()
    objDetect.run()
    
    