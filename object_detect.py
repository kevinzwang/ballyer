#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 22:00:54 2023

@author: john
"""

import cv2
import numpy as np
import pyrealsense2 as rs


import rospy
from geometry_msgs.msg import Point

# for sliders
def nothing(x):
    pass

### SET THE HSV with sliders if needed
def set_color_range(pipeline):

    # Windows
    cv2.namedWindow("Original Frame")
    cv2.namedWindow("HSV Mask")
    cv2.moveWindow("Original Frame", 10, 10)
    cv2.moveWindow("HSV Mask", 660, 10)

    # Trackbars
    cv2.createTrackbar('HMin', 'HSV Mask', 0, 179, nothing)
    cv2.createTrackbar('HMax', 'HSV Mask', 0, 179, nothing)
    cv2.createTrackbar('SMin', 'HSV Mask', 0, 255, nothing)
    cv2.createTrackbar('SMax', 'HSV Mask', 0, 255, nothing)
    cv2.createTrackbar('VMin', 'HSV Mask', 0, 255, nothing)
    cv2.createTrackbar('VMax', 'HSV Mask', 0, 255, nothing)

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        # Convert to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Get trackbar positions
        h_min = cv2.getTrackbarPos('HMin', 'HSV Mask')
        h_max = cv2.getTrackbarPos('HMax', 'HSV Mask')
        s_min = cv2.getTrackbarPos('SMin', 'HSV Mask')
        s_max = cv2.getTrackbarPos('SMax', 'HSV Mask')
        v_min = cv2.getTrackbarPos('VMin', 'HSV Mask')
        v_max = cv2.getTrackbarPos('VMax', 'HSV Mask')
        
        # Display the HSV values on the frame
        text = f"H: {h_min}-{h_max}, S: {s_min}-{s_max}, V: {v_min}-{v_max}"
        position = (10, color_image.shape[0] - 10)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(color_image, text, position, font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("Original Frame", color_image)

        # Calculate the mask
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        
        if h_min > h_max:
            lower_red = np.array([0, s_min, v_min])
            upper_red = np.array([h_max, s_max, v_max])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            
            lower_yellow = np.array([h_min, s_min, v_min])
            upper_yellow = np.array([179, s_max, v_max])
            mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, lower, upper)

        # Show mask
        cv2.imshow("HSV Mask", mask)

        # Exit the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            return h_min, h_max, s_min, s_max, v_min, v_max
        
        
        
### DETECTION SECTION

# finds the center of the ball
def find_center(contour):
    
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
        
    return cX, cY

# convert the image to world coords
def convert_image_to_world(camera_parameters, coords_image):
    x_world = (coords_image['x']- camera_parameters.ppx) * coords_image['z'] / camera_parameters.fx
    y_world= (coords_image['y'] - camera_parameters.ppy) * coords_image['z'] / camera_parameters.fy
    z_world = coords_image['z']

    return x_world, y_world, z_world
    

# detects and tracks the ball, and streams the x, y, z in world coords
def detect_ball(h_min, h_max, s_min, s_max, v_min, v_max, pipeline, align, camera_parameters, coords_image, ball_pub, draw_type='circle'):
    
    # Tracking if needed
    # tracker = cv2.TrackerCSRT_create()
    # tracking = False

    while True:
        
        # Camera frame and alignment
        frames = pipeline.wait_for_frames()
        alligned_frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = alligned_frames.get_color_frame()
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        
        # get the color image and blur it
        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.GaussianBlur(color_image, (11, 11), 0)

        # get the hsv for the color image for mask
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Handle the hue wrap-around case
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
                
        
        # Find the contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Get the biggest contour and use it to find circle around ball
        if contours:
            contour = max(contours, key = cv2.contourArea)
            
            if cv2.contourArea(contour):

                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                coords_image['x'] = min(max(center[0], 0), width - 1)
                coords_image['y'] = min(max(center[1], 0), height - 1)
                depth =  depth_frame.get_distance(coords_image['x'], coords_image['y'])
                coords_image['z'] = depth
                
    
                # Draw the circle and center
                cv2.circle(color_image, center, radius, (255, 0, 0), 2)
                cv2.circle(color_image, center, 5, (0, 255, 255), -1)
                
                # Draw coords_image of center
                coord_text = f"({center[0]}, {center[1]})"
                cv2.putText(color_image, coord_text, (center[0] + 10, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Draw the depth value on the frame
                depth_text = f"Depth: {depth:.2f}m"
                cv2.putText(color_image, depth_text, (center[0] + 10, center[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        
        ## See the contours
        # cv2.drawContours(color_image, contours, -1, (0,255,0), 3)

        cv2.imshow("Detected Ball", color_image)
        
        # output the coords_image here
        # x, y, z = coords_image['x'], coords_image['y'], coords_image['z']
        # print(f"x: {x}, y: {y}, z: {z}")
        
        # output the world_coords
        if not (any(coords_image.values()) is None):
            x_world, y_world, z_world = convert_image_to_world(camera_parameters, coords_image)
            # print(f'x world: {x_world}, y world: {y_world}, z world: {z_world}')
            print(f'X: {x_world}, Y: {y_world}, Z: {z_world}')
        
        
        # ROS publish
        
        if not (any(coords_image.values()) is None):
                x_world, y_world, z_world = convert_image_to_world(camera_parameters, coords_image)
                point_msg = Point()
                point_msg.x = x_world
                point_msg.y = y_world
                point_msg.z = z_world
                ball_pub.publish(point_msg) 
                
        # Exit the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    

def main():
    
    # ROS Stuff
    rospy.init_node('ball_tracker', anonymous=True)
    ball_pub = rospy.Publisher('ball_position', Point, queue_size=10)
    
    
    ## Camera comp
    # cap = cv2.VideoCapture(1)
    
    # realsense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    # realsense calibration parameters
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    camera_parameters = depth_profile.get_intrinsics()


    ## Sliders to set HSV color range
    # h_min, h_max, s_min, s_max, v_min, v_max = set_color_range(pipeline)
    # print(f"{h_min}, {h_max}, {s_min}, {s_max}, {v_min}, {v_max}")
    
    h_min, h_max, s_min, s_max, v_min, v_max = 158, 10, 147, 255, 91, 255

    # Ball Detection. Press q to quit windows. If frozen, restart your kernal. 
    coords_image = {'x': None, 'y': None, 'z': None}
    detect_ball(h_min, h_max, s_min, s_max, v_min, v_max, pipeline, align, camera_parameters, coords_image, ball_pub)
        
    
    # Clean up
    # cap.release()
    pipeline.stop()
    cv2.destroyAllWindows()

    
if __name__ == "__main__":
    main()

    
    