#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 22:00:54 2023

@author: john
"""

import cv2
import numpy as np
import pyrealsense2 as rs


# IMPORTS FOR ROS
## UNCOMMENT THIS TO USE ROS
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

# IMPORTS FOR ARUCO
import matplotlib.pyplot as plt





################ HSV SETTING SECTION ################ 


# needed for sliders
def nothing(x):
    pass



def set_color_range(pipeline):
    
    """
    SETS THE HSV with sliders if needed.
    
    The orange of the ball has a high value for the first slider and a low value for the second slider.
    This is because the hsv of orange wraps around the color spectrum. 
    
    Aim for the masking to be white circle with little noise. 
    """

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






################ ARUCO SECTION ################ 


def make_aruco():
    
    """
    Creates ARUCO from the predefined dictionary. Saves to SVG file. 
    """
    
    # # Define the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Generate the marker
    marker_size_pixels = 1000  # High resolution for the marker image
    marker_id = 2
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_pixels)
    
    # # Display the marker using matplotlib
    plt.imshow(marker_image, cmap='gray')
    plt.axis('off')  # Turn off axis numbers and labels
    
    # Save as SVG
    svg_filename = 'aruco_marker_v2.svg'
    plt.savefig(svg_filename, format='svg', bbox_inches='tight', pad_inches=0, dpi=300)
    plt.close()
    
    

# def aruco_to_camera(rvec, tvec, aruco_G):
    
#     """
#     Uses the pose of the aruco and known aruco position to get the world coordinates of the camera.
#     rvec and tvec come from aruco_detection and are OpenCV's way of passing in the aruco pose. 
#     We use rodrigues to convert it to a matrix.
#     The transpose of the aruco pose and the known position of the Aruco are used to find
#     the camera in the world frame. 
#     """
    
#     # Get R and t from G transformation
#     aruco_R = aruco_G[:3, :3]
#     aruco_t = aruco_G[:3, 3]
    
#     # Convert rotation pose vector to rotation matrix. Just opencv stuff.  
#     aruco_R_pose, _ = cv2.Rodrigues(rvec)
    
#     # This converts the pose of the aruco to the camera pose
#     aruco_R_pose_inv = aruco_R_pose.T
#     aruco_pose_t_inv = -np.dot(aruco_R_pose_inv, tvec)
#     # Using this pose and the known aruco R and t, we get camera world position
#     camera_world_position = aruco_R @ aruco_pose_t_inv + aruco_t

#     return camera_world_position, aruco_R_pose_inv


def aruco_to_camera(rvec, tvec, aruco_G):
    
    """
    Uses the pose of the aruco and known aruco position to get the world coordinates of the camera.
    rvec and tvec come from aruco_detection and are OpenCV's way of passing in the aruco pose. 
    We use rodrigues to convert it to a matrix.
    The transpose of the aruco pose and the known position of the Aruco are used to find
    the camera in the world frame. 
    """
    
    # Get R and t from G transformation
    aruco_R = aruco_G[:3, :3]
    aruco_t = aruco_G[:3, 3]
    
    # Convert rotation pose vector to rotation matrix. Just opencv stuff.  
    aruco_R_pose, _ = cv2.Rodrigues(rvec)
    
    # This converts the pose of the aruco to the camera pose
    aruco_R_pose_inv = aruco_R_pose.T
    aruco_pose_t_inv = -np.dot(aruco_R_pose_inv, tvec)
    # # Using this pose and the known aruco R and t, we get camera world position
    # camera_world_position = aruco_R @ aruco_pose_t_inv + aruco_t

    return aruco_R_pose_inv, aruco_pose_t_inv

def detect_aruco(pipeline, align, camera_parameters, aruco_G, marker_length):
    
    """
    This detects the aruco using the camera and returns the cameras position in the world frame. 
    Press 'q' after satisfied with detection. 
    """
    
    camera_world_position = None

    while True:
        # Start streaming
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        width = depth_frame.get_width()
        height = depth_frame.get_height()

        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
        # Define the ArUco dictionary
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters =  cv2.aruco.DetectorParameters()
    
        # Convert RealSense camera parameters to OpenCV camera matrix format
        camera_matrix = np.array([[camera_parameters.fx, 0, camera_parameters.ppx],
                                  [0, camera_parameters.fy, camera_parameters.ppy],
                                  [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((5, 1))  # Assuming no distortion
    
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        # Detect ArUco marker
        corners, ids, rejectedCandidates = detector.detectMarkers(gray_image)
    
        if ids is not None:
            # Estimate pose of the ArUco marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
            
            # Draw a box around the first detected marker
            first_marker_corners = corners[0].reshape(-1, 2)
            cv2.polylines(color_image, [first_marker_corners.astype(np.int32)], True, (0, 255, 0), 2)
            
            # Calculate the position of the marker in the image
            marker_position = np.mean(first_marker_corners, axis=0)
            
            # Get the world position of the camera
            aruco_R_pose_inv, aruco_pose_t_inv = aruco_to_camera(rvecs[0][0], tvecs[0][0], aruco_G)
            
            camera_world_position = aruco_pose_t_inv
    
            # Calculate distances
            distance_to_marker = np.linalg.norm(tvecs[0][0])
            distance_to_robot_base = np.linalg.norm(camera_world_position - aruco_G[:3, 3])
            
            # This is using the realsense depth sensor to compare. 
            depth =  depth_frame.get_distance(marker_position[0],marker_position[1])

            # Text on image setting
            text_color = (0,0,0)
            bg_color = (255,255,255)
            text_pos = (10, color_image.shape[0] - 30)  # Adjust the y-coordinate as needed
            font_scale=0.4
            thickness=1
            
            # Rectangle for behind text
            text = f"Dist to Robot Base: {distance_to_robot_base:.2f}m"
            font = cv2.FONT_HERSHEY_SIMPLEX
            text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
            x, y = text_pos
            bg_start = (x - text_size[1] - 5, y - text_size[1] - 100)
            bg_end = (x + text_size[0] + 65, y + 10)
            cv2.rectangle(color_image, bg_start, bg_end, bg_color, cv2.FILLED)


            # Displays the text on the image
            cv2.putText(color_image, f"Marker Pos Camera: {np.array2string(marker_position, precision=2)}", 
                        (text_pos[0], text_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness)
            cv2.putText(color_image, f"Camera Dist to Marker: {distance_to_marker:.2f}m", 
                        (text_pos[0], text_pos[1]-30), cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness)
            cv2.putText(color_image, f"qDepth Sensor to Marker: {depth:.2f}m", 
                        (text_pos[0], text_pos[1]-60), cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness)
            cv2.putText(color_image, f"Camera World: {np.array2string(camera_world_position, precision=2, separator=', ')}", 
                        (text_pos[0], text_pos[1]-90), cv2.FONT_HERSHEY_SIMPLEX, font_scale - .05, text_color, thickness)
            
            # print("Marker Position in Camera coordinates:", marker_position)
            # print("Camera World Position:", np.array2string(camera_world_position, separator=', '))
            
        # Show the image with the detected marker and box
        cv2.imshow("ArUco", color_image)
        
        # press q once you are satified with detection of camera and aruco.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    return aruco_R_pose_inv, aruco_pose_t_inv
        




################ BALL DETECTION SECTION ################ 



def find_center(contour):
    
    """
    Finds the center of the ball using it's biggest contour'
    """
    
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
        
    return cX, cY


def convert_image_to_world(camera_parameters, coords_image):
    

    x_world = (coords_image['x']- camera_parameters.ppx) * coords_image['z'] / camera_parameters.fx
    y_world= (coords_image['y'] - camera_parameters.ppy) * coords_image['z'] / camera_parameters.fy
    z_world = coords_image['z']

    return x_world, y_world, z_world
    


def detect_ball(h_min, h_max, s_min, s_max, v_min, v_max, pipeline, align, camera_parameters, coords_image, ball_position_pub, aruco_R_pose_inv, aruco_pose_t_inv, aruco_G, draw_type='circle'):
    
    """
    Detects the ball and streams to ROS topic 
    """
    # Tracking if needed
    # tracker = cv2.TrackerCSRT_create()
    # tracking = False
    
    
    # TEXT SETTINGS
    # Rectangle for behind text
    text = "Depth: 0.00m"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = .5
    thickness = 1
    text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
    
    # Set colors
    text_color = (0,0,0)
    circle_color = (255,0,0)
    bg_color = (255,255,255)


    while True:
        
        # Camera frame and alignment
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        
        # get the color image and blur it
        color_image = np.asanyarray(color_frame.get_data())
        # color_image = cv2.GaussianBlur(color_image, (11, 11), 0)

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
                print("rad ball: ", radius)
                
                coords_image['z'] = 0
                if radius != 0:
                    coords_image['z'] = 40/radius
                depth = coords_image['z']
                
                print("Depth in inch: ", depth *.0254)

                coords_image['x'] = min(max(center[0], 0), width - 1)
                coords_image['y'] = min(max(center[1], 0), height - 1)
                # depth =  depth_frame.get_distance(coords_image['x'], coords_image['y'])

                # coords_image['z'] = depth
                
                
                # Draw the circle and center
                cv2.circle(color_image, center, radius, circle_color, 2)
                cv2.circle(color_image, center, 5, circle_color, -1)
                
                # Draw coords_image of circle center over a rectangle
                coord_text = f"({center[0]}, {center[1]})"
                coord_pos =  (center[0] + 10, center[1] - 10)
                x, y = coord_pos
                bg_start = (x, y - text_size[1] - 5)
                bg_end = (x + text_size[0] + 5, y + 5)
                cv2.rectangle(color_image, bg_start, bg_end, bg_color, cv2.FILLED)
                cv2.putText(color_image, coord_text, coord_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
                
                # Draw the depth value on the frame over a rectangle
                depth_text = f"Depth: {depth:.2f}m"
                depth_pos =  (center[0] + 10, center[1] + 30)
                x, y = depth_pos
                bg_start = (x, y - text_size[1] - 5)
                bg_end = (x + text_size[0] + 5, y + 5)
                cv2.rectangle(color_image, bg_start, bg_end, bg_color, cv2.FILLED)
                cv2.putText(color_image, depth_text, depth_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
                

        ## Uncomment this to see the contours.
        #cv2.drawContours(color_image, contours, -1, (0,255,0), 3)


        cv2.imshow("Detected Ball", color_image)
        
        # # output the coords_image here
        # x, y, z = coords_image['x'], coords_image['y'], coords_image['z']
        # print(f"x: {x}, y: {y}, z: {z}")
        
        # output the world_coords
        if not (any([v is None for v in coords_image.values()])):
            # output the coords_image here
            x, y, z = coords_image['x'], coords_image['y'], coords_image['z']
            coord = np.array([x,y,z])
            # print(coord)
            # print(aruco_pose_t_inv)
            # print("aruco_pose_t_inv.T", aruco_pose_t_inv.T)
            # aruco_calc = aruco_pose_t_inv.T @ coord
            # aruco_trans = {'x': None, 'y': None, 'z': None}
    
            # aruco_trans['x'] = aruco_calc[0]
            # aruco_trans['y'] = aruco_calc[0]
            # aruco_trans['z'] = aruco_calc[0]

            # print("aruco_trans: " , aruco_trans)
            # aruco_x_world, aruco_y_world, aruco_z_world = convert_image_to_world(camera_parameters, aruco_trans)
            # print(f'aruco_trans_X: {aruco_x_world}, aruco_trans_Y: {aruco_y_world}, aruco_trans_Z: {aruco_z_world}')
        
        
            # convert ball to camera world
            print(f"x: {x}, y: {y}, z: {z}")
            x_world, y_world, z_world = convert_image_to_world(camera_parameters, coords_image)
            
            # convert camera to aruco
            ball_position_camera = np.array([x_world,y_world, -z_world])
            ball_homogeneous = np.array([*ball_position_camera, 1])
            
            # print("ball_homogeneous:", ball_homogeneous)
            
            # print("aruco_R_pose_inv:", aruco_R_pose_inv)
            # print("aruco_pose_t_inv:", aruco_pose_t_inv)
            
            aruco_R_pose_inv = np.vstack([aruco_R_pose_inv, np.zeros(3)])
            aruco_pose_t_inv = np.hstack([aruco_pose_t_inv.T, 1])
            # print("aruco_R_pose_inv:", aruco_R_pose_inv)
            # print("aruco_pose_t_inv:", aruco_pose_t_inv)
            # print("aruco_R_pose_inv.shape:", aruco_R_pose_inv.shape)
            # print("aruco_pose_t_inv.shape:", aruco_pose_t_inv.shape)
            
            aruco_pose_t_inv_reshaped = aruco_pose_t_inv.reshape(-1, 1)
            aruco_G_pose =  np.hstack([aruco_R_pose_inv, aruco_pose_t_inv_reshaped]) 
            
            ball_position_aruco_homogeneous = aruco_G_pose @ ball_homogeneous

            # print("aruco_G_pose:", aruco_G_pose)
            # aruco_R_pose_inv = np.vstack([aruco_R_pose_inv, np.zeros(3)])
            # ball_position_aruco_homogeneous = np.dot(np.vstack([aruco_R_pose_inv, np.zeros(3)]).T, 
            #                                          np.hstack([aruco_pose_t_inv, 1])) @ ball_homogeneous
            ball_position_aruco = ball_position_aruco_homogeneous[:3]
            
            ball_homogeneous_aruco = np.array([*ball_position_aruco, 1])
            ball_position_world_homogeneous = aruco_G @ ball_homogeneous
            ball_position_world = ball_position_world_homogeneous[:3]

            print("ball_position_world:", ball_position_world)
            
            if not any([v is None for v in coords_image.values()]):

                    point_msg = Point()
                    point_msg.x = ball_position_world[0]
                    point_msg.y = ball_position_world[1]
                    point_msg.z = ball_position_world[2]
                    ball_position_pub.publish(point_msg) 
                        
            
            # z up , x right, y oop
            
            
    
            # coord = np.array([x_world,y_world, z_world])
            # aruco_trans = aruco_pose_t_inv.T @ coord

            # print(f'x world: {x_world}, y world: {y_world}, z world: {z_world}')
            # print(f'X: {x_world}, Y: {y_world}, Z: {z_world}')
            # print(f'aruco_trans_X: {aruco_trans[0]}, aruco_trans_Y: {aruco_trans[1]}, aruco_trans_Z: {aruco_trans[2]}')
        
        
        
        ### UNCOMMENT THIS TO USE ROS 
        # # ROS publish to ball_position
        # if not any([v is None for v in coords_image.values()]):
        #         x_world, y_world, z_world = convert_image_to_world(camera_parameters, coords_image)
        #         point_msg = Point()
        #         point_msg.x = x_world
        #         point_msg.y = y_world
        #         point_msg.z = z_world
        #         ball_position_pub.publish(point_msg) 
                
        # Exit the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    

def main():
    
    """
    
    NOTES:
    
    To run set the aruco_G, marker_length for the aruco after measuring.
    Find with '### CHANGE THIS AFTER YOU MEASURE!'
    
    Uncomment ROS imports, publishers in main and publisher in detect_ball. 
    Find with '### UNCOMMENT THIS TO USE ROS'
    
    Press q after detecting aruco.
    
    Press q to quit ball detection loop.
    
    If your window gets stuck or error, just restart kernel to fix. 
    
    ignore the warnings for now. Just due to window and opencv, will fix later. 
    QObject::moveToThread: Current thread (0x56547c828280) is not the object's thread (0x56547ab43910).
    Cannot move to target thread (0x56547c828280)
    
    """
    
    
    ## Built in web camera 
    # cap = cv2.VideoCapture(1)
    
    ################ REALSENSE CALIBRATION ################ 

    # Start the pipeline and config the frame rate, resolution, etc
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    # Align the image
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    # realsense calibration parameters
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    camera_parameters = depth_profile.get_intrinsics()


    ################ HSV CALIBRATION ################ 

    ## Use sliders to set HSV color range. Use this if color of ball not detected. 
    # h_min, h_max, s_min, s_max, v_min, v_max = set_color_range(pipeline)
    # print(f"{h_min}, {h_max}, {s_min}, {s_max}, {v_min}, {v_max}")
    
    #h_min, h_max, s_min, s_max, v_min, v_max = 158, 10, 147, 255, 91, 255  #orange
    h_min, h_max, s_min, s_max, v_min, v_max = 64, 83, 166, 255, 35, 255 #green
    
    
    
    
    # ################ ARUCO CALIBRATION ################ 
    
    # ## Make new aruco if needed
    # # make_aruco() 
 
    ### CHANGE THIS AFTER YOU MEASURE!
    # Measure the aruco's position and set this G = [[R, t],[0, 0, 0, 1]]  
    # Known aruco_G compared to robot base frame. Change this to known aruco position!
    aruco_G = np.array([[0, 0, 1, 0.13335], 
                        [1, 0, 0, 0], 
                        [0, 1, 0, -0.1143], 
                        [0, 0, 0, 1]])
    # aruco_G = np.array([[1, 0, 0, 0], 
    #                     [0, 1, 0, 0], 
    #                     [0, 0, 1, 0], 
    #                     [0, 0, 0, 1]])
    
    # ### CHANGE THIS AFTER YOU MEASURE!
    # # Set to width of aruco in meters. Measure world aruco after printing on paper. 
    #marker_length= 0.18 #calibrated
    marker_length= 0.137

    # # Get the camera in the world frame
    # camera_world_position, aruco_pose_t_inv = detect_aruco(pipeline, align, camera_parameters, aruco_G, marker_length)
    aruco_R_pose_inv, aruco_pose_t_inv = detect_aruco(pipeline, align, camera_parameters, aruco_G, marker_length)
    # camera_world_position = camera_world_position
    # print("Camera World Position:", np.array2string(camera_world_position, separator=', '))
    
    
    
    # ### UNCOMMENT THIS TO USE ROS 
    # # Publish camera position to ROS topic camera_position
    rospy.init_node('camera_tracking', anonymous=True)
    # camera_position_pub = rospy.Publisher('camera_position', Point, queue_size=10)
    # camera_position_msg = Point()
    # camera_position_msg.x = camera_world_position[0]
    # camera_position_msg.y = camera_world_position[1]
    # camera_position_msg.z = camera_world_position[2]
    # camera_position_pub.publish(camera_position_msg) 

    # ################ BALL DETECTION ################ 
    
    #Ball Detection. Press q to quit windows. If frozen, restart your kernal. 
    
    ## UNCOMMENT THIS TO USE ROS. IN DETECT BALL ALSO NEED TO UNCOMMENT ROS 
    #Set up ROS topic ball_position. This is passed into detect_ball function, because it streams.

    ball_position_pub = rospy.Publisher('ball_position', Point, queue_size=10)
    # ball_position_pub = None

    # # Store the coords_image dictionary.
    coords_image = {'x': None, 'y': None, 'z': None}
    
    
    detect_ball(h_min, h_max, s_min, s_max, v_min, v_max, pipeline, align, camera_parameters, coords_image, ball_position_pub, aruco_R_pose_inv, aruco_pose_t_inv, aruco_G)
    
    

    ############### CLEAN UP CAMERA ################ 

    # cap.release()
    
    pipeline.stop()
    cv2.destroyAllWindows()
    


if __name__ == "__main__":
    main()    ball_position_pub = None


    
    