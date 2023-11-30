#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def get_user_input():
    while not rospy.is_shutdown():
        try:
            # Ask the user for input in the terminal
            # Ex: Enter ball position (format: x,y,z): 1.0,2.5,3.2
          
            user_input = raw_input("Enter ball position (format: x,y,z): ")

            # Parse user input string and extract x, y, z values
            x, y, z = map(float, user_input.split(','))

            # Create a Point message for the ball position
            ball_position = Point()
            ball_position.x = x
            ball_position.y = y
            ball_position.z = z

            # Publish the ball position
            ball_position_publisher.publish(ball_position)

        except ValueError:
            rospy.logerr("Invalid input format. Please provide x, y, z values separated by commas.")
            continue

if __name__ == '__main__':
    try:
        rospy.init_node('user_input_ball_node', anonymous=True)

        # Create a publisher for the "ball_position" topic
        ball_position_publisher = rospy.Publisher('ball_position', Point, queue_size=10)

        # Get user input and publish ball position
        get_user_input()

    except rospy.ROSInterruptException:
        pass
