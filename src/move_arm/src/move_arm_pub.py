#!/usr/bin/env python

#Example publish rostopic pub /user_input std_msgs/String "1.0,2.0,3.0"

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('user_pose_input', anonymous=True)

    # Create a publisher for the "arm_goal" topic
    arm_goal_publisher = rospy.Publisher('arm_goal', PoseStamped, queue_size=10)

    while True:
        pose_str = input('Position (y z): ')
        try:
            y, z = map(float, pose_str.split())
        except:
            print('Quitting...')
            break

        arm_goal = PoseStamped()
        arm_goal.header = Header(stamp=rospy.Time.now(), frame_id='base')
        arm_goal.pose.position.x = 0.6
        arm_goal.pose.position.y = y
        arm_goal.pose.position.z = z
        arm_goal.pose.orientation.x = 0.0
        arm_goal.pose.orientation.y = 2 ** 0.5 / 2
        arm_goal.pose.orientation.z = 0.0
        arm_goal.pose.orientation.w = 2 ** 0.5 / 2

        arm_goal_publisher.publish(arm_goal)
        print("Position published!")
