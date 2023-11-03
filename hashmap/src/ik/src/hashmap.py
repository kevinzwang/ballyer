#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

def ik_service_client():
    service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
    ik_service_proxy = rospy.ServiceProxy(service_name, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    header = Header(stamp=rospy.Time.now(), frame_id='base')

    # Create a PoseStamped and specify header (specifying a header is very important!)
    pose_stamped = PoseStamped()
    pose_stamped.header = header

    # x,y, z positions
    x = 0.4
    #ys = np.linspace(-.7, .3, 4)  
    ys = np.array([-0.7, -0.65, -0.6, -0.55, -0.5, -0.45, -0.4, -0.35, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3])
    #zs = np.linspace(0, .7, 2)
    zs = np.array([0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7])


    # Set end effector quaternion: YOUR CODE HERE
    x_o, y_o, z_o, w_o = (0, -1, 0, 0)

    t_start = rospy.get_time()

    main_dict = {}
    for y in ys:
        y_dict = {}
        for z in zs:
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation.x = x_o
            pose_stamped.pose.orientation.y = y_o
            pose_stamped.pose.orientation.z = z_o
            pose_stamped.pose.orientation.w = w_o


            # Add desired pose for inverse kinematics
            ik_request.pose_stamp.append(pose_stamped)
            # Request inverse kinematics from base to "right_hand" link
            ik_request.tip_names.append('right_hand')

            try:
                rospy.wait_for_service(service_name, 5.0)
                response = ik_service_proxy(ik_request)
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Service call failed: %s" % (e,))

            # Check if result valid, and type of seed ultimately used to get solution
            if (response.result_type[0] > 0):
                # Format solution into Limb API-compatible dictionary
                print("")
                rospy.loginfo("x: " + str(x) + ", y: " + str(y) + ", z: " + str(z))
                rospy.loginfo("Response:\n%s", response.joints[0].position)
                y_dict[z] = response.joints[0].position
            else:
                rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
                rospy.logerr("Result Error %d", response.result_type[0])

        main_dict[y] = y_dict
    t_end = rospy.get_time()

    print(t_end - t_start)

    with open('hashmap.txt', 'w') as f:
         f.write(str(main_dict))



def main():
    rospy.init_node("ik_service_client")
    ik_service_client()

if __name__ == '__main__':
    main()
