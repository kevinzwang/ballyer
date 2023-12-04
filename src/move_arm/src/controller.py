#!/usr/bin/env python

import rospy
import intera_interface
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

TUCK_POSE = Pose()
TUCK_POSE.position.x = 0.6
TUCK_POSE.position.y = 0.0
TUCK_POSE.position.z = 0.4
TUCK_POSE.orientation.x = 0
TUCK_POSE.orientation.y = 2 ** 0.5 / 2
TUCK_POSE.orientation.z = 0
TUCK_POSE.orientation.w = 2 ** 0.5 / 2


class Controller:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        
        self.goal_last_updated = 0
        self.goal = None
        self.limb = intera_interface.Limb('right')
        
        service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
        self.ik_service_proxy = rospy.ServiceProxy(service_name, SolvePositionIK)
        rospy.wait_for_service(service_name, 5.0)
        
    def set_goal(self, pose_stamped):
        print("Computing new IK")

        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(pose_stamped)
        ik_request.tip_names.append('right_hand')
        
        # seed with the current pos so that the new goal is close
        # if too many IK requests fail we should remove this
        seed = JointState()
        for name, pos in self.limb.joint_angles().items():
            seed.name.append(name)
            seed.position.append(pos)
        ik_request.seed_angles.append(seed)
        
        try:
            response = self.ik_service_proxy(ik_request)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return
        
        if response.result_type[0] <= 0:
            rospy.logerr("IK failed")
            return
                
        self.goal_last_updated = rospy.get_time()
        self.goal = response.joints[0].position
        
    def control_loop(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.goal_last_updated + 10 < rospy.get_time():
                # if the goal hasn't been successfully updated in a while, reset the arm
                tuck_pose_stamped = PoseStamped()
                tuck_pose_stamped.pose = TUCK_POSE
                tuck_pose_stamped.header = Header(stamp=rospy.Time.now(), frame_id='base')

                self.set_goal(tuck_pose_stamped)
                
            if self.goal is not None:
                goal_dict = dict(zip(self.limb.joint_names(), self.goal))
                self.limb.set_joint_position_speed(0.9)
                self.limb.set_joint_positions(goal_dict)
            
            r.sleep()
            
    def shutdown(self):
        rospy.loginfo("Shutting down arm controller")
        self.limb.exit_control_mode()
        rospy.sleep(0.1)
        
    def run(self):
        # queue_size=1 because we should discard messages if they are sending faster than we can process them
        rospy.Subscriber('arm_goal', PoseStamped, self.set_goal, queue_size=1) 
        
        self.control_loop()

if __name__ == '__main__':
    rospy.init_node('arm_controller')
    
    controller = Controller()
    controller.run()
