import rospy
import intera_interface
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

TUCK_POSE = PoseStamped()
# TODO: set tuck pose

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
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(pose_stamped)
        ik_request.tip_names.append('right_hand')
        
        # seed with the old goal so that the new goal is close to the old goal
        # if too many IK requests fail we should remove this
        if self.goal is not None:
            ik_request.seed_angles.append(self.goal)
        
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
            if self.goal_last_updated + 5 < rospy.get_time():
                # if the goal hasn't been successfully updated in a while, reset the arm
                self.set_goal(TUCK_POSE)
                
            if self.goal is not None:
                goal_dict = dict(zip(self.limb.joint_names(), self.goal))
                self.limb.set_joint_position_speed(0.5)
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
