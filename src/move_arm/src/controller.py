import rospy
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

TUCK_POSE = PoseStamped()

class Controller:
    def __init__(self):
        self.goal_last_updated = 0
        self.goal = None
        
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
        self.goal = response.joints[0]
        
    def control_loop(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.goal_last_updated + 3 < rospy.get_time():
                # if the goal hasn't been successfully updated in 3 seconds, reset the arm
                self.set_goal(TUCK_POSE)
                
            # TODO: move arm to goal
            
            r.sleep()
        
    def run(self):
        rospy.init_node('arm_controller')
        
        # queue_size=1 because we should discard messages if they are sending faster than we can process them
        rospy.Subscriber('arm_goal', PoseStamped, self.set_goal, queue_size=1) 
        
        self.control_loop()

if __name__ == '__main__':
    controller = Controller()
    controller.run()
