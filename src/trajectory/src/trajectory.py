#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Header


class BallPredictorNode:
    def __init__(self, min_data_points=3, x_f=0.6, plot_interval=None):
        self.x_f = x_f
        self.min_data_points = min_data_points
        self.plot_interval = plot_interval

        rospy.init_node('ball_predictor_node', anonymous=True)

        self.x = []
        self.y = []
        self.z = []

        rospy.Subscriber('ball_position', PointStamped, self.position_callback)

        self.arm_goal_publisher = rospy.Publisher('arm_goal', PoseStamped, queue_size=1)
        
        self.ball_last_seen = 0
        
    def position_callback(self, data):
        if self.ball_last_seen + 10 < rospy.get_time():
            rospy.loginfo("Ball not seen in a while, resetting data")
            self.x = []
            self.y = []
            self.z = []

        if data.point.x < self.x_f or data.point.z < 0:
            # ignore balls past the plane
            return
        
        self.x.append(data.point.x)
        self.y.append(data.point.y)
        self.z.append(data.point.z)
        self.ball_last_seen = rospy.get_time()
        
        if len(self.x) >= self.min_data_points:
            x_f, y_f, z_f = self.predict_final_position()
            
            arm_goal = PoseStamped()
            arm_goal.header = Header(stamp=rospy.Time.now(), frame_id='base')
            arm_goal.pose.position.x = x_f - 0.0381  # x offset
            arm_goal.pose.position.y = y_f
            arm_goal.pose.position.z = z_f - 0.1143  # z offset
            arm_goal.pose.orientation.x = 0.0
            arm_goal.pose.orientation.y = 0.707
            arm_goal.pose.orientation.z = 0.0
            arm_goal.pose.orientation.w = 0.707
            
            self.arm_goal_publisher.publish(arm_goal)
            print(arm_goal)

    def predict_final_position(self, plot=False):
        # y is a linear function of x
        y_coeff = np.polyfit(self.x, self.y, 1)
        
        # z is a quadratic function of x, since there is gravity
        z_coeff = np.polyfit(self.x, self.z, 2)
        
        y_f = np.polyval(y_coeff, self.x_f)
        z_f = np.polyval(z_coeff, self.x_f)
        
        if plot:
            ax = plt.figure().add_subplot(projection='3d')
            
            x = np.linspace(self.x[0], self.x_f, 100)
            y = np.polyval(y_coeff, x)
            z = np.polyval(z_coeff, x)
            
            ax.plot(x, y, z)
            ax.plot(self.x, self.y, self.z, 'ro', label='data')
            ax.plot(self.x_f, y_f, z_f, 'go', label='goal')
            
            ax.legend()
            
            plt.show()
            print("should show plot")
        
        return (self.x_f, y_f, z_f)
        
    def run(self):
        r = rospy.Rate(10)

        last_plot_time = 0
        while not rospy.is_shutdown():
            t = rospy.get_time()
            if len(self.x) >= self.min_data_points:
                if self.plot_interval is not None and last_plot_time + self.plot_interval <= t:
                    self.predict_final_position(plot=True)
                    last_plot_time = t
            r.sleep()
            

if __name__ == '__main__':
    try:
        predictor_node = BallPredictorNode(min_data_points=3, x_f=0.6)
        predictor_node.run()
    except rospy.ROSInterruptException:
        pass
