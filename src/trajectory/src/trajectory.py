#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import matplotlib.pyplot as plt

class BallPredictorNode:
    def __init__(self, min_data_points=3, x_f=0.6, plot_interval=None):
        self.x_f = x_f
        self.min_data_points = min_data_points
        self.plot_interval = plot_interval

        rospy.init_node('ball_predictor_node', anonymous=True)

        self.x = []
        self.y = []
        self.z = []

        rospy.Subscriber('ball_position', Point, self.position_callback)

        self.arm_goal_publisher = rospy.Publisher('arm_goal', PoseStamped)
        
    def position_callback(self, data):
        self.x.append(data.x)
        self.y.append(data.y)
        self.z.append(data.z)
        
        if len(self.x) >= self.min_data_points:
            x_f, y_f, z_f = self.predict_final_position()
            
            arm_goal = PoseStamped()
            arm_goal.header.stamp = rospy.Time.now()
            arm_goal.pose.position.x = x_f - 0.0381  # x offset
            arm_goal.pose.position.y = y_f
            arm_goal.pose.position.z = z_f - 0.1143  # z offset
            arm_goal.pose.orientation.x = 0.0
            arm_goal.pose.orientation.y = 0.707
            arm_goal.pose.orientation.z = 0.0
            arm_goal.pose.orientation.w = 0.707
            
            self.arm_goal_publisher.publish(arm_goal)
            print(arm_goal)

    def predict_final_position(self):
        # y is a linear function of x
        y_coeff = np.polyfit(self.x, self.y, 1)
        
        # z is a quadratic function of x, since there is gravity
        z_coeff = np.polyfit(self.x, self.z, 2)
        
        y_f = np.polyval(y_coeff, self.x_f)
        z_f = np.polyval(z_coeff, self.x_f)
        
        if self.plot_interval is not None and (len(self.x) - self.min_data_points) % self.plot_interval == 0:
            ax = plt.figure().add_subplot(projection='3d')
            
            x = np.linspace(self.x[0], self.x_f, 100)
            y = np.polyval(y_coeff, x)
            z = np.polyval(z_coeff, x)
            
            ax.plot(x, y, z)
            ax.plot(self.x, self.y, self.z, 'ro', label='data')
            ax.plot(self.x_f, y_f, z_f, 'go', label='goal')
            
            ax.legend()
            
            plt.show()
        
        return (self.x_f, y_f, z_f)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        predictor_node = BallPredictorNode(min_data_points=3, x_f=0.6)
        predictor_node.run()
    except rospy.ROSInterruptException:
        pass
