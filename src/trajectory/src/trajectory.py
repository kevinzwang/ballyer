#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import matplotlib.pyplot as plt

class BallPredictorNode:
    def __init__(self, min_data_points=3, x_f=0.6):
        rospy.init_node('ball_predictor_node', anonymous=True)

        # Initialize arrays to store position data as NumPy arrays
        self.x_data = np.array([])
        self.y_data = np.array([])
        self.z_data = np.array([])

        # Subscribe to the "ball_position" topic
        rospy.Subscriber('ball_position', Point, self.position_callback)

        # Create a publisher for the "arm_goal" topic
        self.arm_goal_publisher = rospy.Publisher('arm_goal', PoseStamped, queue_size=10)

        # Initialize the plot
        '''
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Ball Trajectory Prediction')
        self.scatter = self.ax.scatter([], [], label='Real Position')
        self.prediction_line, = self.ax.plot([], [], label='Trajectory Prediction')
        self.ax.legend()
        
        '''
        # Define the final x value
        self.x_f = x_f

        # Minimum number of data points required for prediction
        self.min_data_points = min_data_points

    def position_callback(self, data):
        # Update arrays with new position data
        self.x_data = np.append(self.x_data, data.x)
        self.y_data = np.append(self.y_data, data.y)
        self.z_data = np.append(self.z_data, data.z)

        # Check if there are enough data points for prediction
        if len(self.x_data) >= self.min_data_points:
            # Predict and publish final position as an arm goal
            arm_goal = self.predict_final_position()
            self.arm_goal_publisher.publish(arm_goal)
            print(arm_goal)

            # Clear and redraw the entire plot
            # self.ax.clear()
            ''' 
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_title('Ball Trajectory Prediction')
            self.scatter = self.ax.scatter(self.x_data, self.y_data, self.z_data, label='Real Position')
            self.prediction_line, = self.ax.plot([], [], label='Trajectory Prediction')
            self.ax.legend()
            '''
            # Update the prediction line in the plot
            x_vals = np.linspace(min(self.x_data), max(self.x_data), 100)
            y_vals = np.polyval(np.polyfit(self.x_data, self.y_data, 1), x_vals)
            z_vals = np.polyval(np.polyfit(self.x_data, self.z_data, 2), x_vals)
            #self.prediction_line.set_data(x_vals, y_vals)

            # Draw the updated plot
            #self.fig.canvas.draw_idle()

    def predict_final_position(self):
        # Fit a linear curve to the current position data (X vs. Y)
        linear_coefficients = np.polyfit(self.x_data, self.y_data, 1)

        # Fit a quadratic curve to the current position data (X vs. Z)
        quadratic_coefficients = np.polyfit(self.x_data, self.z_data, 2)

        # Use the linear equation to predict y at the final x value
        predicted_y = np.polyval(linear_coefficients, self.x_f)

        # Use the quadratic equation to predict z at the final x value
        predicted_z = np.polyval(quadratic_coefficients, self.x_f)

        # Create a PoseStamped message for the arm goal
        arm_goal = PoseStamped()
        arm_goal.header.stamp = rospy.Time.now()
        arm_goal.pose.position.x = self.x_f - 0.0381  # x offset
        arm_goal.pose.position.y = predicted_y 
        arm_goal.pose.position.z = predicted_z - 0.1143  # z offset
        arm_goal.pose.orientation.x = 0.0
        arm_goal.pose.orientation.y = 0.707
        arm_goal.pose.orientation.z = 0.0
        arm_goal.pose.orientation.w = 0.707

        return arm_goal

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        predictor_node = BallPredictorNode(min_data_points=3, x_f=0.6)
        predictor_node.run()
    except rospy.ROSInterruptException:
        pass
