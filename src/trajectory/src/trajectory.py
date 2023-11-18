#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt

class BallPredictorNode:
    def __init__(self, min_data_points=3):
        rospy.init_node('ball_predictor_node', anonymous=True)

        # Initialize arrays to store position data as NumPy arrays
        self.x_data = np.array([])
        self.y_data = np.array([])
        self.z_data = np.array([])

        # Subscribe to the "ball_position" topic
        rospy.Subscriber('ball_position', Point, self.position_callback)

        # Create a publisher for the "final_position" topic
        self.final_position_publisher = rospy.Publisher('final_position', Point, queue_size=10)

        # Initialize the plot
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Ball Trajectory Prediction')
        self.scatter = self.ax.scatter([], [], [], label='Real Position')
        self.prediction_line, = self.ax.plot([], [], [], label='Trajectory Prediction')
        self.ax.legend()

        # Define the final x value
        self.x_f = 0.4

        # Minimum number of data points required for prediction
        self.min_data_points = min_data_points

    def position_callback(self, data):
        # Update arrays with new position data
        self.x_data = np.append(self.x_data, data.x)
        self.y_data = np.append(self.y_data, data.y)
        self.z_data = np.append(self.z_data, data.z)

        # Update the plot
        self.scatter.set_offsets(np.column_stack((self.x_data, self.y_data, self.z_data)))
        self.ax.figure.canvas.draw()

        # Check if there are enough data points for prediction
        if len(self.x_data) >= self.min_data_points:
            # Predict and publish final position
            final_position = self.predict_final_position()
            self.final_position_publisher.publish(final_position)

    def predict_final_position(self):
        # Fit a linear curve to the current position data (X vs. Y)
        linear_coefficients = np.polyfit(self.x_data, self.y_data, 1)

        # Fit a quadratic curve to the current position data (X vs. Z)
        quadratic_coefficients = np.polyfit(self.x_data, self.z_data, 2)

        # Use the linear equation to predict y at the final x value
        predicted_y = np.polyval(linear_coefficients, self.x_f)

        # Use the quadratic equation to predict z at the final x value
        predicted_z = np.polyval(quadratic_coefficients, self.x_f)

        # Create a Point message for the final position
        final_position = Point()
        final_position.x = self.x_f
        final_position.y = predicted_y
        final_position.z = predicted_z

        # Update the prediction line in the plot
        x_vals = np.linspace(min(self.x_data), self.x_f, 100)
        y_vals = np.polyval(linear_coefficients, x_vals)
        z_vals = np.polyval(quadratic_coefficients, x_vals)
        self.prediction_line.set_data(x_vals, y_vals)
        self.prediction_line.set_3d_properties(z_vals)

        return final_position

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        predictor_node = BallPredictorNode()
        predictor_node.run()
    except rospy.ROSInterruptException:
        pass
