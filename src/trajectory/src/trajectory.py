#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs import Point

x_data = np.array([])
y_data = np.array([])
z_data = np.array([])

x_f = 0.4

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

def new_point(Point):
    #Add point to existing point arrays
    x_data = np.append(xs, [Point.x])
    y_data = np.append(ys, [Point.y])
    z_data = np.append(zs, [Point.z])

    #Recalc trajectory
    y_f, z_f = calc_end_point()

    f_p = Point
    f_p.x = x_f
    f_p.y = y_f
    f_p.z = z_f

    #
    pub.publish(f_p)

    
#Calcs end point at x_f
def calc_end_point():
    # Fitting models to predict Y and Z based on X

    # Fitting models
    # Assuming a linear relationship for Y = f(X)
    coef_y_from_x = np.polyfit(x_data, y_data, 1)

    # Assuming a quadratic relationship for Z = f(X) (parabolic due to gravity)
    coef_z_from_x = np.polyfit(x_data, z_data, 2)

    # Function to predict Y and Z from X
    def predict_yz_from_x(x):
        y_predicted = np.polyval(coef_y_from_x, x)
        z_predicted = np.polyval(coef_z_from_x, x)
        return y_predicted, z_predicted

    # Predicting over a range of X values
    x_range = np.linspace(x_data[0], 0.4, 100)
    y_predicted, z_predicted = predict_yz_from_x(x_range)

    ax.clear()

    ax.scatter(x_data, y_data, z_data, color='b', label='Actual Trajectory')

    # Predicted trajectory based on X
    ax.plot(x_range, y_predicted, z_predicted, color='r', label='Predicted Trajectory from X')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('YZ Trajectory Prediction Based on X Coordinate')
    ax.legend()

    plt.show()

    return y_predicted[-1], z_predicted[-1]




    
def do_the_thing():
    rospy.Subscriber("john_thingy", Point, new_point)
    pub = rospy.Publisher('final_point', Point, queue_size=10)
    rospy.spin()
    


if __name__ == '__main__':
    rospy.init_node('traj_pub', anonymous=True)
    rospy.init_node('traj_sub', anonymous=True)
    do_the_thing()