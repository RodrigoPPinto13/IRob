#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import signal
import sys

class EKFErrorComputation2D:
    def __init__(self):
        # Lists to store time, errors, and uncertainties
        self.time_stamps = []
        self.error_x = []
        self.error_y = []
        self.uncertainty_x = []
        self.uncertainty_y = []

        # Subscribe to /odometry/filtered topic (EKF estimate)
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        # TF listener to get the ground truth from the motion capture system
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def odom_callback(self, msg):
        # Get the current time for plotting
        current_time = rospy.get_time()

        # Extract the EKF estimated position (x, y)
        ekf_position = msg.pose.pose.position

        # Extract covariance (for uncertainty calculation)
        covariance_matrix = np.array(msg.pose.covariance).reshape((6, 6))  # 6x6 covariance matrix
        position_covariance = covariance_matrix[0:2, 0:2]  # Extract the x, y covariance
        position_uncertainty = np.sqrt(np.diag(position_covariance))

        # Try to get the ground truth pose from the TF tree
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'mocap_laser_link', rospy.Time(0), rospy.Duration(1.0))
            ground_truth_position = transform.transform.translation

            # Calculate error in position (EKF estimate minus ground truth)
            error_x = ekf_position.x - ground_truth_position.x
            error_y = ekf_position.y - ground_truth_position.y

            # Append time, error, and uncertainty to the lists
            self.time_stamps.append(current_time)
            self.error_x.append(error_x)
            self.error_y.append(error_y)
            self.uncertainty_x.append(position_uncertainty[0])
            self.uncertainty_y.append(position_uncertainty[1])

        except tf2_ros.LookupException as ex:
            rospy.logwarn("TF lookup failed: {}".format(ex))
        except tf2_ros.ExtrapolationException as ex:
            rospy.logwarn("TF extrapolation failed: {}".format(ex))

    def plot_errors(self, save_path='ekf_error_plot.png'):
        # Plot the errors in x and y
        plt.figure(figsize=(10, 6))
        
        # Error in X
        plt.subplot(2, 1, 1)
        plt.plot(self.time_stamps, self.error_x, label="Error in X", color='blue')
        plt.fill_between(self.time_stamps, np.array(self.error_x) - np.array(self.uncertainty_x),
                         np.array(self.error_x) + np.array(self.uncertainty_x), color='b', alpha=0.2)
        plt.title('Position Error and Uncertainty Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Error X (m)')
        plt.legend()

        # Error in Y
        plt.subplot(2, 1, 2)
        plt.plot(self.time_stamps, self.error_y, label="Error in Y", color='green')
        plt.fill_between(self.time_stamps, np.array(self.error_y) - np.array(self.uncertainty_y),
                         np.array(self.error_y) + np.array(self.uncertainty_y), color='g', alpha=0.2)
        plt.xlabel('Time (s)')
        plt.ylabel('Error Y (m)')
        plt.legend()

        plt.tight_layout()

        # Save the plot to a file
        plt.savefig('/home/alexandre/error.png')

        # Show the plot
        plt.show()

def signal_handler(sig, frame):
    # When Ctrl+C is pressed, plot and save the data
    rospy.loginfo("Shutting down, plotting and saving data...")
    ekf_error_computation.plot_errors('ekf_error_plot.png')
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('ekf_error_computation_node_2d')
    ekf_error_computation = EKFErrorComputation2D()

    # Set up signal handler to catch Ctrl+C and plot the results
    signal.signal(signal.SIGINT, signal_handler)

    # Spin ROS node
    rospy.spin()
