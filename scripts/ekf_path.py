#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdometryPath:
    def __init__(self):
        # Publisher for the path generated from /odometry/filtered
        self.path_pub = rospy.Publisher('/ekf_path', Path, queue_size=10)

        # Path message to store the accumulated poses
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = "odom"  # Set the frame ID (usually "odom" or "map")

        # Subscribe to the /odometry/filtered topic
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Create a PoseStamped message from the Odometry message
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose  # Copy pose from Odometry message

        # Append the pose to the Path message
        self.ekf_path.poses.append(pose_stamped)

        # Update the path's header with the current time
        self.ekf_path.header.stamp = rospy.Time.now()

        # Publish the updated path
        self.path_pub.publish(self.ekf_path)

if __name__ == '__main__':
    rospy.init_node('odometry_path_node')
    odometry_path = OdometryPath()
    rospy.spin()
