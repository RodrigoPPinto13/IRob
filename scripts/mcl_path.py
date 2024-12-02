#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped  # Import the correct message type

class AMCLPath:
    def __init__(self):
        rospy.init_node('amcl_path_publisher')
        
        self.path = Path()
        self.path.header.frame_id = "map"  # The frame ID should match your map frame

        # Subscriber to the AMCL estimated pose (PoseWithCovarianceStamped)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Publisher for the Path
        self.path_pub = rospy.Publisher('/amcl_path', Path, queue_size=10)

    def pose_callback(self, pose_msg):
        # Convert PoseWithCovarianceStamped to PoseStamped (because Path expects PoseStamped)
        pose_stamped = PoseStamped()
        pose_stamped.header = pose_msg.header
        pose_stamped.pose = pose_msg.pose.pose  # Extract the pose (ignoring the covariance)

        # Add the received pose to the path
        self.path.header.stamp = rospy.Time.now()  # Update the timestamp
        self.path.poses.append(pose_stamped)  # Add the pose to the list of poses

        # Publish the updated path
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        amcl_path_publisher = AMCLPath()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
