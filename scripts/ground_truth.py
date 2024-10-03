#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Transform

# Callback to log differences
def compute_transform_difference(event):
    try:
        # Lookup transformation mocap -> mocap_laser_link
        (trans_laser, rot_laser) = listener.lookupTransform('/mocap', '/mocap_laser_link', rospy.Time(0))

        # Lookup transformation mocap -> base_footprint
        (trans_base, rot_base) = listener.lookupTransform('/mocap', '/base_footprint', rospy.Time(0))

        # Calculate the difference in translation (Euclidean distance)
        trans_diff = np.linalg.norm(np.array(trans_laser) - np.array(trans_base))

        # Convert rotations (quaternions) to Euler angles and compute the angular difference
        rot_laser_euler = tf.transformations.euler_from_quaternion(rot_laser)
        rot_base_euler = tf.transformations.euler_from_quaternion(rot_base)
        rot_diff = np.linalg.norm(np.array(rot_laser_euler) - np.array(rot_base_euler))

        # Log or publish the difference
        rospy.loginfo("Translation Difference: {:.4f} meters, Rotation Difference: {:.4f} radians".format(trans_diff, rot_diff))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform not available at this time")

if __name__ == '__main__':
    rospy.init_node('transform_difference_calculator')

    listener = tf.TransformListener()

    # Call the function at 1 Hz
    rospy.Timer(rospy.Duration(1.0), compute_transform_difference)

    rospy.spin()
