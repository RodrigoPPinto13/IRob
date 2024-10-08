#!/usr/bin/env python

import rospy
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose

class GroundTruthSubscriber:
    def __init__(self):
        self.ground_truth_pub = rospy.Publisher('/ground_truth_path', Path, queue_size=10)

        self.ground_truth_path = Path()
        self.ground_truth_path.header.frame_id = "mocap"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rate = rospy.Rate(1)  # 1 Hz

        self.process_tf()

    def process_tf(self):
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform('mocap', 'mocap_laser_link', rospy.Time(0))

                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "mocap"
                
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation

                self.ground_truth_path.poses.append(pose)
                self.ground_truth_path.header.stamp = pose.header.stamp

                self.ground_truth_pub.publish(self.ground_truth_path)

            except tf2_ros.LookupException:
                rospy.logwarn("Transform between 'mocap' and 'mocap_laser_link' not available yet")
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn("Transform exception: {}".format(e))

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ground_truth_subscriber')
    ground_truth_subscriber = GroundTruthSubscriber()
    rospy.spin()
