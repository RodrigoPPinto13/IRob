#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry  # Adjust to the correct message type if needed
import rosbag

class GroundTruthDownsampler:
    def __init__(self, bag_file, output_topic):
        self.output_topic = output_topic
        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz

        with rosbag.Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == '/mocap':  # Adjust to your actual ground truth topic
                    self.pub.publish(msg)
                    self.rate.sleep()  # Sleep to maintain 1 Hz

def main():
    rospy.init_node('ground_truth_downsampler', anonymous=True)
    bag_file = rospy.get_param('~bag_file', 'catkin_ws/src/IRob-main/data/fixed_slam_easy.bag')  # Specify bag file path
    output_topic = '/ground_truth_downsampled'
    GroundTruthDownsampler(bag_file, output_topic)
    rospy.spin()

if __name__ == '__main__':
    main()
