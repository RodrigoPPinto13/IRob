#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

x_init = 1.2173992395401
y_init = -5.0

def callback(data):
    adjusted_odom = Odometry()
    adjusted_odom.header = data.header
    adjusted_odom.child_frame_id = data.child_frame_id

    adjusted_odom.pose.pose.position.x = data.pose.pose.position.x - x_init
    adjusted_odom.pose.pose.position.y = data.pose.pose.position.y - y_init

    adjusted_odom.pose.pose.orientation = data.pose.pose.orientation
    adjusted_odom.twist = data.twist

    pub.publish(adjusted_odom)

if __name__ == '__main__':
    rospy.init_node('odom_adjuster')

    pub = rospy.Publisher('/adjusted_odom', Odometry, queue_size=10)

    rospy.Subscriber('/odom', Odometry, callback)

    rospy.spin()
