#!/usr/bin/env python

import rospy
import time
import psutil
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped

# Variables to track data
goal_reached = False

goal_timeout = 600  # Maximum allowed time (in seconds) to reach a goal

# Position and path tracking
last_position = (0, 0)
distance_traveled = 0.0

# Goals to be sent (automate this instead of using RViz's 2D Nav Goal tool)
goals = [
    (0.05, 1.98),
    (2.44, 0.4),
    (-0.14, -3.88),
    (0.80, -1.75),
]

# Callback for result to determine success or failure
def result_callback(result_msg):
    global goal_reached
    if result_msg.status.status == 3:  # 3 is the SUCCESS status in MoveBaseActionResult
        goal_reached = True
        rospy.loginfo("Goal reached.")
    else:
        goal_reached = False
        rospy.logwarn("Goal not reached.")

# Callback for odometry to calculate path length
def odom_callback(odom_msg):
    global last_position, distance_traveled
    position = odom_msg.pose.pose.position
    distance = ((position.x - last_position[0]) ** 2 + (position.y - last_position[1]) ** 2) ** 0.5
    distance_traveled += distance
    last_position = (position.x, position.y)

# Function to send goals
def send_goal(goal_pos):
    global distance_traveled, last_position

    # Reset distance tracking for each new goal
    distance_traveled = 0.0
    last_position = (0, 0)

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Allow some time for the publisher to register

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    # Define goal coordinates
    goal.pose.position.x = goal_pos[0]
    goal.pose.position.y = goal_pos[1]
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0  # No rotation

    rospy.loginfo(f"Sending goal to /move_base_simple/goal at ({goal_pos[0]}, {goal_pos[1]})")
    goal_pub.publish(goal)

def main():
    global last_position, distance_traveled, path_lengths, goal_reached

    rospy.init_node('move_base_performance_monitor', anonymous=True)

    # Subscribers
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(1)  # Run at 1 Hz

    for goal_pos in goals:
        # Send the goal and monitor performance
        send_goal(goal_pos)
        rospy.sleep(1)  # Allow time for goal publishing and processing

        goal_reached = False

        # Monitor until goal is reached, failed, or timeout occurs
        while not rospy.is_shutdown():

            # Check if goal was reached
            if goal_reached:
                path_lengths.append(distance_traveled)
                rospy.loginfo(f"Path length for this goal: {distance_traveled}")
                break

            rate.sleep()

    # Plot results after all trials are done
    plot_results()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down performance monitor.")
