#!/usr/bin/env python

import rospy
import time
import psutil
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped

# Variables to track data
path_lengths = []
times_to_goal = []
success_rates = []
cpu_usages = []
start_time = 0
goal_reached = False

goal_timeout = 600  # Maximum allowed time (in seconds) to reach a goal

# Position and path tracking
last_position = (0, 0)
distance_traveled = 0.0

# Goals to be sent (automate this instead of using RViz's 2D Nav Goal tool)
goals = [
    (-0.5, -2.0),
    (0.5, -2.0),
    (-0.5, 2.0),
    (0.5, 2.0),
]

# Callback to track CPU usage
def record_cpu_usage():
    cpu_percent = psutil.cpu_percent(interval=1)
    cpu_usages.append(cpu_percent)

# Callback for goal sent
def goal_callback(goal_msg):
    global start_time
    rospy.loginfo("Goal sent, recording start time.")
    start_time = time.time()

# Callback for result to determine success or failure
def result_callback(result_msg):
    global goal_reached, times_to_goal
    end_time = time.time()
    if result_msg.status.status == 3:  # 3 is the SUCCESS status in MoveBaseActionResult
        goal_reached = True
        rospy.loginfo("Goal reached.")
        times_to_goal.append(end_time - start_time)
    else:
        goal_reached = False
        rospy.logwarn("Goal not reached.")
        times_to_goal.append(float('inf'))  # Use infinity for failed goals

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

# Function to plot results
def plot_results():
    global path_lengths, times_to_goal, cpu_usages

    plt.figure(figsize=(12, 8))

    # Plot path lengths
    plt.subplot(2, 2, 1)
    plt.plot(range(len(path_lengths)), path_lengths, marker='o')
    plt.title("Path Lengths")
    plt.xlabel("Trial")
    plt.ylabel("Distance (meters)")

    # Plot times to goal
    plt.subplot(2, 2, 2)
    plt.plot(range(len(times_to_goal)), times_to_goal, marker='o')
    plt.title("Time to Goal")
    plt.xlabel("Trial")
    plt.ylabel("Time (seconds)")

    # Plot success rates
    plt.subplot(2, 2, 3)
    success = [1 if t != float('inf') else 0 for t in times_to_goal]
    plt.plot(range(len(success)), success, marker='o')
    plt.title("Success Rate")
    plt.xlabel("Trial")
    plt.ylabel("Success (1 = reached, 0 = failed)")

    # Plot CPU usage
    plt.subplot(2, 2, 4)
    plt.plot(range(len(cpu_usages)), cpu_usages, marker='o')
    plt.title("CPU Usage")
    plt.xlabel("Time (s)")
    plt.ylabel("CPU Usage (%)")

    plt.tight_layout()
    plt.savefig("move_base_original.png")
    plt.show()

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

        goal_sent_time = time.time()  # Record when the goal was sent
        goal_reached = False

        # Monitor until goal is reached, failed, or timeout occurs
        while not rospy.is_shutdown():
            record_cpu_usage()  # Record CPU usage
            elapsed_time = time.time() - goal_sent_time

            # Check if goal was reached
            if goal_reached:
                path_lengths.append(distance_traveled)
                rospy.loginfo(f"Path length for this goal: {distance_traveled}")
                break

            # Check if timeout occurred
            if elapsed_time >= goal_timeout:
                rospy.logwarn(f"Goal at {goal_pos} timed out after {goal_timeout} seconds.")
                times_to_goal.append(goal_timeout)
                path_lengths.append(distance_traveled)
                break

            rate.sleep()

    # Plot results after all trials are done
    plot_results()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down performance monitor.")
