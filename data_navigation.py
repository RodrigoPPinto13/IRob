#!/usr/bin/env python

import rospy
import time
import psutil
import matplotlib.pyplot as plt
import statistics
import csv  # Import the csv module
from geometry_msgs.msg import PoseWithCovarianceStamped 
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped

# Variables to track data
path_lengths = []
times_to_goal = []
start_time = 0
goal_reached = False

goal_timeout = 600  # Maximum allowed time (in seconds) to reach a goal

# Position and path tracking
last_position = (0, 0)
distance_traveled = 0.0

# Goals to be sent
goals = [
    (-0.01, 1.98),
    (2.44, 0.4),
    (0.631, -4.34),
    (0.80, -1.75),
]

# Callback for result to determine success or failure
def result_callback(result_msg):
    global goal_reached, times_to_goal
    end_time = time.time()
    if result_msg.status.status == 3:  # SUCCESS status
        goal_reached = True
        rospy.loginfo("Goal reached.")
        times_to_goal.append(end_time - start_time)
    else:
        goal_reached = False
        rospy.logwarn("Goal not reached.")
        times_to_goal.append(float('inf'))  # Use infinity for failed goals

# Callback for amcl to calculate path length
def amcl_callback(amcl_msg):
    global last_position, distance_traveled
    position = amcl_msg.pose.pose.position
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
    global path_lengths, times_to_goal

    plt.figure(figsize=(12, 8))

    # Path Lengths Plot
    plt.subplot(2, 2, 1)
    plt.plot(range(len(path_lengths)), path_lengths, marker='o')
    plt.title("Path Lengths")
    plt.xlabel("Trial")
    plt.ylabel("Distance (meters)")
    plt.legend()

    # Time to Goal Plot
    plt.subplot(2, 2, 2)
    plt.plot(range(len(times_to_goal)), times_to_goal, marker='o')
    plt.title("Time to Goal")
    plt.xlabel("Trial")
    plt.ylabel("Time (seconds)")
    plt.legend()

    plt.tight_layout()
    plt.savefig("move_base_5.png")
    plt.show()

# Function to save data to CSV
def save_data_to_csv(filename):
    global path_lengths, times_to_goal
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Trial", "Path Length (m)", "Time to Goal (s)"])  # Header
        for i in range(len(path_lengths)):
            writer.writerow([i + 1, path_lengths[i], times_to_goal[i] if i < len(times_to_goal) else 'N/A'])

def main():
    global last_position, distance_traveled, path_lengths, goal_reached, start_time

    rospy.init_node('move_base_performance_monitor', anonymous=True)
    # Subscribers
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)

    rate = rospy.Rate(1)  # Run at 1 Hz
    
    start_time = time.time()

    for goal_pos in goals:
        # Send the goal and monitor performance
        send_goal(goal_pos)
        rospy.sleep(1)  # Allow time for goal publishing and processing

        goal_sent_time = time.time()  # Record when the goal was sent
        goal_reached = False

        # Monitor until goal is reached, failed, or timeout occurs
        while not rospy.is_shutdown():

            # Check if goal was reached
            if goal_reached:
                path_lengths.append(distance_traveled)
                rospy.loginfo(f"Path length for this goal: {distance_traveled}")
                break

            # Check if timeout occurr
            rate.sleep()

    # Plot results after all trials are done
    plot_results()

    # Save results to CSV
    save_data_to_csv("move_base_performance_5.csv")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down performance monitor.")
