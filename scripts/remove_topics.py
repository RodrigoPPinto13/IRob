import rosbag

# Input and output bag file paths
input_bag_path = '/home/alexandre/catkin_ws/src/irob-main/bag/bruh.bag'
output_bag_path = '/home/alexandre/catkin_ws/src/irob-main/bag/bruh_modified.bag'

x_init = 1.2173992395401
y_init = -5.0

with rosbag.Bag(input_bag_path, 'r') as input_bag:
    # Create a new bag file for writing
    with rosbag.Bag(output_bag_path, 'w') as output_bag:
        # Iterate through messages in the input bag
        for topic, msg, t in input_bag.read_messages():
            # Check if the topic is not the map topic
            if topic not in ['/map', '/map_metadata', '/turtlebot3_slam_gmapping/entropy', '/rosout', '/rosout_agg']:
                output_bag.write(topic, msg, t)
