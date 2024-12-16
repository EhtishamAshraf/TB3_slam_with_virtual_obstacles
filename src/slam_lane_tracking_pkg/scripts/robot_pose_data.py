#!/usr/bin/env python3

# This node, subscribes to the /odom topic and saves the robot's pose data in a .json file

# Importing libraries:
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import numpy as np
import json
import os

# Empty lists to store the Pose data of the robot
X_list = []
Y_list = []
yaw_list = []

# Specify the path to your .json file
file_path = '/home/masters/catkin_ws/src/slam_lane_tracking_pkg/scripts/pose_data.json'
# file_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/1-Robotics/3-Robotics_Project/Navigation_virtual_lane_hardware_ws/src/slam_lane_tracking_pkg/scripts/pose_data.json'

# Odom callback to get the robot's pose data
def odom_callback(robot_pose):
    global position_list, X_list, Y_list, yaw_list

    # Extracting the (x, y) position of the robot from the Odometry message
    pos = robot_pose.pose.pose.position
    
    # Extracting the orientation quaternion and converting it to yaw (rotation about z-axis)
    orientation_q = robot_pose.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    
    # Limiting x, y, yaw to maximum of 2 decimal places
    x_rounded = round(pos.x, 2)
    y_rounded = round(pos.y, 2)
    yaw_rounded = round(yaw, 2)
    
    # Appending the data in the list
    X_list.append(x_rounded)
    Y_list.append(y_rounded)
    yaw_list.append(yaw_rounded)

    # Calling the read_existing_data to see if there's some data already present in the file
    data = read_existing_data(file_path)
    
    # opening and appending the data into the file
    data["Robot Position"] = {
        "X": X_list,
        "Y": Y_list,
        "Yaw": yaw_list
    }

    with open(file_path, 'w') as f:
        json.dump(data, f, indent=4)
    print("Position data updated successfully!")

# Function to read existing data from the JSON file
def read_existing_data(file_path):
    if os.path.exists(file_path):
        with open(file_path, 'r') as f:
            try:
                return json.load(f)
            except json.JSONDecodeError:
                return {}
    return {}

def main():
    global pub    
    rospy.init_node('lane_with_centroids', anonymous=True)              # initializing rosnode
    rospy.Subscriber('/odom', Odometry, odom_callback)                  # Subscribe to the '/odom' topic to get the robot's odometry data
    rospy.spin()                                                        # Keep the node running until shut down

if __name__ == '__main__':
    try:
        main()                          # Call the main function to start the node
    except rospy.ROSInterruptException: # Handle the case where ROS is interrupted
        pass

