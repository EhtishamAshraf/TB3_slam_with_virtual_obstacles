#!/usr/bin/env python3

# This node takes as input the map created with Gmapping and also takes the
# odom data and centroids of the lanes and gives a processed map with
# both lanes marked as a virtual obstacle for navigation

# Importing libraries
import rospy
from PIL import Image
import numpy as np
import math
import json

# Function to convert real world coordinates to the map coordinates:
def real_world_to_map(real_x, real_y, map_origin, map_resolution, map_height):
    pixel_x = int((real_x - map_origin[0]) / map_resolution)
    pixel_y = int(map_height - (real_y - map_origin[1]) / map_resolution) # subtract from the map_height because real world coordinates have origin at bottom-left, whereas map has origin at top-left
    return pixel_x, pixel_y

# Function to process the map and provide with a new map, which has lanes marked as virtual obstacles
def process_map():
    rospy.init_node('virtual_obstacles_node', anonymous=True)
    rospy.loginfo("Virtual Obstacles Node Started.")

    # Map parameters
    map_resolution = 0.05               # meters per pixel
    map_origin = [-10.0, -10.0]         # Origin in real-world coordinates (the real-world coordinates of the bottom-left corner of the map in meters)
    map_width = 384                     # Map size in pixels (width)
    map_height = 384                    # Map size in pixels (height)
    
    output_map_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/1-Robotics/3-Robotics_Project/Navigation_virtual_lane_hardware_ws/src/slam_lane_tracking_pkg/maps/new_map_obs.pgm'
    map_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/1-Robotics/3-Robotics_Project/Navigation_virtual_lane_hardware_ws/src/slam_lane_tracking_pkg/maps/new_map.pgm'

    # Loading robot pose and lane centroid data
    pose_file_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/1-Robotics/3-Robotics_Project/Navigation_virtual_lane_hardware_ws/src/slam_lane_tracking_pkg/scripts/pose_data.json'
    centroid_pose_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/1-Robotics/3-Robotics_Project/Navigation_virtual_lane_hardware_ws/src/slam_lane_tracking_pkg/scripts/centroid_data.json'

    # Open the Pose data file and save the data in a list
    with open(pose_file_path, 'r') as f:
        pose_data_json = json.load(f)
    robot_positions = pose_data_json.get("Robot Position", {})
    x_positions = robot_positions.get("X", [])
    y_positions = robot_positions.get("Y", [])
    yaw_positions = robot_positions.get("Yaw", [])

    # Open the centroids data file and save the data in a list
    with open(centroid_pose_path, 'r') as f:
        centroids_data_json = json.load(f)
    centroid_data = centroids_data_json.get("Lanes Data", {})
    left_line_position = centroid_data.get("Left X", [])
    right_line_position = centroid_data.get("Right X", [])
    centroid_data = centroids_data_json.get("Centre Data", {})
    centre_line_position = centroid_data.get("Centre X", [])

    # Prepare coordinate lists
    real_world_coords = []
    centroids_in_world_coords = []
    
    # appending the data in the lists. 
    for x, y, yaw, left_x, right_x, centre_x in zip(x_positions, y_positions, yaw_positions, left_line_position, right_line_position, centre_line_position):
        real_world_coords.append((x, y, yaw))
        centroids_in_world_coords.append((left_x, right_x, centre_x))

    rospy.loginfo(f"Number of robot positions: {len(real_world_coords)}")
    rospy.loginfo(f"Number of line points: {len(centroids_in_world_coords)}")

    # Load map and set all obstacles to free space before adding virtual obstacles
    img = Image.open(map_path)
    map_pixels = np.array(img)
    map_pixels[map_pixels == 0] = 255  # Reset obstacles to free space

    # Initializing variables for consecutive occurrence tracking
    consecutive_upper_line = None
    consecutive_lower_line = None
    upper_count = 0
    lower_count = 0

    # Processing the coordinates
    for (real_x, real_y, pose), (L_x, R_x, C_x) in zip(real_world_coords, centroids_in_world_coords):
        upper_line = round(abs((C_x - L_x)) * 0.01, 2)  # using only up to decimals
        lower_line = round(abs((C_x - R_x)) * 0.01, 2) 

        # Checking consecutive upper_line occurrences
        if upper_line == consecutive_upper_line:
            upper_count += 1
        else:
            consecutive_upper_line = upper_line
            upper_count = 1

        # Checking consecutive lower_line occurrences
        if lower_line == consecutive_lower_line:
            lower_count += 1
        else:
            consecutive_lower_line = lower_line
            lower_count = 1

        # Only process if a value occurs 3 times in a row - this will make sure that we don't use outliers 
        if upper_count >= 3:
            if upper_line < 0.14 or upper_line > 0.18: # if upper line is far away from the robot's center then cap it to 0.14 meter
                upper_line = 0.14    
            # upper_line = upper_line + 0.04      

            print("Processing upper_line after 3 consecutive occurrences:", upper_line)
            
            # Calculate coordinates for upper line
            real_upper_x = real_x + upper_line * math.cos(pose - 1.5708)
            real_upper_y = real_y + upper_line * math.sin(pose - 1.5708)
            map_upper_x, map_upper_y = real_world_to_map(real_upper_x, real_upper_y, map_origin, map_resolution, map_height)

            if 0 <= map_upper_x < map_width and 0 <= map_upper_y < map_height:
                map_pixels[map_upper_y, map_upper_x] = 0  # Mark upper line obstacle

        if lower_count >= 3:
            print("Processing lower_line after 3 consecutive occurrences:", lower_line)
            
            if lower_line < 0.14 or lower_line > 0.18: # if lower line is far away from the robot's center then cap it to 0.14 meter
                lower_line = 0.14   
            # lower_line = lower_line + 0.04
            
            # Calculate coordinates for lower line
            real_lower_x = real_x + lower_line * math.cos(pose + 1.5708)
            real_lower_y = real_y + lower_line * math.sin(pose + 1.5708)
            map_lower_x, map_lower_y = real_world_to_map(real_lower_x, real_lower_y, map_origin, map_resolution, map_height)

            if 0 <= map_lower_x < map_width and 0 <= map_lower_y < map_height:
                map_pixels[map_lower_y, map_lower_x] = 0  # Mark lower line obstacle

    # Save the modified map
    modified_img = Image.fromarray(map_pixels)
    modified_img.save(output_map_path)
    rospy.loginfo(f"Modified map saved to: {output_map_path}")

if __name__ == '__main__':
    try:
        process_map()
    except rospy.ROSInterruptException:
        pass
