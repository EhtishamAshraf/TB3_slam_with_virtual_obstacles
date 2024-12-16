# ROS-based Lane Tracking using Perception and Autonomous Navigation by SLAM using real Turtlebot3
This repository uses "Turtlebot3 package" and also features a custom ROS package, "slam_lane_tracking_pkg", combined in a way to enable Turtlebot3 to autonomously navigate an autorace environment by tracking lanes and simultaneously mapping the surrounding. The package equips Turtlebot3 with the ability to track lanes (yellow and white lines) using camera sensor data.

Once the map is created, the "robot’s pose data" and "lane's centroid data" is used to designate the detected lanes as virtual obstacles, ensuring they are avoided during autonomous navigation. 
To check the simulation results please refer to the following two repositories: [autonomous lane tracking simulation.](https://github.com/EhtishamAshraf/turtlebot3_lane_tracking.git) and
[virtual lane marking and navigation simulation.](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation.git)

### System's Block Diagram
The below diagram shows the project's overview and system's block diagram respectively.

![system overview](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/9cc6b523596605adb87029ce7314993fbb587df0/src/slam_lane_tracking_pkg/Image/12-flow_chart.png)

![block diagram](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/9cc6b523596605adb87029ce7314993fbb587df0/src/slam_lane_tracking_pkg/Image/11-block_diagram.png)

### Demo Video
You can watch the demo video of Turtlebot3 tracking the lane in real arena by clicking on the below image
[![Watch the video](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation/blob/main/src/slam_lane_tracking_pkg/Image/video_image.png)](https://youtu.be/-JGLHBf8EOU)

You can watch the demo video of virtual lane marking and Autonomous Navigation by a Turtlebot3 in real world arena by clicking on the below image
[![Watch the video](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation/blob/main/src/slam_lane_tracking_pkg/Image/navigation.png)](https://youtu.be/vERA8F4Mlvc)

## Environment
Below image shows the real world arena used in this project. The world contains white and yellow lines.
![World](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/9cc6b523596605adb87029ce7314993fbb587df0/src/slam_lane_tracking_pkg/Image/Arena_1.png)

## Packages used
Following are the packages used in this project

**1. Gmapping:**                 used for SLAM

**2. Move Base:**                used for Autonomous Navigation

**3. turtlebot3_autorace_2020:** used for tracking the lanes (it has been updated to detect and save the lanes centroids data in a .json file)

**4. slam_lane_tracking_pkg:**   used for processing the map, saving the robot's pose data in a .json file

---

### Notes 
1.  Details about cloning the repository are given at the end of this **readme file**
2.  Please note that the "lane tracking from scratch" has been implemented in this repo: [autonomous lane tracking simulation.](https://github.com/EhtishamAshraf/turtlebot3_lane_tracking.git) and autonomous navigation and mapping from scratch has been implemented in this repo: [virtual lane marking and navigation simulation.](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation.git)
3.  If you plan to work on Simulation, then please download the package named **turtlebot3_simulations**
4.  If you work on hardware, please refer to the **Hardware Testing** section of the following repo: [Turtlebot3 Hardware testing](https://github.com/EhtishamAshraf/TurtleBotTrajControl_SMC.git) to connect to the real Turtlebot3.

---

## Camera Calibration
The process of determining the camera’s parameters to improve accuracy in visual perception tasks.
Camera calibration consists of:
1. Intrinsic Calibration
2. Extrinsic Calibration

![calibration](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/581b85ea0a86228bb4897a8d718a96c61d755c8c/src/slam_lane_tracking_pkg/Image/3.png)

![calibration](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/16-world_to_image_plane.png)

**Intrinsic Calibration:**
    It focuses on the camera's internal properties and ensures accurate measurement of distances, shapes, and angles in captured images:
1. Focal Length: Determines the zoom level.
2. Principal Point: The center of the camera's imaging sensor.
3. Distortion Coefficients: Corrects fish-eye or barrel distortions.

**Extrinsic Calibration:**
    It defines the camera's position and orientation in the robot's coordinate system and is
critical for integrating camera data with camera and other sensors for SLAM 
and navigation:
1. Translation (x, y, z): The physical distance from the robot’s center to the camera.
2. Rotation: The angle of the camera relative to the robot’s frame of reference.
   
![calibration](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/17-calibration.png)

![calibration](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/18-intrinsic_extrinsic.png)

## 1. Lane Tracking Logic
  - Convert RGB image to HSV
  - Define the HSV range for detecting white and yellow color lines in the image
  - Create and display the masked image (highlighting the white and yellow detected lines)
  - Combine the white and yellow masks using a bitwise OR operation, allowing both lane 
    lines to be   
    visible in a single image
  - Find the contours in the masked image
  - If contours exist:
      - Get the second and third biggest contour based on the area
      - apply bitwise_and to extract the region where yellow and white lines are present 
        in the 
        original image
      - draw the contour for visualization
      - find the moments of the selected contours
      - find the centroids of the selected contours from the moments
      - draw a circle to highlight the centroid

![Camera Output](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/4.png)
        
### Robot's Movement Control using PD Controller
  - calculate the error by finding the difference b/w camera_center and the x position of the   
    centroids of both lines
  - calculate the PD controller output based on the proportional, and derivative terms
  - track the lane using PD controller
    
The below image shows the block diagram of the PD Controller

![PD block diagram](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/9cc6b523596605adb87029ce7314993fbb587df0/src/slam_lane_tracking_pkg/Image/14-PD_Controller.png)

Following is the formula used to calculate the control effort by the controller

![PD Formula](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/9cc6b523596605adb87029ce7314993fbb587df0/src/slam_lane_tracking_pkg/Image/15-PD_Formula.png)

#### Formula for Desired Center
The desired center is calculated as the midpoint between the yellow and white lines. The formula is:

desired_center = Left Line X point – Right Line X point / 2

## 2. Data Collection and Conversion

### A. Robot's Pose data
While the robot is tracking the lane, its position and orientation data are continuously recorded in a .json file.

### B. Lane's Centroids data
While the robot is tracking the lane, the X centroid of left and right lanes along with the X centroid of the centre of the robot are continuously recorded in a .json file. 

**The Robot's Pose data along with the centroid's data is later utilized to mark the lanes as virtual obstacles for Autonomous Navigation.**

### C. Coordinates Conversion
1. In this project, the lanes centroids were recorded in image pixel coordinates, and were converted to the world coordinates using the following formula:
   
![image to world](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/2b719f8e66f678206f4f3e0d9a124188577439f5/src/slam_lane_tracking_pkg/Image/9-image_to_real_world.png)

3. The Lane's centroids along with the Robot's Pose data were converted to the map coordinate system from the world coordinate system using the following formula:
   
![world to map](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/9cc6b523596605adb87029ce7314993fbb587df0/src/slam_lane_tracking_pkg/Image/10-real_world_to_map.png)


## 3. Marking Lanes as Virtual Obstacles  
1. Robot starts tracking the lanes
2. Robot also maps the environment simultaneously
3. The Robot's pose data and centroids of both lanes are stored in a file

**Based on the above information the lanes are marked as virtual obstacles.**

![virtual obstacles](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/13-virtual_obstacles_flowchart.png)

In the figure below, it could be seen that the robot detects the centroids of both lanes and the centre of the robot while it is tracking the lane.

![lane centre](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/254ff36138c7d3f193de4add3ec26b8c27fec33e/src/slam_lane_tracking_pkg/Image/4-lane_with_centroids.png)

## 4. Map Processing
The map generated by the Gmapping node does not recognize the lane lines as obstacles. To address this, the "virtual_obstacles_node.py" script is used during pre-processing to mark the lanes as virtual obstacles.

The original map:

![Original Map](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/1-raw_map.png)


The updated/processed map:

![Original Map](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/2-processed_map.png)

## 5. Autonomous Navigation
Once the updated map with lanes marked as virtual obstacles is saved, then the robot can start the navigation within this known map.
For this the robot is provided with the start and the end points. Move base package is used for Autonomous Navigation.

Robot could be seen at the starting point in the known map
![Path Planning](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/6-Navigation.png)

Robot is given a goal point for Autonomous Navigation
![Navigation 1](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/7-Path_Planning.png)

Robot starts navigating towards the goal point

![Navigation 2](https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles/blob/d8ee77b808e20a995e924059716f959c14ea1595/src/slam_lane_tracking_pkg/Image/5-Navigation.png)


## Running the Project
Before running the project, make sure that you have calibrated the Turtlebot3's camera by following the corrosponding section from this link: [Turtlebot3 User Manual.](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#getting-started) Please follow chapter 8 (section 8.1 till section 8.4)

### For Lane Detection

Assuming the camera is perfectly calibrated, following are the steps required to run the project:
1. Start roscore on your PC (Terminal 1)
```bash
roscore
```
2. Connect with the Turtlebot3 from PC terminal by opening the ssh (Terminal 2)
```bash
ssh ubuntu@<turtlebot-ip>
```
Replace <turtlebot-ip> with the TurtleBot's IP address and enter the password when prompted.
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
3. Follow complete step 2 again on a new terminal but this time replace the bringup command with the following line (Terminal 3)
```bash
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```
4. Run camera intrinsic calibration on PC (Terminal 4)
```bash
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
```
5. Run camera extrinsic calibration on PC (Terminal 5)
```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
```
6. Run detect lane node on PC (Terminal 6): This step, will automatically save the centroids data in 
   a .json file.
```bash
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action
```
7. Run control lane node on PC (Terminal 7)
```bash
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
```
### For Lane Detection with SLAM and then Navigation
Follow steps 1 to 7 from the previous section and then run the following commands
8. Run SLAM node on PC (Terminal 8)
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
9. Run Robot pose data saving node on PC (Terminal 9)
```bash
rosrun slam_lane_tracking_pkg robot_pose_data.py
```
10. Once the entire arena has been explored, the map could be saved with the following command 
    (Terminal 10)
```bash
rosrun map_server map_saver -f ~/map
```
11. After saving the map, close all the previous terminals (except Terminal 1) and open a new terminal and run the following command to process the map to mark the lanes as virtual obstacles (Terminal 11)
```bash
rosrun slam_lane_tracking_pkg virtual_obstacles_node.py
```
12. Once the map is updated, the Turtlebot3 will be ready to navigate Autonomously.
    Run the following command on a new terminal (Terminal 12)
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
Remember to change the map_file path accordingly.

# Clone the repository
Create a folder in any directory before proceeding:
```bash
sudo apt-get update
```
```bash
sudo apt-get install git
```
```bash
git clone https://github.com/EhtishamAshraf/TB3_slam_with_virtual_obstacles.git
```
```bash
cd TB3_slam_with_virtual_obstacles
```
Run the below commands in root folder of the workspace
```bash
catkin_make 
```
```bash
source devel/setup.bash 
```
Navigate to the Scripts folder inside the slam_lane_tracking_pkg package and make the Python files executable by running the following command:
```bash
chmod +x *.py
```

This script is responsible for saving the robot's pose data.
```bash
lane_tracking.py
```
This script is responsible for marking lanes as virtual obstacles.
```bash
virtual_obstacles_node.py
```
This script is responsible for detecting lanes and saving the centroids data.
```bash
detect_lane.py
```

## Achievements
→ During this project, Turtlebot3’s Package (“detect lane” node) has been successfully modified to detect the centroids of 
  the lanes while doing the lane tracking.
  
→ The “detect lane” node has been modified to save the centroids data in a .json file.

## Limitations
→ **Narrow Lane Width:** The current implementation of navigation within the lane performs well for short distances and straight paths. However, during turns, the robot may struggle to maneuver effectively due to the limited width of the lane. This constraint can lead to difficulties in maintaining proper alignment and orientation, impacting the overall navigation performance.
