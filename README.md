# Virtual Obstacle Lane Navigation with Autonomous Mapping using real Turtlebot3
This repository contains using "Turtlebot3 package" and also features a custom ROS package, "slam_lane_tracking_pkg", combined in a way to enable Turtlebot3 to autonomously navigate an autorace environment by tracking lanes and simultaneously mapping the surroundings. The package equips Turtlebot3 with the ability to track lanes (yellow and white lines) using camera sensor data .

Once the map is created, the "robot’s pose data" and "lines centroid's data" is used to designate the detected lanes as virtual obstacles, ensuring they are avoided during autonomous navigation. 
To check the simulation results please refer to the following two repositories: [autonomous lane tracking simulation.](https://github.com/EhtishamAshraf/turtlebot3_lane_tracking.git) and
[virtual lane marking and navigation simulation.](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation.git)

### Demo Video
You can watch the demo video of Turtlebot3 tracking the lane in real arena by clicking on the below image
[![Watch the video](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation/blob/main/src/slam_lane_tracking_pkg/Image/video_image.png)](https://youtu.be/-JGLHBf8EOU)

You can watch the demo video of virtual lane marking and Autonomous Navigation by a Turtlebot3 in real world arena by clicking on the below image
[![Watch the video](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation/blob/main/src/slam_lane_tracking_pkg/Image/navigation.png)](https://youtu.be/vERA8F4Mlvc)
