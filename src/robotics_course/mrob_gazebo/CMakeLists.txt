cmake_minimum_required(VERSION 3.0.2)
project(mrob_gazebo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  turtlebot3_gazebo
  pointcloud_to_laserscan
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrob_gazebo
 CATKIN_DEPENDS turtlebot3_gazebo pointcloud_to_laserscan
#  DEPENDS system_lib
)
