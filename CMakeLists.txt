cmake_minimum_required(VERSION 3.0.2)
project(lidar_cam_fuse)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Compile as C++14
add_compile_options(-std=c++14)

# Find PCL's components, if found, specific env vars are set
find_package(PCL REQUIRED)

# The same with others
find_package(OpenCV 3.4.2 REQUIRED)

# For catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  message_filters
)

# 
catkin_package(
 CATKIN_DEPENDS message_runtime roscpp sensor_msgs pcl_conversions cv_bridge pcl_ros
)

# Set the directories for headers of packages
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(fuse
  src/main.cpp
  src/ros_bag_load.cpp
  # src/lidar_cam_fuse.cpp
  # src/common.cpp
)

# Link the libraries to the executable
target_link_libraries(fuse ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} stdc++fs)