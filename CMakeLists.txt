cmake_minimum_required(VERSION 3.0.2)
project(lidar_cam_proj)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Compile as C++14
add_compile_options(-std=c++14)

# Find PCL's components, if found, specific env var set
find_package(PCL REQUIRED)

# Find OpenCV
find_package(OpenCV 3.4.2 REQUIRED)

# Set the directories for headers
include_directories(
  # Directories of PCL if PCL found
  ${PCL_INCLUDE_DIRS}
  # Directories of PCL if OpenCV found
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(lidar_cam_proj
  src/lidar_cam_proj.cpp
)

# Link the libraries
target_link_libraries(lidar_cam_proj ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})