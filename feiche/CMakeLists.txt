cmake_minimum_required(VERSION 2.8.3)
project(feiche)
SET(CMAKE_BUILD_TYPE "Realease")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES feiche
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}./include)
FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization)
catkin_package()
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_executable(feiche src/lidar_main.cpp)
target_link_libraries(feiche ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${GSLLLIB})
