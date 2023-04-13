cmake_minimum_required(VERSION 3.0.2)
project(charuco)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)







find_package(Eigen3 REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(OpenCV 4 REQUIRED)

message(${OpenCV_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES send_msg
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)





add_executable(charuco src/charuco.cpp)

target_link_libraries( charuco   ur_rtde::rtde ${OpenCV_LIBS} ${catkin_LIBRARIES})
