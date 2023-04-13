cmake_minimum_required(VERSION 3.0.2)
project(send_msg)

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



set(SRC_marker src/marker.cpp )

link_directories(/usr/lib/**)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(OpenCV 4 REQUIRED)

message(${OpenCV_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/home/sun/learn/learn_cmake/src/qr_pose_ros/src/send_msg/include)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        
        "/usr/lib/**"
        "/usr/local/lib/**"
        "/usr/include/**"
        "/usr/local/include/**"
)

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



add_library(marker SHARED ${SRC_marker})

add_executable(test1 src/test_opencv.cpp)
add_executable(demo src/demo_opencv.cpp)
add_executable(qrcode src/qrcode.cpp)
add_executable(qrcode1 src/qrcode1.cpp)
target_link_libraries( test1 zbar marker  ur_rtde::rtde ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries( demo zbar marker  ur_rtde::rtde ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries( qrcode zbar marker  ur_rtde::rtde ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries( qrcode1  marker Qt5::Core Qt5::Gui Qt5::Widgets ur_rtde::rtde ${OpenCV_LIBS} ${catkin_LIBRARIES})



