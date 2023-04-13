cmake_minimum_required(VERSION 3.0.0)
project(test_mark VERSION 0.1.0)
set(SRC_marker src/marker.cpp )

link_directories(/usr/lib/**)

find_package(Eigen3 REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(OpenCV 4 REQUIRED)

message(${OpenCV_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/home/sun/learn/learn_cmake/src/QR_pose/QR_POSE/include/)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        
        "/usr/lib/**"
        "/usr/local/lib/**"
        "/usr/include/**"
        "/usr/local/include/**"
)
add_library(marker SHARED ${SRC_marker})

add_executable(test src/test_opencv.cpp)
add_executable(demo src/demo_opencv.cpp)
target_link_libraries( test zbar marker  ur_rtde::rtde ${OpenCV_LIBS} )
target_link_libraries( demo zbar marker  ur_rtde::rtde ${OpenCV_LIBS} )


