cmake_minimum_required(VERSION 3.0.0)
project(test_mark VERSION 0.1.0)
set(SRC_marker src/marker.cpp )

link_directories(/usr/lib/**)

find_package(Eigen3 REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)



message(${OpenCV_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/home/sun/learn/learn_cmake/src/qrcode/include/)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        
        "/usr/lib/**"
        "/usr/local/lib/**"
        "/usr/include/**"
        "/usr/local/include/opencv4/**"
        "/usr/include/opencv2/aruco/**"
      
)
add_library(marker SHARED ${SRC_marker})

link_libraries("/usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0")

add_executable(qrcode src/qrcode.cpp)
add_executable(qrcode1 src/qrcode1.cpp)
# add_executable(aruco src/test_aruco.cpp)
# add_executable(realsense_qr src/realsense_qr.cpp)
# add_executable(test_realsense src/test_realsense.cpp)

target_link_libraries(qrcode marker Qt5::Core Qt5::Gui Qt5::Widgets ur_rtde::rtde ${OpenCV_LIBS} )
target_link_libraries(qrcode1 marker zbar Qt5::Core Qt5::Gui Qt5::Widgets ur_rtde::rtde ${OpenCV_LIBS} )
# target_link_libraries( aruco marker ${OpenCV_LIBS} )
# target_link_libraries( realsense_qr marker ${OpenCV_LIBS}  -lrealsense2)
# target_link_libraries( test_realsense marker ${OpenCV_LIBS}  -lrealsense2)




