cmake_minimum_required(VERSION 2.8.3)
project(opencv_testing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  stereo_msgs
  std_msgs
  cv_bridge
)

catkin_package()

include_directories(
  include_directories
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(disparity opencv.cpp)
target_link_libraries(disparity ${catkin_LIBRARIES} ${OpenCV_LIBS})
