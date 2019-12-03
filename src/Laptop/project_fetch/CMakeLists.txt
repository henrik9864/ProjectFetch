cmake_minimum_required(VERSION 2.8.3)
project(project_fetch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  sensor_msgs
  message_generation
  message_filters
  geometry_msgs
)

find_package(Qt5 5.7.0 REQUIRED COMPONENTS
  Core
)

set(CMAKE_AUTOMOC ON)

add_message_files(
  FILES
  CameraImageInfo.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include_directories
  ${catkin_INCLUDE_DIRS}
)

add_executable(synchronizer synchronizer.cpp)
target_link_libraries(synchronizer ${catkin_LIBRARIES})

add_executable(target_tracker target_tracker.cpp)
target_link_libraries(target_tracker ${catkin_LIBRARIES} Qt5::Core)

QT5_USE_MODULES(target_tracker Widgets Core)