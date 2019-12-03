cmake_minimum_required(VERSION 2.8.3)
project(project_fetch)

SET(CMAKE_CXX_FLAGS "-g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)

set(HEADER_FILES $(YOUR_DIRECTORY)/Stepper.H)
add_library(stepper src/libs/Stepper.cpp src/libs/Stepper.h)
target_link_libraries(stepper pigpio)

find_package(Threads)

add_executable(car_control src/car.cpp)
# add_executable(target_tracker src/target_tracker.cpp)
# target_include_directories(car ${pigpio_INCLUDE_DIRS})
target_link_libraries(car_control ${catkin_LIBRARIES} pigpio stepper ${CMAKE_THREAD_LIBS_INIT})
# target_link_libraries(target_tracker ${catkin_LIBRARIES})
# add_dependencies(car ${catkin_EXPORTED_TARGETS})
target_include_directories(car_control PRIVATE $(YOUR_DIRECTORY))
# add_executable(stepper src/car.cpp)
# target_link_libraries(stepper ${catkin_LIBRARIES} pigpio)

