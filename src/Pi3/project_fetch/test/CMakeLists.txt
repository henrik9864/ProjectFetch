cmake_minimum_required (VERSION 2.8.9)
project (test)

#set(pigpio_DIR /lib/pigpio/util)
set(CMAKE_MODULE_PATH /lib/pigpio/util)

#---
find_package(pigpio REQUIRED)
#---

add_executable(Test test.cpp)
target_include_directories(Test PUBLIC ${pigpio_INCLUDE_DIRS})
target_link_libraries(Test ${pigpio_LIBRARIES})
