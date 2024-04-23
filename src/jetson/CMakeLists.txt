cmake_minimum_required(VERSION 3.0.2)  # Updated minimum required version
project(jetson)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
  thrusters
)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES jetson
  CATKIN_DEPENDS roscpp std_msgs thrusters
#  DEPENDS system_lib
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(JHPWMPCA9685 src/JHPWMPCA9685.cpp include/JHPWMPCA9685.h)
add_executable(controller src/controller.cpp )
target_link_libraries(controller JHPWMPCA9685 ${catkin_LIBRARIES})
add_dependencies(controller thrusters_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

