cmake_minimum_required(VERSION 2.8.3)
project(mobility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation 
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
  tf
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES rover.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES sensor_msgs std_msgs )

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  mobility src/mobility.cpp src/DDSAController.cpp
)

add_dependencies(mobility ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  mobility
  ${catkin_LIBRARIES}
)

