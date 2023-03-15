cmake_minimum_required(VERSION 3.0.2)
project(turtle_bringup)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  hmsgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    geometry_msgs 
    message_generation 
    nav_msgs 
    roscpp 
    rospy 
    sensor_msgs 
    std_msgs 
    tf2
    tf2_ros
    hmsgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  tf2_turtle
    src/tf2_turtle.cpp)
target_link_libraries(tf2_turtle ${catkin_LIBRARIES})