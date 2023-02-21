cmake_minimum_required(VERSION 2.8.3)
project(ee4308_bringup)
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
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  tf2_turtle
    src/tf2_turtle.cpp)
target_link_libraries(tf2_turtle ${catkin_LIBRARIES})
