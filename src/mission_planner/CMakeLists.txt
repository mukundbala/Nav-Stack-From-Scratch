cmake_minimum_required(VERSION 3.0.2)
project(mission_planner)
add_compile_options(-std=c++17)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  bot_utils
  tmsgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    geometry_msgs 
    roscpp 
    rospy
    bot_utils
    tmsgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_planner_node 
src/mission_planner.cpp
src/mission_planner_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(
  mission_planner_node
  ${catkin_LIBRARIES}
)
