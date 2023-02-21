cmake_minimum_required(VERSION 3.0.2)
project(commander)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  bot_utils
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tmsgs
)

catkin_package(
 CATKIN_DEPENDS 
  bot_utils 
  geometry_msgs 
  nav_msgs 
  roscpp 
  rospy 
  std_msgs
  tmsgs
)

###########
## Build ##
###########

include_directories(
  include/commander
  include/local_planner
  include/pid_controller
  ${catkin_INCLUDE_DIRS}
)

add_executable(commander_node
  src/local_planner/local_planner.cpp
  src/pid_controller/pid_controller.cpp
  src/commander/commander.cpp
  src/commander/commander_node.cpp)


target_link_libraries(commander_node
${catkin_LIBRARIES}
)