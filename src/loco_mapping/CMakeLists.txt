cmake_minimum_required(VERSION 3.0.2)
project(loco_mapping)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  bot_utils
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS 
  sensor_msgs 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  roscpp 
  rospy
  message_generation 
  message_runtime
  bot_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(motion_filter_node
  src/motion_filter.cpp 
  src/motion_filter_node.cpp)

add_executable(occupancy_grid_node 
  src/occupancy_grid.cpp 
  src/occupancy_grid_node.cpp)

target_link_libraries(
  motion_filter_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  occupancy_grid_node 
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_loco_mapping.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
