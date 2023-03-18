cmake_minimum_required(VERSION 3.0.2)
project(drone_commander)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  bot_utils
  geometry_msgs
  hector_uav_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tmsgs
  hmsgs
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)


catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    geometry_msgs 
    std_srvs
    hector_uav_msgs
    message_generation 
    nav_msgs 
    roscpp 
    rospy 
    sensor_msgs 
    std_msgs 
    tf2
    bot_utils
    tmsgs
    hmsgs
)


include_directories(
  include/drone_commander
  include/trajectory_generator
  include/velocity_controller
  ${catkin_INCLUDE_DIRS}
)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(drone_commander_node
  src/drone_commander/drone_commander.cpp
  src/drone_commander/mission_states.cpp
  src/trajectory_generator/trajectory_generator.cpp
  src/velocity_controller/velocity_controller.cpp
  src/drone_commander/drone_commander_node.cpp
)
add_dependencies(drone_commander_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_commander_node ${catkin_LIBRARIES}${Eigen3_LIBRARIES})

