cmake_minimum_required(VERSION 2.8.3)
project(ee4308_hector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  std_srvs
  hector_uav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
)
find_package(OpenCV REQUIRED)

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
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  hector_main
    src/common.cpp
    src/main.cpp)
target_link_libraries(hector_main ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(
  hector_move
    src/common.cpp
    src/move.cpp)
target_link_libraries(hector_move ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(
  hector_motion
    src/common.cpp
    src/motion.cpp)
target_link_libraries(hector_motion ${catkin_LIBRARIES} ${OpenCV_LIBS})