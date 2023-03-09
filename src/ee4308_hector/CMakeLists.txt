cmake_minimum_required(VERSION 2.8.3)
project(ee4308_hector)
add_compile_options(-std=c++17)

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
  bot_utils
  tmsgs
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

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
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  hector_main
    src/common.cpp
    src/hector_trajectory.cpp
    src/main.cpp)
add_dependencies(hector_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_main ${catkin_LIBRARIES} ${OpenCV_LIBS}${Eigen3_LIBRARIES})


add_executable(
  hector_move
    src/common.cpp
    src/hector_trajectory.cpp
    src/move.cpp)
add_dependencies(hector_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_move ${catkin_LIBRARIES} ${OpenCV_LIBS}${Eigen3_LIBRARIES})

add_executable(
  hector_motion
    src/common.cpp
    src/hector_trajectory.cpp
    src/motion.cpp)
add_dependencies(hector_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_motion ${catkin_LIBRARIES} ${OpenCV_LIBS}${Eigen3_LIBRARIES})

