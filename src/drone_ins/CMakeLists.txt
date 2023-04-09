cmake_minimum_required(VERSION 2.8.3)
project(drone_ins)
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
  ${catkin_INCLUDE_DIRS}
)


add_executable(
  drone_ins
    src/motion.cpp)
add_dependencies(drone_ins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_ins ${catkin_LIBRARIES} ${OpenCV_LIBS}${Eigen3_LIBRARIES})

