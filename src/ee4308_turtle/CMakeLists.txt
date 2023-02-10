cmake_minimum_required(VERSION 2.8.3)
project(ee4308_turtle)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    nav_msgs
    sensor_msgs
    std_msgs
    geometry_msgs
    roscpp
    rospy
)


catkin_package(
    CATKIN_DEPENDS 
        message_runtime 
        message_generation 
        sensor_msgs 
        std_msgs 
        geometry_msgs 
        nav_msgs 
        roscpp 
        rospy 
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  turtle_main 
    src/main.cpp
    src/planner.cpp
    src/grid.cpp
    src/common.cpp
    src/los.cpp
    src/trajectory.cpp)
target_link_libraries(turtle_main ${catkin_LIBRARIES})


add_executable(
  turtle_move
    src/common.cpp
    src/move.cpp)
target_link_libraries(turtle_move ${catkin_LIBRARIES})

add_executable(
  turtle_motion
    src/common.cpp
    src/motion.cpp)
target_link_libraries(turtle_motion ${catkin_LIBRARIES})
#add_dependencies(motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})