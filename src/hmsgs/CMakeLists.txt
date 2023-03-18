cmake_minimum_required(VERSION 3.0.2)
project(hmsgs)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Goal.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS 
 message_runtime 
 std_msgs 
 geometry_msgs
)