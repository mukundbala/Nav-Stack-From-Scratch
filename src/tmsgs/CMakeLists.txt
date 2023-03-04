cmake_minimum_required(VERSION 3.0.2)
project(tmsgs)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  nav_msgs
)
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Goal.msg
  TurtlePath.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UpdateTurtleGoal.srv
  TriggerPlannerReplan.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
 CATKIN_DEPENDS 
 message_runtime 
 std_msgs 
 nav_msgs
 geometry_msgs
)

