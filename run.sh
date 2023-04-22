#!/bin/bash
export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

# get world and task
source params.sh

# source the project
source devel/setup.bash

# get the world specific parameters as environment variables
source `rospack find robot_bringup`/worlds/`echo $WORLD`/`echo $WORLD`.sh

# reset the gazebo
echo "Resetting Gazebo. Wait 1s for Gazebo to complete reset."
rosservice call /gazebo/reset_simulation
sleep 1s # allow gazebo to reset so as not to mess with the time.
# roslaunch code for reset sim. Put here for reference
#    <node pkg="rosservice" type="rosservice" name="reset_simulation" args="call --wait /gazebo/reset_simulation {}" /> 

# bring up gazebo and build all models
echo "Begin..."
roslaunch robot_bringup run_robots.launch
if [ $TASK == "mode1" ];then
echo "Stopping Turtlebot3 by publishing to /turtle/cmd_vel"
rostopic pub -1 /turtle/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
elif [ $TASK == "mode2" ];then
echo "Stopping Hector by setting disabling motors!"
rostopic pub -1 /hector/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
rosservice call /hector/enable_motors "enable: false" 
else
echo "Stopping Turtlebot3 by publishing to /turtle/cmd_vel"
rostopic pub -1 /turtle/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
echo "Stopping Hector by disabling motors!"
rostopic pub -1 /hector/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
rosservice call /hector/enable_motors "enable: false" 
fi