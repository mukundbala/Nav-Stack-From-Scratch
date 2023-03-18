#!/bin/bash
export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

# get world and task
source params.sh

# source the project
source devel/setup.bash

# get the world specific parameters as environment variables
source `rospack find turtle_bringup`/worlds/`echo $WORLD`/`echo $WORLD`.sh

# reset the gazebo
echo "Resetting Gazebo. Wait 1s for Gazebo to complete reset."
rosservice call /gazebo/reset_simulation
sleep 1s # allow gazebo to reset so as not to mess with the time.
# roslaunch code for reset sim. Put here for reference
#    <node pkg="rosservice" type="rosservice" name="reset_simulation" args="call --wait /gazebo/reset_simulation {}" /> 

# bring up gazebo and build all models
echo "Begin..."
roslaunch turtle_bringup soloflight.launch