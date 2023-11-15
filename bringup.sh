#!/bin/bash
export NSFSWS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $NSFSWS`

# get world and task
source params.sh

# source the project
source devel/setup.bash

# get the world specific parameters as environment variables
source `rospack find robot_bringup`/worlds/`echo $WORLD`/`echo $WORLD`.sh

# bring up gazebo and build all models
roslaunch robot_bringup bringup_robots.launch
