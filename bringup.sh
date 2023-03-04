export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

# get world and task
source params.sh

# source the project
source devel/setup.bash

# get the world specific parameters as environment variables
source `rospack find turtle_bringup`/worlds/`echo $WORLD`/`echo $WORLD`.sh

# bring up gazebo and build all models
roslaunch turtle_bringup bringup_`echo $TASK`.launch
