#ifndef HBOT__VELOCITY_CONTROLLER_H
#define HBOT__VELOCITY_CONTROLLER_H

#include "ros/ros.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "hector_uav_msgs/EnableMotors.h"
#include "std_msgs/Bool.h"
#include "bot_utils/bot_utils.h"


#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <fstream>
#include <signal.h> 
#include <eigen3/Eigen/Dense>

class VelocityController
{

};
#endif //HBOT__VELOCITY_CONTROLLER_H