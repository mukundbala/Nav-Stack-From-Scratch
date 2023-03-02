#ifndef TBOT__COMMANDER_H
#define TBOT__COMMANDER_H

#include "ros/ros.h"
#include "pid_params.h"
#include "bot_utils/bot_utils.h"
#include "bot_utils/map_data.h"
#include "pid_controller.h"
#include "local_planner.h"

#include <nav_msgs/Path.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Int32MultiArray.h>
#include "tmsgs/TriggerPlannerReplan.h"
#include "tmsgs/TurtlePath.h"
#include <vector>
#include <deque>
#include <tuple>

class Commander
{
private:
    //robot information
    geometry_msgs::PoseStamped robot_pose_; 
    bot_utils::Pos2D robot_position_;
    double robot_heading_;

    //map information
    bot_utils::MapData mapdata; //occupancy map data
    
    //path information
    std::vector<bot_utils::Pos2D> path_; //path array
    int curr_path_id; //id of the path

    //trajectory and target
    std::deque<bot_utils::Pos2D> trajectory_; 
    bot_utils::Pos2D current_target_;
    bool current_traj_state_;

    //triggers
    bool generate_trajectory_;

    //PID Controller Params
    PIDParams pid_params_;

    //Local Planner Params
    double target_dt_;
    double average_speed_;
    std::string traj_type_;

    //Commander Params
    double rate_;
    bool enable_commander_;
    double close_enough_;

    //subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber inflation_sub_;
    ros::Subscriber lo_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber speed_sub_;

    //publishers
    ros::Publisher target_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher cmd_vel_pub_;

    //client
    ros::ServiceClient replan_client_;

    //published messages
    geometry_msgs::PointStamped target_msg_;
    nav_msgs::Path traj_msg_;
    std_msgs::Bool replan_msg_;
    geometry_msgs::Twist cmd_vel_msg_;

    //nodehandle
    ros::NodeHandle nh_;

public:
    Commander(ros::NodeHandle &nh);

    //callbacks
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);

    void inflationCallback(const std_msgs::Int32MultiArrayConstPtr &inflation);

    void logoddsCallback(const std_msgs::Int32MultiArrayConstPtr &lo);

    void pathCallback(const tmsgs::TurtlePath &path);

    bool checkTrajectory();

    bool checkDist();

    void run();

    //there is a fair bit of params to load, so we load step by step
    bool loadMapParams(); //load map params

    bool loadPIDParams(); //load pid specific params

    bool loadTrajParams(); //load trajectory specific params

    bool loadCommanderParams(); //load commander specific params

    //helper stuff for grids
    bot_utils::Index pos2idx(bot_utils::Pos2D &pos);

    bool oob(bot_utils::Index &idx);

    int flatten(bot_utils::Index &idx);

    bool checkCell(bot_utils::Index &idx);

    //writing message
    void writeTrajMsg();



};


#endif //TBOT__COMMANDER_H