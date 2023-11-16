#ifndef TBOT__NAVIGATOR_H
#define TBOT__NAVIGATOR_H

#include "ros/ros.h"

#include "bot_utils/bot_utils.h"
#include "bot_utils/map_data.h"

#include "tmsgs/Goal.h"
#include "tmsgs/TurtlePath.h"
#include "tmsgs/UpdateTurtleGoal.h"
#include "tmsgs/PlanMainPath.h"
#include "tmsgs/FindFallbackPosition.h"

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>

#include <vector>

/*
* @info
  Navigator class serves as the interface between the Commander
  Mission Planner and Global Planner. It implements the overall control logic
  for the navigation stack. 
  
  This ensures that:
    - Latest goals are received from the Mission Planner and updated
    - Planning is triggered appropriately, based on replan and plan triggers
    - Processes replan triggers from the Commander based on trajectory quality
*/

class Navigator
{
private:
    //pose from motion filter and the associated map index
    geometry_msgs::PoseStamped robot_pose_;
    bot_utils::Pos2D robot_position_;
    bot_utils::Index robot_index_;

    //Next best position if robot is on inflated cell
    bot_utils::Pos2D backup_robot_position_;
    
    //Navigation state of the robot
    bool robot_status_;
    bool backup_mode_;

    //All the map data and meta data. This to to make it easy to share with planner without copying so much stuff
    bot_utils::MapData mapdata;
    
    //goals
    bot_utils::Pos2D current_goal_;
    bool goal_status_;
    int current_goal_id_;

    //planning triggers
    bool trigger_plan;
    bool trigger_replan; //this replan is purely done by the commander

    //path from Global Planner
    std::vector<bot_utils::Pos2D> path_;
    int path_id_;

    //Path message for communication with other ros nodes
    tmsgs::TurtlePath path_comm_msg_;

    //Path message for RVIZ visualization
    nav_msgs::Path path_msg_;

    /*
    Subscribers
    * pose_sub: Subscribe to robot pose from motion filter
    * inflation_sub_ and lo_sub: Subscribes to log odds map from occupancy grid
    * goal_sub_: Subscribes to new goal from mission planner
    * replan_sub_: Subcribes to a replan state from 
    */
   
    ros::Subscriber pose_sub_;
    ros::Subscriber inflation_sub_;
    ros::Subscriber lo_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber replan_sub_;

    /*
    * Connects to a server in Mission Planner to update a goal position.
      This update happens when a goal position appears on an inflated
      cell due to drift in odometry. Typicall observed when using the 
      motion filter.
    */
    ros::ServiceClient update_goal_client_;
    ros::ServiceClient main_planner_client_;
    ros::ServiceClient fallback_planner_client_;

    //publisher
    ros::Publisher path_pub_;
    ros::Publisher path_comm_pub_;

    //nodehandle
    ros::NodeHandle nh_;
    double rate_;
    const double EPS_ = 1e-6; //for float comparisons

    //verbosity
    bool verbose_;
public:
    //Class constructor for global planner
    Navigator(ros::NodeHandle &nh);
    
    void run();

    //callbacks
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);

    void inflationCallback(const std_msgs::Int32MultiArrayConstPtr &inflation);

    void logoddsCallback(const std_msgs::Int32MultiArrayConstPtr &inflation);

    void goalCallback(const tmsgs::GoalConstPtr &goal);

    void replanCallback(const std_msgs::BoolConstPtr &trigger);
    
    //Request Goal Update
    void request_goal_update(bot_utils::Pos2D &goal_to_update);

    //Service Call wrapper for main path. 
    std::vector<bot_utils::Pos2D> request_path(bot_utils::Pos2D &start_pos, bot_utils::Pos2D &goal_pos);

    //Service call wrapper for fallback point
    bot_utils::Pos2D request_backup_position(bot_utils::Pos2D &bad_pos);

    //Param loading utility
    bool loadParams();

    //Utility to convert to nav_msgs/Path
    void writeToPathMsg();

    /*
    Map Utilities
    * flatten: Converts 2D grid coordinates to 1D key to access 1D occupancy grid array
    * oob: Out of Bounds (oob) check to check if a an index is in the map
    * pos2idx: Converts cartesian position to 2D grid coordinates
    * idx2pos: Converts 2D grid position to cartesian position
    * testpos: Check if oob, and if cell is free
    */
    bot_utils::Index pos2idx(bot_utils::Pos2D &pos);

    bot_utils::Pos2D idx2pos(bot_utils::Index &idx);

    int flatten(bot_utils::Index &idx);

    bool oob(bot_utils::Index &idx);

    bool testPos(bot_utils::Pos2D idx);
};

#endif //TBOT__NAVIGATOR_H