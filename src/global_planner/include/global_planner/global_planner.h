#ifndef TBOT__GLOBAL_PLANNER_H
#define TBOT__GLOBAL_PLANNER_H

#include "ros/ros.h"
#include "grid_planner_core.h"
#include "astar.h"
#include "djikstra.h"
#include "inflatedastar.h"
#include "arastar.h"

#include "bot_utils/bot_utils.h"
#include "bot_utils/map_data.h"

#include "tmsgs/PlanMainPath.h"
#include "tmsgs/FindFallbackPosition.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <memory>
#include <mutex>
/*
GlobalPlanner class has the following roles:
- Subscribe to pose
- Subscribe to mapinfo/inflation grid
- Subscribe to mapinfo/logodds grid
- Subscribe to incoming goal
- Check status of goal
*/

class GlobalPlanner
{
private:

    //robot state
    geometry_msgs::PoseStamped robot_pose_;
    bot_utils::Pos2D robot_position_;
    bot_utils::Index robot_index_;

    //Current Start and End goals
    bot_utils::Pos2D current_start_;
    bot_utils::Pos2D current_goal_;

    //current path created
    std::vector<bot_utils::Pos2D> current_path_;

    //All the map data and meta data. This to to make it easy to share with planner without copying so much stuff
    bot_utils::MapData mapdata;
    
    //List of planners
    const std::vector<std::string> available_primary_planners = {"djikstra","astar","inflatedastar","arastar"};
    const std::vector<std::string> available_secondary_planners = {"djikstra","astar"};
    
    //Primary Planner (Anytime Planners)
    std::string primary_planner_name_;
    std::shared_ptr<GridPlannerCore> primary_planner_;

    //Secondary Planner (Always optimal)
    std::string secondary_planner_name_;
    std::shared_ptr<GridPlannerCore> secondary_planner_;

    //Fallback Planner
    Djikstra fallback_planner_;

    /*
    anytime_routine_: This is true if the chosen planner is an anytime planner
    trigger_anytime_routine_: This will be triggered after the main planning sequence
    minimum_refinement_: Will be true if 
    */
    bool anytime_routine_;
    bool trigger_anytime_routine_;
    bool minimum_refinement_;

    //A mutex to protect the shared planner resource
    std::mutex planner_mutex_;

    //refine path publisher
    ros::Publisher refined_path_pub_;

    //subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber inflation_sub_;
    ros::Subscriber lo_sub_;

    //planning servers
    ros::ServiceServer main_planner_server_;
    ros::ServiceServer fallback_planner_server_;

    //nodehandle
    ros::NodeHandle nh_;
    const double EPS_ = 1e-6; //for float comparisons

    //verbosity
    bool verbose_;
    double rate_;
public:
    //Class constructor for global planner
    GlobalPlanner(ros::NodeHandle &nh);
    void run();

    //callbacks

    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);
    
    void inflationCallback(const std_msgs::Int32MultiArrayConstPtr &inflation);

    void logoddsCallback(const std_msgs::Int32MultiArrayConstPtr &inflation);

    //services
    bool planMainPathServiceCallback(tmsgs::PlanMainPath::Request &req , tmsgs::PlanMainPath::Response &res);
    bool findFallbackServiceCallback(tmsgs::FindFallbackPosition::Request &Req , tmsgs::FindFallbackPosition::Response &res);
    
    //load params
    bool loadParams();

    //setup planners polymorphically
    void setupPlanner();

    //helper stuff
    bot_utils::Index pos2idx(bot_utils::Pos2D &pos);

    bot_utils::Pos2D idx2pos(bot_utils::Index &idx);

    int flatten(bot_utils::Index &idx);

    bool oob(bot_utils::Index &idx);

    bool testPos(bot_utils::Pos2D idx);

    nav_msgs::Path toPathMsg(std::vector<bot_utils::Pos2D> &path);

    std::vector<bot_utils::Pos2D> reconnectRefinedPath(std::vector<bot_utils::Pos2D> &raw_refined);

};

#endif