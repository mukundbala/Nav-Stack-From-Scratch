#ifndef TBOT__GLOBAL_PLANNER_H
#define TBOT__GLOBAL_PLANNER_H

#include "ros/ros.h"
#include "grid_planner_core.h"
#include "astar.h"
#include "thetastar.h"
#include "djikstra.h"
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

    //subscribers
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

};

#endif