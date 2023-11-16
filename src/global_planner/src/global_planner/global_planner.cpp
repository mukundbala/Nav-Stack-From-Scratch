#include "global_planner.h"

GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh)
{
    nh_ = nh;
    //loading parameters from param server
    bool param_status = loadParams();
    ROS_WARN_COND(!param_status , "[GlobalPlanner]: Parameters have not been loaded correctly!");
    //initalize map variables
    mapdata.origin_ = mapdata.pos_min_;
    mapdata.map_size_.i = std::round((mapdata.pos_max_.x - mapdata.pos_min_.x) / mapdata.cell_size_);
    mapdata.map_size_.j = std::round((mapdata.pos_max_.y - mapdata.pos_min_.y) / mapdata.cell_size_);
    mapdata.total_cells_ = mapdata.map_size_.i * mapdata.map_size_.j;

    //set up grid arrays
    mapdata.grid_inflation_.resize(mapdata.total_cells_);
    mapdata.grid_logodds_.resize(mapdata.total_cells_);

    //setup subscribers
    inflation_sub_ = nh_.subscribe("grid/mapinfo/inflation" , 1 , &GlobalPlanner::inflationCallback , this);
    lo_sub_ = nh_.subscribe("grid/mapinfo/logodds" , 1 , &GlobalPlanner::logoddsCallback , this);

    ROS_INFO("[GlobalPlanner]: Global Planner prepared!");
}


void GlobalPlanner::inflationCallback(const std_msgs::Int32MultiArrayConstPtr &inflation)
{
    mapdata.grid_inflation_ = inflation->data;
}

void GlobalPlanner::logoddsCallback(const std_msgs::Int32MultiArrayConstPtr &lo)
{
    mapdata.grid_logodds_ = lo->data;
}

void GlobalPlanner::setupPlanner()
{
    auto it_primary = std::find(available_primary_planners.begin(),available_primary_planners.end(),primary_planner_name_);
    auto it_secondary = std::find(available_secondary_planners.begin(),available_secondary_planners.begin(),secondary_planner_name_);

    if (it_primary != available_primary_planners.end())
    {
        if (primary_planner_name_ == "djikstra")
        {
            std::string cost_mode = "g";
            primary_planner_ = std::make_shared<Djikstra>(cost_mode , mapdata);
        }

        else if (primary_planner_name_ == "astar")
        {
            std::string cost_mode = "f";
            primary_planner_ = std::make_shared<Astar>(cost_mode , mapdata);
        }

        else if (primary_planner_name_ == "inflatedastar")
        {
            std::string cost_mode = "f";
            primary_planner_ = std::make_shared<InflatedAstar>(cost_mode, mapdata);
        }

        else if (primary_planner_name_ == "arastar")
        {
            std::string cost_mode = "f";
            //primary_planner_ = std::make_shared<ARAStar>(cost_mode , mapdata);
        }
    }

    else
    {
        std::string cost_mode = "f";
        primary_planner_ = std::make_shared<Astar>(cost_mode , mapdata); 
    }

    if (it_secondary != available_secondary_planners.end())
    {
        if (secondary_planner_name_ == "djikstra")
        {
            std::string cost_mode = "g";
            secondary_planner_ = std::make_shared<Djikstra>(cost_mode , mapdata);
        }

        else if (secondary_planner_name_ == "astar")
        {
            std::string cost_mode = "f";
            secondary_planner_ = std::make_shared<Astar>(cost_mode,mapdata);
        }
    }

    std::string fallback_cost_mode = "g";
    fallback_planner_.prepPlanner(fallback_cost_mode,mapdata);
}

bool GlobalPlanner::planMainPathServiceCallback(tmsgs::PlanMainPath::Request &req , tmsgs::PlanMainPath::Response &res)
{
    ROS_INFO("[GlobalPlanner]: Firing Main Planner Service!");
    bot_utils::Pos2D start;
    bot_utils::Pos2D goal;

    start.setCoords(req.start_pos.x,req.start_pos.y);
    goal.setCoords(req.goal_pos.x , req.goal_pos.y);

    auto path = primary_planner_->generatePath(start,goal,mapdata);

    res.path = toPathMsg(path);
    ROS_INFO("[GlobalPlanner]: Main Planner Service Succeeded!");
    return true;

}

bool GlobalPlanner::findFallbackServiceCallback(tmsgs::FindFallbackPosition::Request &req , tmsgs::FindFallbackPosition::Response &res)
{
    ROS_INFO("[GlobalPlanner]: Firing Fallback Planner Service!");
    bot_utils::Pos2D bad_point;
    bad_point.setCoords(req.bad_position.x,req.bad_position.y);

    bot_utils::Pos2D better_point = fallback_planner_.find_better_point(bad_point,mapdata);

    geometry_msgs::Point better_point_msg;
    better_point_msg.x = better_point.x;
    better_point_msg.y = better_point.y;

    res.fallback_position = better_point_msg;
    ROS_INFO("[GlobalPlanner]: Fallback Planner Service Succeeded!");
    return true;
}

nav_msgs::Path GlobalPlanner::toPathMsg(std::vector<bot_utils::Pos2D>& path)
{
    nav_msgs::Path path_msg;
    
    for (auto& pt : path)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = pt.x;
        pose_msg.pose.position.y = pt.y;
        path_msg.poses.push_back(pose_msg);
    }

    return path_msg;
}


void GlobalPlanner::run()
{
    ROS_INFO("[GlobalPlanner]: Waiting for topics");
    ros::Rate spinrate(rate_);

    while(ros::ok() && nh_.param("trigger_nodes" , true) && (mapdata.grid_inflation_.empty() 
                                                            || mapdata.grid_logodds_.empty())) 
                                                        
    {
        ros::spinOnce();
        spinrate.sleep();
    }

    ROS_INFO("[GlobalPlanner]: All topics received!");

    setupPlanner();

    //setup servers
    main_planner_server_ = nh_.advertiseService("plan_main_path",&GlobalPlanner::planMainPathServiceCallback,this);
    fallback_planner_server_ = nh_.advertiseService("find_fallback_position",&GlobalPlanner::findFallbackServiceCallback,this);
    ROS_INFO("[GlobalPlanner]: Services Active!");
    
    while (ros::ok() && nh_.param("trigger_nodes" , true))
    {
        ros::spinOnce();
        spinrate.sleep();
    }
    
}


int GlobalPlanner::flatten(bot_utils::Index &idx)
{
    return idx.i * mapdata.map_size_.j + idx.j;
}

bool GlobalPlanner::oob(bot_utils::Index &idx)
{
    return (idx.i < 0 || idx.i >= mapdata.map_size_.i || idx.j < 0 && idx.j >= mapdata.map_size_.j);
}

bot_utils::Index GlobalPlanner::pos2idx(bot_utils::Pos2D &pos)
{
    int i = round((pos.x - mapdata.origin_.x) / mapdata.cell_size_);
    int j = round((pos.y - mapdata.origin_.y) / mapdata.cell_size_);
    return bot_utils::Index(i,j);
}

bot_utils::Pos2D GlobalPlanner::idx2pos(bot_utils::Index &idx)
{
    double x = idx.i * mapdata.cell_size_ + mapdata.origin_.x;
    double y = idx.j * mapdata.cell_size_ + mapdata.origin_.y;
    return bot_utils::Pos2D(x,y);
}

bool GlobalPlanner::testPos(bot_utils::Pos2D pos)
{
    bot_utils::Index idx = pos2idx(pos);
    int key = flatten(idx);

    if (oob(idx))
    {
        return false; //test fail
    }

    if (mapdata.grid_inflation_.at(key) > 0)
    {
        return false; //in map, but on inflated
    }
    else if (mapdata.grid_logodds_.at(key) > mapdata.lo_thresh_)
    {
        return false; //in map, not inflated but log odds occupied
    }
    else
    {
        return true; //in map, not inflated not lo occupied
    }
}


bool GlobalPlanner::loadParams()
{
    bool status = true;
    if (!nh_.param("cell_size" , this->mapdata.cell_size_ , 0.05))
    {
        ROS_WARN("[GlobalPlanner]: Param cell_size not found, defaulting to 0.05");
        status = false;
    }

    if (!nh_.param("log_odds_thresh" , this->mapdata.lo_thresh_ , 10))
    {
        ROS_WARN("[GlobalPlanner]: Param log_odds_thresh not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("log_odds_cap" , this->mapdata.lo_cap_ , 20))
    {
        ROS_WARN("[GlobalPlanner]: Param log_odds_cap not found, defaulting to 20");
        status = false;
    }

    if (!nh_.param("min_x" , this->mapdata.pos_min_.x , -10.0))
    {
        ROS_WARN("[GlobalPlanner]: Param min_x not found, defaulting to -10");
        status = false;
    }

    if (!nh_.param("min_y" , this->mapdata.pos_min_.y , -10.0))
    {
        ROS_WARN("[GlobalPlanner]: Param min_y not found, defaulting to -10");
        status = false;
    }

    if (!nh_.param("max_x" , this->mapdata.pos_max_.x , 10.0))
    {
        ROS_WARN("[GlobalPlanner]: Param max_x not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("max_y" , this->mapdata.pos_max_.y , 10.0))
    {
        ROS_WARN("[GlobalPlanner]: Param max_y not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("planner_verbose" , this->verbose_ , false))
    {
        ROS_WARN("[GlobalPlanner]: Param verbose_planner not found, defaulting to false");
        status = false;
    }

    if (!nh_.param<std::string>("primary_planner" , this->primary_planner_name_ , "astar"))
    {
        ROS_WARN("[GlobalPlanner]: Param primary_planner not found, defaulting to astar");
    }

    if (!nh_.param<std::string>("secondary_planner" , this->secondary_planner_name_ , "astar"))
    {
        ROS_WARN("[GlobalPlanner]: Param primary_planner not found, defaulting to astar");
    }

    if (!nh_.param("gp_rate" , this->rate_ , 25.0))
    {
        ROS_WARN("[GlobalPlanner]: Param gp_rate not found, defaulting to 25.0");
    }

    return status;
}