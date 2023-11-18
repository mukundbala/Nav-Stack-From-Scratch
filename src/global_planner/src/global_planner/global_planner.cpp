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

    //initialise robot variables
    robot_position_.setCoords(-500,-500);
    robot_index_.setIdx(-500,-500);

    //set trigger routine to false. Only planner service can turn it on
    anytime_routine_ = false;
    trigger_anytime_routine_ = false;
    minimum_refinement_ = false;

    //set up grid arrays
    mapdata.grid_inflation_.resize(mapdata.total_cells_);
    mapdata.grid_logodds_.resize(mapdata.total_cells_);

    //setup publisher
    refined_path_pub_ = nh_.advertise<nav_msgs::Path>("refined_path",1,true);

    //setup subscribers
    inflation_sub_ = nh_.subscribe("grid/mapinfo/inflation" , 1 , &GlobalPlanner::inflationCallback , this);
    lo_sub_ = nh_.subscribe("grid/mapinfo/logodds" , 1 , &GlobalPlanner::logoddsCallback , this);
    pose_sub_ = nh_.subscribe("pose" , 1 , &GlobalPlanner::poseCallback , this);

    ROS_INFO("[GlobalPlanner]: Global Planner prepared!");
}

//Pose Callback
void GlobalPlanner::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
{
    this->robot_pose_ = *pose_msg;
    this->robot_position_.x = robot_pose_.pose.position.x;
    this->robot_position_.y = robot_pose_.pose.position.y;
    robot_index_ = pos2idx(robot_position_);

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
            anytime_routine_ = false;
            primary_planner_ = std::make_shared<Djikstra>(cost_mode , mapdata);
        }

        else if (primary_planner_name_ == "astar")
        {
            std::string cost_mode = "f";
            anytime_routine_ = false;
            primary_planner_ = std::make_shared<Astar>(cost_mode , mapdata);
        }

        else if (primary_planner_name_ == "inflatedastar")
        {
            std::string cost_mode = "f";
            anytime_routine_ = false;
            primary_planner_ = std::make_shared<InflatedAstar>(cost_mode, mapdata);
        }

        else if (primary_planner_name_ == "arastar")
        {
            std::string cost_mode = "f";
            anytime_routine_ = true;
            primary_planner_ = std::make_shared<ARAstar>(cost_mode , mapdata);
        }
    }

    else
    {
        std::string cost_mode = "f";
        anytime_routine_ = false;
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
    /*
    Why lock_guard? 
    - The main planner contains the open list and current state of the tree, which are shared resources.
    - These resources are used both for generating the quick initial plan and for the subsequent refinement in the main loop.
    - Using lock_guard here ensures mutual exclusion, protecting these shared resources from concurrent access.
    - This prevents race conditions, where the refinement process in the main loop might access or modify the open list or the tree's state 
      while this service callback is also trying to access or modify them.
    - By locking the shared resources during the processing of a new path planning request, we maintain data integrity and ensure 
      that the actions of generating a new path and refining an existing path do not interfere with each other.
    */
    std::lock_guard<std::mutex> guard(planner_mutex_);
    
    //set these to false as we are now in a new planning iteration
    trigger_anytime_routine_ = false;
    minimum_refinement_ = false;

    ROS_INFO("[GlobalPlanner]: Firing Main Planner Service!");
    bot_utils::Pos2D start;
    bot_utils::Pos2D goal;

    start.setCoords(req.start_pos.x,req.start_pos.y);
    goal.setCoords(req.goal_pos.x , req.goal_pos.y);

    //update internal store of goals
    current_start_ = start;
    current_goal_ = goal;

    //update the internal store of paths
    current_path_ = primary_planner_->generatePath(start,goal,mapdata);
    
    trigger_anytime_routine_ = true;
    res.path = toPathMsg(current_path_);
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
    path_msg.header.frame_id = "world";
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

    while(ros::ok() && nh_.param("trigger_nodes" , true) && (robot_index_.i == -500 
                                                            ||mapdata.grid_inflation_.empty() 
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
        /*
        Why lock_guard? 
        - The main planner contains the open list and current state of the tree, which are shared resources.
        - These resources are used both for generating the quick initial plan and for the subsequent refinement in the main loop.
        - Using lock_guard here ensures mutual exclusion, protecting these shared resources from concurrent access.
        - This prevents race conditions, where the refinement process in the main loop might access or modify the open list or the tree's state 
          while this service callback is also trying to access or modify them.
        - By locking the shared resources during the processing of a new path planning request, we maintain data integrity and ensure 
        that the actions of generating a new path and refining an existing path do not interfere with each other.
        */
        //Flag that checks if the selected planner can do anytime routines,
        if (anytime_routine_)
        {
            std::lock_guard<std::mutex> guard(planner_mutex_);
            if (trigger_anytime_routine_)
            {
                auto refined_path = primary_planner_->RefinePathRoutine(mapdata);
                //If a valid refinement is received
                if (refined_path.size() > 1)
                {
                    auto processed_path = fallback_planner_.ProcessPath(refined_path,mapdata);
                    auto msg = toPathMsg(processed_path);
                    refined_path_pub_.publish(msg);
                    minimum_refinement_ = true;
                }
                else
                {
                    //e_ < 1 --> Stop refinement
                    if (!minimum_refinement_)
                    {
                        ROS_INFO("[Global Planner]: All Refinement Failed!. Using secondary planner!");
                        //a case where the path was not refined at all, we can use the secondary optimal planner
                        auto optimal_refined = secondary_planner_->generatePath(robot_position_,current_goal_,mapdata);
                        auto msg = toPathMsg(optimal_refined);
                        refined_path_pub_.publish(msg);
                    }
                    else
                    {
                        ROS_INFO("[Global Planner]: Refinement complete!");
                    }
                    trigger_anytime_routine_ = false;
                }
            }

        }
        
        spinrate.sleep();
    }
    
}

std::vector<bot_utils::Pos2D> GlobalPlanner::reconnectRefinedPath(std::vector<bot_utils::Pos2D> &raw_refined)
{
    //search the path to find the point that is closest to the robot and cut everything before that
    int target_index = -1;
    double dist = 1e5;
    for (int i = 0 ; i < raw_refined.size() ; ++i)
    {
        auto dist_to_robot = bot_utils::dist_euc(raw_refined[i],robot_position_);
        if (dist < dist_to_robot)
        {
            target_index = i;
            dist_to_robot = dist;
        }
    }

    //copy over all the points from the target index!
    std::vector<bot_utils::Pos2D> reconnected_path;
    reconnected_path = std::vector<bot_utils::Pos2D>(raw_refined.begin() + target_index,raw_refined.end());

    return reconnected_path; 

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