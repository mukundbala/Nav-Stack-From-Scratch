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
    robot_position_.setCoords(-500,-500); //-500 is a very unlikely value
    robot_index_.setIdx(-500,-500); //-500 is a very unlikely value

    //set up grid arrays
    mapdata.grid_inflation_.resize(mapdata.total_cells_);
    mapdata.grid_logodds_.resize(mapdata.total_cells_);

    //setup goals
    current_goal_.setCoords(-1000 , -1000); //again some very unlikely value
    current_goal_idx_ = -1;
    prev_goal_idx_ = -1;
    current_goal_state_ = GoalState::GOOD;

    //setup planning
    trigger_plan = true;
    path_.clear();
    path_msg_.poses.clear();
    path_msg_.header.frame_id = "world";

    //setup subscribers
    pose_sub_ = nh_.subscribe("pose" , 1 , &GlobalPlanner::poseCallback , this);
    inflation_sub_ = nh_.subscribe("grid/mapinfo/inflation" , 1 , &GlobalPlanner::inflationCallback , this);
    lo_sub_ = nh_.subscribe("grid/mapinfo/logodds" , 1 , &GlobalPlanner::logoddsCallback , this);
    goal_sub_ = nh_.subscribe("goal" , 1 , &GlobalPlanner::goalCallback , this);

    //setup subscribers
    update_goal_pub_ = nh_.advertise<tmsgs::Goal>("update_goal" , 1 , true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1 , true);

    ROS_INFO("[GlobalPlanner]: Global Planner prepared!");
}

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

void GlobalPlanner::goalCallback(const tmsgs::GoalConstPtr &goal)
{
    //check if the arriving goal is the same as our current goal
    if (fabs(goal->goal_position.x - current_goal_.x) > EPS_ && fabs(goal->goal_position.y - current_goal_.y) > EPS_)
    {
        current_goal_.setCoords(goal->goal_position.x , goal->goal_position.y);
        current_goal_idx_ = goal->idx;
        trigger_plan = true; //we trigger planning here for this goal
    }
}

// void GlobalPlanner::replanCallback(const std_msgs::BoolConstPtr &replan) //do this shit after commander is done
// {
//     this->trigger_plan = replan->data;
//     this->path_.clear(); //clear the path
//     this->path_msg_.poses.clear();
// }

void GlobalPlanner::run()
{
    ros::Rate spinrate(rate_);

    ROS_INFO("[GlobalPlanner]: Waiting for topics");

    while(ros::ok() && nh_.param("trigger_nodes" , true) && (robot_position_.x == -500 
                                                            || mapdata.grid_inflation_.empty() 
                                                            || mapdata.grid_logodds_.empty() 
                                                            || current_goal_.x == -1000)) //safeguard, make sure everythign that needs to load has loaded
    {
        spinrate.sleep();
        ros::spinOnce();
        ROS_INFO("[GlobalPlanner]: Waiting for topics");
    }

    Astar main_planner(mapdata);
    // Djikstra emergency_planner(mapdata); //TODO @karthi

    while(ros::ok() && nh_.param("trigger_nodes" , true))
    {
        ros::spinOnce(); //process a round of callbacks
        bool goal_status = testGoal();
        current_goal_state_ = goal_status ? GoalState::GOOD : GoalState::BAD;

        if (current_goal_state_ == GoalState::GOOD)
        {
            if (!trigger_plan)
            {
                if (!path_.empty())
                {
                    path_pub_.publish(path_msg_);
                }
                else
                {
                    trigger_plan = true;
                }
            }
            else //trigger_plan is true
            {
                ROS_INFO("[GlobalPlanner]: Generating a path!");
                ROS_INFO_STREAM("[GlobalPlanner]: From (" << robot_position_.x << "," << robot_position_.y <<") to (" << current_goal_.x << "," << current_goal_.y << ")");
                ROS_INFO("[GlobalPlanner]:Using A*");

                path_ = main_planner.plan(robot_position_ , current_goal_, mapdata);

                ROS_INFO("PRINTING PATH");
                for (auto &p : path_)
                {
                    ROS_INFO_STREAM("(" << p.x << "," << p.y << ")");
                }
                ROS_INFO("PRINTING PATH DONE");

                if (path_.size() == 0) //we shall handle the case of path_size == 1 downstream in the trajectory planner
                {
                    //we might have moved into an obstacle area
                    current_goal_state_ = GoalState::BAD;
                    ROS_INFO("[GlobalPlanner]: Bad goal");
                    continue;
                }
                else
                {
                    path_msg_.poses.clear();
                    for (bot_utils::Pos2D &pos : path_)
                    {
                        geometry_msgs::PoseStamped pse;
                        pse.pose.position.x = pos.x;
                        pse.pose.position.y = pos.y;
                        path_msg_.poses.push_back(pse);
                    }
                    path_pub_.publish(path_msg_);
                    trigger_plan = false; //finished planning, set the trigger to false
                }
            }
        }

        else if (current_goal_state_ == GoalState::BAD)
        {
            //TODO @Karthi
            // trigger_plan = true;
            // // geometry_msgs::Point new_goal = emergency_planner(current_goal_, robot_position_ , mapdata);
            // tmsgs::Goal new_goal_msg;
            // new_goal_msg.goal_position = new_goal;
            // new_goal_msg.idx = current_goal_idx_;
            // update_goal_pub_.publish(new_goal_msg);
        }
        spinrate.sleep();
    }
}


int GlobalPlanner::flatten(bot_utils::Index &idx)
{
    return idx.i * mapdata.map_size_.j + idx.j;
}

bool GlobalPlanner::oob(bot_utils::Index &idx)
{
    return (idx.i <=0 || idx.i >= mapdata.map_size_.i || idx.j < 0 && idx.j >= mapdata.map_size_.j);
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

bool GlobalPlanner::testGoal()
{
    bot_utils::Index idx = pos2idx(current_goal_);
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

    if (!nh_.param("gp_rate" , this->rate_ , 25.0))
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
    return status;
}