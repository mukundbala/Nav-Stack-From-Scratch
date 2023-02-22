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
    robot_status_ = true; //robot is okay

    //set up grid arrays
    mapdata.grid_inflation_.resize(mapdata.total_cells_);
    mapdata.grid_logodds_.resize(mapdata.total_cells_);

    //setup goals
    current_goal_.setCoords(-1000 , -1000); //again some very unlikely value
    current_goal_idx_ = -1;
    prev_goal_idx_ = -1;
    goal_status_ = true;

    //setup planning
    trigger_plan = false;
    path_.clear();
    path_msg_.poses.clear();
    path_msg_.header.frame_id = "world";

    //setup subscribers
    pose_sub_ = nh_.subscribe("pose" , 1 , &GlobalPlanner::poseCallback , this);
    inflation_sub_ = nh_.subscribe("grid/mapinfo/inflation" , 1 , &GlobalPlanner::inflationCallback , this);
    lo_sub_ = nh_.subscribe("grid/mapinfo/logodds" , 1 , &GlobalPlanner::logoddsCallback , this);
    goal_sub_ = nh_.subscribe("goal" , 1 , &GlobalPlanner::goalCallback , this);
    replan_sub_ = nh_.subscribe("trigger_replan" , 1 , &GlobalPlanner::replanCallback , this);
    
    //setup publishers
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
        this->current_goal_.setCoords(goal->goal_position.x , goal->goal_position.y);
        // this->current_goal_idx_ = goal->idx;
        this->path_.clear();
        this->path_msg_.poses.clear();
        trigger_plan = true;
        ROS_INFO("[GlobalPlanner]: New goal received!");
    }
    else
    {
        trigger_plan = false; //if we dont get a new goal, we always set trigger plan to false
    }
}

void GlobalPlanner::replanCallback(const std_msgs::BoolConstPtr &replan) //do this shit after commander is done
{
    // bool trigger = replan->data; //this is the trigger from the trajectory planner
    // if (trigger)
    // {
    //     this->trigger_plan = true;
    //     this->path_.clear();
    //     this->path_msg_.poses.clear();
    // }
}

void GlobalPlanner::run()
{
    ros::Rate spinrate(rate_);

    ROS_INFO("[GlobalPlanner]: Waiting for topics");

    while(ros::ok() && nh_.param("trigger_nodes" , true) && nh_.param("trigger_mp",false) && (robot_position_.x == -500 
                                                            || mapdata.grid_inflation_.empty() 
                                                            || mapdata.grid_logodds_.empty() 
                                                            || current_goal_.x == -1000)
                                                            || !trigger_plan) //safeguard, make sure everythign that needs to load has loaded
    {
        spinrate.sleep();
        ros::spinOnce();
        ROS_INFO("[GlobalPlanner]: Waiting for topics");
    }

    Astar main_planner(mapdata);
    Djikstra emergency_planner(mapdata);
    ROS_INFO("[GlobalPlanner]: Starting Global Planner!");
    while(ros::ok() && nh_.param("trigger_nodes" , true))
    {
        ros::spinOnce(); //process a round of callbacks
        robot_status_ = testPos(robot_position_);
        goal_status_ = testPos(current_goal_);
        ROS_INFO_STREAM("GOAL STATUS" << goal_status_);
        ROS_INFO_STREAM("ROBOT_STATUS" << robot_status_);
        if (trigger_plan) 
        {
            //A planning trigger will ALWAYS clear our path
            //planning is triggered for a 3 key reasons
            //1. Mission Planner has given us a new goal without any updates from Global Planner
            //2. There was a bad goal and we updated the goal
            //3. The trajectory was bad and we need a new trajectory
            //4. Trajectory can be bad because the targets were perhaps lying on an inflated/occupied cell
            if (path_.empty())
            {

                if (robot_status_ && goal_status_) //if both are okay. This likely means that some part of the traj was bad
                {
                    path_ = main_planner.plan(robot_position_ , current_goal_,mapdata);
                    ROS_INFO("[GlobalPlanner]: Robot and Goal Good. Path Planned!");
                }
                
                else if (!robot_status_ && goal_status_) //this is the case where its a bad robot status, but the goal is still ok
                {
                    bot_utils::Pos2D backup_start = emergency_planner.plan(robot_position_ , mapdata); //find a new nearest start
                    path_ = main_planner.plan(backup_start , current_goal_, mapdata); //plan from the backup to the goal
                    ROS_INFO("[GlobalPlanner]: Robot in bad position. Planning path out now!");
                }

                else if (robot_status_ && !goal_status_) //this means that our goal is in an occupied area
                {
                    bot_utils::Pos2D backup_goal = emergency_planner.plan(current_goal_ , mapdata);
                    //however, we cannot plan right away because only the mission planner has the right to update goals
                    tmsgs::Goal new_goal;
                    new_goal.action = 2;
                    new_goal.goal_position.x = backup_goal.x;
                    new_goal.goal_position.y = backup_goal.y;
                    update_goal_pub_.publish(new_goal); //we wait for the new goal to come back to us
                    ROS_INFO("[GlobalPlanner]: Bad goal given. Waiting for Mission planner");
                    spinrate.sleep();
                    continue;
                }

                else if (!robot_status_ && !goal_status_)
                {
                    bot_utils::Pos2D backup_start = emergency_planner.plan(robot_position_ , mapdata); //find a new nearest start
                    path_ = main_planner.plan(backup_start , current_goal_, mapdata); //plan from the backup to the goal
                    ROS_INFO("[GlobalPlanner]: Bad goal and Bad robot position. Getting robot out");
                }
                
                ROS_INFO("[GlobalPlanner]: Printing New Path");
                if (!path_.empty())
                {
                    for (auto &pos : path_)
                    {
                        ROS_INFO_STREAM("(" << pos.x << "," << pos.y <<")");
                        geometry_msgs::PoseStamped pt;
                        pt.pose.position.x = pos.x;
                        pt.pose.position.y = pos.y;
                        path_msg_.poses.push_back(pt); //write our path to goal
                    }
                    path_pub_.publish(path_msg_);
                    trigger_plan = false; //we set this to false
                }
                else
                {
                    ROS_INFO("No path found after replanning!");
                }
            }
            else //path is not empty
            {
                path_pub_.publish(path_msg_);
                trigger_plan = false; //we set this to false
            }
            
        }

        else //no trigger_plan. Nothing is wrong, no changes received
        {
            if (path_.empty())
            {
                ROS_INFO("WHAT");
            }
            else
            {
                path_pub_.publish(path_msg_);
            }

        }
        // if (!path_.empty())
        // {
        //     path_pub_.publish(path_msg_);
        // }
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