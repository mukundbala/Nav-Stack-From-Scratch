#include "mission_planner.h"

MissionPlanner::MissionPlanner(ros::NodeHandle &nh)
{
    nh_ = nh;
    XmlRpc::XmlRpcValue goal_loader;
    nh_.getParam("goals",goal_loader);

    if (goal_loader.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for (int i = 0 ; i < goal_loader.size() ; ++i)
        {
            XmlRpc::XmlRpcValue ob = goal_loader[i];

            bot_utils::Pos2D my_goal;
            my_goal.x = ob[0];
            my_goal.y = ob[1];
            goals_.push_back(my_goal);
        }
    }
    ROS_WARN_COND(goals_.size() <= 0 , "[MissionPlanner]: No goals given!");

    if (!nh_.param("mp_rate" , this->rate_ , 25.0))
    {
        ROS_WARN("[MissionPlanner]: Param mp_rate not found. Defaulting to 25.0");
    }

    if (!nh_.param("goal_radius" , this->goal_radius_, 0.1))
    {
        ROS_WARN("[MissionPlanner]: Param goal_radius_ not found. Defaulting to 0.10");
    }

    robot_position_.x = -500;
    robot_position_.y = -500;

    goal_pub_ = nh_.advertise<tmsgs::Goal>("goal",1);

    pose_sub_ = nh_.subscribe("pose" , 1 , &MissionPlanner::poseCallback , this);
    // update_goal_sub_ = nh_.subscribe("update_goal" , 1 , &MissionPlanner::updateGoalCallback , this);
    update_goal_server_ = nh_.advertiseService("update_t_goal",&MissionPlanner::updateGoalService,this);

    ROS_INFO("###GOALS###");
    int cnt = 0;
    for (auto &gl : goals_)
    {
        ROS_INFO_STREAM("Goal " << cnt << ": (" << gl.x << "," << gl.y<<")");
        cnt++;
    }
    ROS_INFO("###GOALS###");
    ROS_INFO_STREAM("[Mission Planner]: Mission Planner Prepared!");
}

void MissionPlanner::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
{
    robot_pose_ = *pose_msg;
    robot_position_.setCoords(robot_pose_.pose.position.x , robot_pose_.pose.position.y);
}

bool MissionPlanner::updateGoalService(tmsgs::UpdateTurtleGoal::Request &req , tmsgs::UpdateTurtleGoal::Response &res)
{
    bot_utils::Pos2D updated_goal(req.to_update.goal_position.x , req.to_update.goal_position.y);
    int action = req.to_update.action;
    if (action == 1)
    {
        ROS_INFO("[MissionPlanner]: Goal added to goals!");
        //this is a temporary goal
        goals_.push_front(updated_goal);
    }

    else if (action == 2)
    {
        ROS_INFO("[MissionPlanner]: Goal replaced!");
        goals_.at(0) = updated_goal; //this is a goal to replace the current goal
    }
    res.response = true;
    return true;
}

void MissionPlanner::run()
{
    ros::Rate spinrate(rate_);
    ROS_INFO("[Mission Planner]: Waiting for topics");
    
    while (ros::ok() && nh_.param("trigger_nodes" , true) && goals_.empty() && robot_position_.x == -500);
    {
        spinrate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("[Mission Planner]: Starting Mission Planner");
    while(ros::ok() && nh_.param("trigger_nodes" , true))
    {
        ros::spinOnce();
        if (goals_.empty())
        {
            ROS_INFO_STREAM("[MissionPlanner]: Final Goal Reached!");
            break;
        }

        double dist_to_goal = bot_utils::dist_euc(robot_position_,goals_.front());
        if (dist_to_goal < goal_radius_)
        {
            ROS_INFO_STREAM("[Mission Planner]: Goal " << "Reached!");
            goals_.pop_front();
            if (goals_.empty())
            {
                ROS_INFO_STREAM("[MissionPlanner]: Final Goal Reached!");
                break;
            }
        }
        tmsgs::Goal goal;
        goal.goal_position.x = goals_.front().x;
        goal.goal_position.y = goals_.front().y;

        goal_pub_.publish(goal);
            
        spinrate.sleep();
    }
    ROS_INFO("[Mission Planner]: Shutting down all nodes!");
    nh_.setParam("trigger_nodes" , false);
}