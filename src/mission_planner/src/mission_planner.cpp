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
    update_goal_sub_ = nh_.subscribe("update_goal" , 1 , &MissionPlanner::updateGoalCallback , this);

    ROS_INFO("###GOALS###");
    int cnt = 0;
    for (auto &gl : goals_)
    {
        ROS_INFO_STREAM("Goal " << cnt << ": (" << gl.x << "," << gl.y<<")");
        cnt++;
    }
    ROS_INFO("###GOALS###");
    ROS_INFO_STREAM("[Mission Planner]: Mission Planner ready!");
}

void MissionPlanner::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
{
    robot_pose_ = *pose_msg;
    robot_position_.setCoords(robot_pose_.pose.position.x , robot_pose_.pose.position.y);
}

void MissionPlanner::updateGoalCallback(const tmsgs::Goal::ConstPtr &updated_goal)
{
    // int idx_to_replace = updated_goal -> idx;
    bot_utils::Pos2D new_goal(updated_goal->goal_position.x , updated_goal->goal_position.y);
    if (updated_goal->action == 1)
    {
        //this is a temporary goal
        goals_.push_front(new_goal);
    }
    else if (updated_goal->action == 2)
    {
        goals_.at(0) = new_goal; //this is a goal to replace the current goal
    }
    // goals_.at(idx_to_replace) = new_goal;
}

void MissionPlanner::run()
{
    ros::Rate spinrate(rate_);
    ROS_INFO("[Mission Planner]: Waiting for topics");
    
    while (ros::ok() && goals_.empty() && nh_.param("trigger_nodes" , true) && robot_position_.x == -500 && nh_.param("trigger_mp" , false));
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
        nh_.setParam("trigger_gp" , true);
    }
    ROS_INFO("[Mission Planner]: Shutting down all nodes!");
    nh_.setParam("trigger_nodes" , false);
}