#ifndef TBOT__MISSION_PLANNER_H
#define TBOT__MISSION_PLANNER_H

#include "ros/ros.h"
#include "bot_utils/bot_utils.h"
#include "tmsgs/Goal.h"
#include "geometry_msgs/PoseStamped.h"
#include "XmlRpc.h"
#include <vector>
class MissionPlanner
{
private:
    int num_goals;
    std::vector<bot_utils::Pos2D> goals_;
    int current_goal_id; //stores the id of the current goal 
    double goal_radius_;

    geometry_msgs::PoseStamped robot_pose_;
    bot_utils::Pos2D robot_position_;

    ros::Publisher goal_pub_;

    ros::Subscriber pose_sub_;
    ros::Subscriber update_goal_sub_;

    ros::NodeHandle nh_;

    double rate_;

public:
    MissionPlanner(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);
    void updateGoalCallback(const tmsgs::Goal::ConstPtr &updated_goal);
    void run();
};

#endif //TBOT__MISSION_PLANNER_H