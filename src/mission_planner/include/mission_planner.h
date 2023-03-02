#ifndef TBOT__MISSION_PLANNER_H
#define TBOT__MISSION_PLANNER_H

#include "ros/ros.h"
#include "bot_utils/bot_utils.h"
#include "tmsgs/Goal.h"
#include "tmsgs/UpdateTurtleGoal.h"

#include "geometry_msgs/PoseStamped.h"
#include "xmlrpcpp/XmlRpc.h"
#include <vector>
#include <deque>
class MissionPlanner
{
private:
    int num_goals;
    std::deque<bot_utils::Pos2D> goals_;
    double goal_radius_;

    geometry_msgs::PoseStamped robot_pose_;
    bot_utils::Pos2D robot_position_;

    ros::Publisher goal_pub_;

    ros::Subscriber pose_sub_;
    // ros::Subscriber update_goal_sub_;
    ros::ServiceServer update_goal_server_;
    ros::NodeHandle nh_;

    double rate_;

public:
    MissionPlanner(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);
    bool updateGoalService(tmsgs::UpdateTurtleGoal::Request &req, tmsgs::UpdateTurtleGoal::Response &res);
    void run();
};

#endif //TBOT__MISSION_PLANNER_H