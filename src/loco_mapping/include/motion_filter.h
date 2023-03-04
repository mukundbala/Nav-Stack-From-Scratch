#ifndef TBOT__MOTION_FILTER_H
#define TBOT__MOTION_FILTER_H
#include "ros/ros.h"
#include "bot_utils/bot_utils.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

class MotionFilter
{
private:
    //vars that store subscriptions
    double imu_angular_vel_; //store of angular velcity from imu
    double imu_linear_acc_; //store of imus linear acceleration
    double wheel_l_; //store of left wheel's rotation. Note that this is unbounded and cumulative
    double wheel_r_; //store of right wheel's rotation. Note that this is unbounded and cumulative

    //robot specific variables
    double axle_track_;
    double wheel_radius_;
    double weight_odom_v_;
    double weight_odom_w_;
    double weight_imu_v_;
    double weight_imu_w_;
    double straight_thresh_;
    bool use_internal_odom_;

    //node spin rate
    double rate_;

    //environment variables
    double initial_x_;
    double initial_y_;

    //logging
    bool verbose_;

    //nodehandle
    ros::NodeHandle nh_;

    //publishers
    ros::Publisher pose_pub_;
    ros::Publisher speed_pub_;//publish purely the magnitude

    //subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber wheel_sub_;
    ros::Subscriber imu_sub_;
    nav_msgs::Odometry odom_msg_;

    //robot position , pos and heading
    geometry_msgs::PoseStamped robot_pose_mf_;
    geometry_msgs::PoseStamped robot_pose_odom_;
    bot_utils::Pos2D robot_position_;
    double robot_heading_;

public:
    MotionFilter(ros::NodeHandle& nh);
    void loadParams();
    void callbackIMU(const sensor_msgs::Imu::ConstPtr &msg);
    void callbackWheels(const sensor_msgs::JointState::ConstPtr &msg);
    void callbackOdom(const nav_msgs::Odometry::ConstPtr &msg);

    void run();
};




#endif //TBOT__MOTION_FILTER_H