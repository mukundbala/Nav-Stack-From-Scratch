#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <fstream>
#include "bot_utils/bot_utils.h"
#include <signal.h>
#include "common.hpp"
#include <eigen3/Eigen/Dense>

#define NaN std::numeric_limits<double>::quiet_NaN()

ros::ServiceClient en_mtrs;
bot_utils::Pos3D hector_pos(NaN,NaN,NaN); //North West Up Frame
double hector_heading = NaN; //North West Up Frame
bot_utils::Pos3D current_target(NaN,NaN,NaN); //North West Up
bool rotate = false;

void disable_motors(int sig)
{
    ROS_INFO(" HMOVE : Disabling motors...");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv); 
}
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    current_target.x = msg->point.x;
    current_target.y = msg->point.y;
    current_target.z = msg->point.z;
}

void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    hector_pos.x = p.x;
    hector_pos.y = p.y;
    hector_pos.z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    hector_heading = atan2(siny_cosp, cosy_cosp);
}

void cbRotate(const std_msgs::Bool::ConstPtr &msg)
{
    rotate = msg->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    bool enable_move;
    bool verbose;
    double Kp_lin;
    double Ki_lin;
    double Kd_lin;
    double Kp_z;
    double Ki_z;
    double Kd_z;
    double yaw_rate;
    double max_lin_vel;
    double max_z_vel;
    double move_iter_rate;

    if (!nh.param("enable_move", enable_move, true))
    {
        ROS_WARN(" HMOVE : Param enable_move not found, set to true");
    }
        
    if (!nh.param("verbose_move", verbose, false))
    {
        ROS_WARN(" HMOVE : Param verbose_move not found, set to false");
    }
        
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
    {
        ROS_WARN(" HMOVE : Param Kp_lin not found, set to 1.0");
    }
        
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
    {
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    }
        
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
    {
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    }
        
    if (!nh.param("Kp_z", Kp_z, 1.0))
    {   
        ROS_WARN(" HMOVE : Param Kp_z not found, set to 1.0");
    }
        
    if (!nh.param("Ki_z", Ki_z, 0.0))
    {
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    }
        
    if (!nh.param("Kd_z", Kd_z, 0.0))
    {
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    }
        
    
    if (!nh.param("yaw_rate", yaw_rate, 0.5))
    {
        ROS_WARN(" HMOVE : Param yaw_rate not found, set to 0.5");
    }
        
    
    if (!nh.param("max_lin_vel", max_lin_vel, 2.0))
    {
        ROS_WARN(" HMOVE : Param max_lin_vel not found, set to 2");
    }
        
    
    if (!nh.param("max_z_vel", max_z_vel, 0.5))
    {
        ROS_WARN(" HMOVE : Param max_z_vel not found, set to 0.5");
    }
        
    
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
    {
        ROS_WARN(" HMOVE : Param move_iter_rate not found, set to 25");
    }
        
    
    // --------- Enable Motors ----------
    ROS_INFO(" HMOVE : Enabling motors...");

    en_mtrs = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");

    hector_uav_msgs::EnableMotors en_mtrs_srv;

    en_mtrs_srv.request.enable = true;
    
    if (en_mtrs.call(en_mtrs_srv))
    {
        ROS_INFO(" HMOVE : Motors enabled!");
    }
        
    else
    {
        ROS_WARN(" HMOVE : Cannot enable motors!");
    }

    signal(SIGINT, disable_motors);

    // --------- Subscribers ----------
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);
    ros::Subscriber sub_rotate = nh.subscribe("rotate", 1, &cbRotate);

    // --------- Publishers ----------
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // --------- Wait for Topics ----------
    ROS_INFO(" HMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && (std::isnan(current_target.x) || std::isnan(hector_pos.x))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce(); // update the topics

    // --------- Begin Controller ----------
    ROS_INFO(" HMOVE : ===== BEGIN =====");
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic
    double dt;
    double prev_time = ros::Time::now().toSec();

    double cmd_lin_vel_x = 0; //velocties must be in the body frame
    double cmd_lin_vel_y = 0;
    double cmd_lin_vel_z = 0;
    double cmd_ang_vel_a = 0;

    double x_error_prev = 0; //the world frame is north west up
    double x_error_cum = 0;
    double y_error_prev = 0;
    double y_error_cum = 0;
    double z_error_prev = 0;
    double z_error_cum = 0;

    double planar_saturation = sqrt(max_lin_vel);
    // main loop
    while (ros::ok() && nh.param("run", true))
    {
        // update all topics
        ros::spinOnce();

        dt = ros::Time::now().toSec() - prev_time;
        if (dt == 0) // ros doesn't tick the time fast enough
            continue;
        prev_time += dt;

        double error_x_world = current_target.x - hector_pos.x;
        double error_y_world = current_target.y - hector_pos.y;

        Eigen::Matrix2d R_MATRIX;
        R_MATRIX << std::cos(hector_heading),std::sin(hector_heading),
                    -std::sin(hector_heading),std::cos(hector_heading);
                    
        Eigen::Vector2d planar_error_world(error_x_world,error_y_world);
        Eigen::Vector2d planar_error_robot = R_MATRIX * planar_error_world;
        
        double error_x = planar_error_robot[0];
        double error_y = planar_error_robot[1];

        // double error_x = cos(-hector_heading) * (current_target.x - hector_pos.x) - sin(-hector_heading) * (current_target.y - hector_pos.y);
        // double error_y = sin(-hector_heading) * (current_target.x - hector_pos.x) + cos(-hector_heading) * (current_target.y - hector_pos.y);
        double error_z = current_target.z - hector_pos.z;

        double error_x_diff = error_x - x_error_prev;
        double error_y_diff = error_y - y_error_prev;
        double error_z_diff = error_z - z_error_prev;

        x_error_cum += (error_x * dt);
        y_error_cum += (error_y * dt);
        z_error_cum += (error_z * dt);

        double px = Kp_lin * error_x;
        double py = Kp_lin * error_y;
        double pz = Kp_z * error_z;

        double ix = Ki_lin * x_error_cum;
        double iy = Ki_lin * y_error_cum;
        double iz = Ki_z * z_error_cum;

        double dx = Kd_lin * (error_x_diff / dt);
        double dy = Kd_lin * (error_y_diff / dt);
        double dz = Kd_z * (error_z_diff / dt);

        double raw_x = px + ix + dx;
        double raw_y = py + iy + dy;
        double raw_z = pz + iz + dz;

        double acc_x = (raw_x - cmd_lin_vel_x) / dt;
        double est_x_vel = cmd_lin_vel_x + (acc_x * dt);

        double acc_y = (raw_y - cmd_lin_vel_y) / dt;
        double est_y_vel = cmd_lin_vel_y + (acc_y * dt);

        double acc_z = (raw_z - cmd_lin_vel_z) / dt;
        double est_z_vel = cmd_lin_vel_z + (acc_z * dt);

        cmd_lin_vel_x = est_x_vel > planar_saturation ? planar_saturation : est_x_vel;
        cmd_lin_vel_y = est_y_vel > planar_saturation ? planar_saturation : est_y_vel;
        cmd_lin_vel_z = est_z_vel > max_z_vel ? est_z_vel : est_z_vel;
        cmd_ang_vel_a = rotate ? yaw_rate : 0;

        x_error_prev = error_x;
        y_error_prev = error_y;
        z_error_prev = error_z;

        // publish speeds
        msg_cmd.linear.x = cmd_lin_vel_x;
        msg_cmd.linear.y = cmd_lin_vel_y;
        msg_cmd.linear.z = cmd_lin_vel_z;
        msg_cmd.angular.z = cmd_ang_vel_a;
        pub_cmd.publish(msg_cmd);

        //// IMPLEMENT /////
        
        // verbose
        if (verbose)
        {
            // ROS_INFO(" HMOVE : Target(%6.3f, %6.3f, %6.3f) FV(%6.3f) VX(%6.3f) VY(%6.3f) VZ(%7.3f)", target_x, target_y, target_z, cmd_lin_vel, cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);
        }

        // wait for rate
        rate.sleep();
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.linear.y = 0;
    msg_cmd.linear.z = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    // disable motors
    ROS_INFO(" HMOVE : Motors Disabled");
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv);

    ROS_INFO(" HMOVE : ===== END =====");

    return 0;
}