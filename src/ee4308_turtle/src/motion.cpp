#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "common.hpp"

double imu_ang_vel = -10; // unlikely to be spinning at -10 at the start
double imu_lin_acc = 0;
double wheel_l = 10;
double wheel_r = 10; // init as 10 bcos both are unlikely to be exactly 10 (both can start at non-zero if u reset the sim)
nav_msgs::Odometry msg_odom;

void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_ang_vel = msg->angular_velocity.z;
    imu_lin_acc = msg->linear_acceleration.x;
}

void cbWheels(const sensor_msgs::JointState::ConstPtr &msg)
{
    wheel_l = msg->position[1]; 
    wheel_r = msg->position[0]; 
}

void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_motion");
    ros::NodeHandle nh;

    // Parse ROS parameters
    bool use_internal_odom;
    bool verbose;
    double initial_x;
    double initial_y;
    if (!nh.param("use_internal_odom", use_internal_odom, true))
    {
        ROS_WARN(" TMOVE : Param use_internal_odom not found, set to true");
    }
        
    if (!nh.param("verbose_motion", verbose, false))
    {
        ROS_WARN(" TMOVE : Param verbose_motion not found, set to false");
    }
        
    if (!nh.param("initial_x", initial_x, 0.0))
    {
        ROS_WARN(" TMOVE : Param initial_x not found, set to 0.0");
    }

    if (!nh.param("initial_y", initial_y, 0.0))
    {
        ROS_WARN(" TMOVE : Param initial_y not found, set to 0.0");
    }
        
    // Publisher
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

    // Prepare published message
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = "world"; //for rviz

    if (use_internal_odom)
    { // subscribes to odom topic --> is the exact simulated position in gazebo; when used in real life, is derived from wheel encoders (no imu).
        // Subscriber
        ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom);

        // initialise rate
        ros::Rate rate(25);

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && msg_odom.header.seq == 0) // dependent on odom
        {
            rate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // begin loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            // write to published message
            robot_pose.pose = msg_odom.pose.pose;
            // uncomment the following when running on real hardware, bcos odom always starts at 0.
            // pose_rbt.pose.position.x += initial_x;
            // pose_rbt.pose.position.y += initial_y;


            // publish pose
            pub_pose.publish(robot_pose);

            if (verbose)
            {
                // get ang_rbt from quaternion
                auto &q = robot_pose.pose.orientation;
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)  FVel(%6.3f)  AVel(%6.3f)",
                         robot_pose.pose.position.x, robot_pose.pose.position.y, atan2(siny_cosp, cosy_cosp),
                         msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z);
            }

            rate.sleep();
        }
    }
    else
    {
        // Parse additional ROS parameters
        Position robot_position(initial_x, initial_y);
        double wheel_radius = 0.033;
        double axle_track = 0.16;
        double weight_odom_v = 0.5;
        double weight_odom_w = 0.5;
        double weight_imu_v = 0.5;
        double weight_imu_w = 0.5;
        double straight_thresh = 0.05;
        double motion_iter_rate = 50.0;
        if (!nh.param("initial_x", robot_position.x, 0.0))
        {
            ROS_WARN(" TMOVE : Param initial_x not found, set to 0.0");
        }
            
        if (!nh.param("initial_y", robot_position.y, 0.0))
        {
            ROS_WARN(" TMOVE : Param initial_y not found, set to 0.0");
        }

        if (!nh.param("wheel_radius", wheel_radius, 0.033))
        {
            ROS_WARN(" TMOVE : Param wheel_radius not found, set to 0.033");
        }

        if (!nh.param("axle_track", axle_track, 0.16))
        {
            ROS_WARN(" TMOVE : Param axle_track not found, set to 0.16");
        }
            
        if (!nh.param("weight_odom_v", weight_odom_v, 0.5))
        {
            ROS_WARN(" TMOVE : Param weight_odom_v not found, set to 0.5");
        }
            
        if (!nh.param("weight_odom_w", weight_odom_w, 0.5))
        {
            ROS_WARN(" TMOVE : Param weight_odom_w not found, set to 0.5");
        }
            
        if (!nh.param("straight_thresh", straight_thresh, 0.05))
        {
            ROS_WARN(" TMOVE : Param straight_thresh not found, set to 0.05");
        }
            
        if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
        {
            ROS_WARN(" TMOVE : Param motion_iter_rate not found, set to 50");
        }
        weight_imu_v = 1 - weight_odom_v;
        weight_imu_w = 1 - weight_odom_w;
        // Subscribers
        ros::Subscriber sub_wheels = nh.subscribe("joint_states", 1, &cbWheels);
        ros::Subscriber sub_imu = nh.subscribe("imu", 1, &cbImu);
        //ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom); //for error calculation only
        // initialise rate
        ros::Rate rate(motion_iter_rate); // higher rate for better estimation

        // initialise message for publishing
        robot_pose.pose.orientation.x = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF
        robot_pose.pose.orientation.y = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && (wheel_l == 10 || wheel_r == 10 || imu_ang_vel == -10)) // dependent on imu and wheels
        {
            rate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // declare / initialise other variables
        //time at previous time step
        double prev_time = ros::Time::now().toSec();
        //size of time step
        double dt = 0;
        //store for left wheel_position @ step t-1
        double wheel_l_prev = wheel_l;
        //store for right wheel_position @ step t-1
        double wheel_r_prev = wheel_r;
        //store for linear velocity @ step t-1
        double linear_vel = 0;
        //store for angular velocity @ step t-1
        double angular_vel = 0;
        //store for robot heading @ step t-1
        double robot_angle = 0;
        ////////////////// DECLARE VARIABLES HERE //////////////////

        // loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION FILTER HERE //////////////////
            double wheel_l_current = wheel_l; //current left wheel rotation from callback (*UPDATE*)
            double wheel_r_current = wheel_r; //current right wheel rotation from callback (*UPDATE*)
            double delta_wheel_l = wheel_l_current - wheel_l_prev; //delta of left wheel rotation
            double delta_wheel_r = wheel_r_current - wheel_r_prev; //delta of right wheel rotation
            
            double linear_velocity_odom = (wheel_radius / (2 * dt)) * (delta_wheel_r + delta_wheel_l); //odometry based linear velocity 
            double angular_velocity_odom = (wheel_radius / (axle_track * dt)) * (delta_wheel_r - delta_wheel_l); //odometry based angular velocity

            double linear_velocity_imu = linear_vel + (imu_lin_acc * dt); //sum of the prev linear velocity + linear_acc * dt from imu
            double angular_velocity_imu = imu_ang_vel; //the imu reading for angular velocity
            
            double weighted_linear_velocity = (weight_odom_v * linear_velocity_odom) + (weight_imu_v * linear_velocity_imu); //fusing linear velocity measurements with a weighted average (*UPDATE*)
            double weighted_angular_velocity = (weight_odom_w * angular_velocity_odom) + (weight_imu_w * angular_velocity_imu); //fusing angular velocity measurements with a weighted average (*UPDATE*)
            
            double delta_heading = weighted_angular_velocity * dt; //using the weighted angular velocity * dt to compute the change in heading
            double current_robot_ang = robot_angle + delta_heading; //adding the change in heading to the current robot angle (*UPDATE*)
            double turn_radius = weighted_linear_velocity / weighted_angular_velocity; //turn radius computation using the ratio of weighted linear and angular velocities

            double current_position_x = 0; //(*UPDATE*)
            double current_position_y = 0; //(*UPDATE*)
            //note that the robot_position variable stores the previous position at this point in the program (xt-1,yt-1)
            if (weighted_angular_velocity > straight_thresh) //case where robot is not straight
            {
                current_position_x = robot_position.x + (turn_radius * (-sin(robot_angle) + sin(current_robot_ang)));
                current_position_y = robot_position.y + (turn_radius * (cos(robot_angle) - cos(current_robot_ang)));
            }

            else //case where robot is straight
            {
                current_position_x = robot_position.x + (weighted_linear_velocity * dt * cos(robot_angle));
                current_position_y = robot_position.y + (weighted_linear_velocity * dt * sin(robot_angle));
            }

            //prepare for the next time step
            wheel_l_prev = wheel_l_current;
            wheel_r_prev = wheel_r_current;
            linear_vel = weighted_linear_velocity;
            angular_vel = weighted_angular_velocity;
            robot_angle = current_robot_ang;
            robot_position.x = current_position_x;
            robot_position.y = current_position_y;

            // inject position and calculate quaternion for pose message, and publish
            robot_pose.pose.position.x = robot_position.x;
            robot_pose.pose.position.y = robot_position.y;
            robot_pose.pose.orientation.x = 0;
            robot_pose.pose.orientation.y = 0;
            robot_pose.pose.orientation.w = cos(robot_angle / 2);
            robot_pose.pose.orientation.z = sin(robot_angle / 2);
            pub_pose.publish(robot_pose);

            if (verbose)
            {
                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)",
                         robot_position.x, robot_position.y, robot_angle);
            }

            // sleep until the end of the required frequency
            rate.sleep();
        }
    }

    ROS_INFO("TMOTION: ===== END =====");
    return 0;
}
