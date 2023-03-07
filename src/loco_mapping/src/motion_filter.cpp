#include "motion_filter.h"

MotionFilter::MotionFilter(ros::NodeHandle &nh):
imu_angular_vel_(-10),
imu_linear_acc_(0),
wheel_l_(10),
wheel_r_(10)
{
    nh_ = nh;
    loadParams();
    //start listening for topics
    wheel_sub_ = nh_.subscribe("joint_states" , 1 , &MotionFilter::callbackWheels ,this);
    imu_sub_  = nh_.subscribe("imu" , 1 , &MotionFilter::callbackIMU , this);
    odom_sub_ = nh_.subscribe("odom" , 1, &MotionFilter::callbackOdom , this);

    //advertise topic
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose" , 1);
    speed_pub_ = nh_.advertise<std_msgs::Float64>("motion_filter_vel" , 1);
    
    robot_position_.setCoords(initial_x_ , initial_y_);
    weight_imu_v_ = 1 - weight_odom_v_;
    weight_imu_w_ = 1 - weight_odom_w_;
    robot_pose_mf_.pose.orientation.x = 0;
    robot_pose_mf_.pose.orientation.y = 0;
    robot_heading_ = 0;
    robot_pose_mf_.header.frame_id = "world";
    robot_pose_odom_.header.frame_id = "world";

    ROS_INFO("[Motion Filter]: Motion Filter Prepared!");
}

void MotionFilter::callbackIMU(const sensor_msgs::Imu::ConstPtr &msg)
{
    this->imu_angular_vel_ = msg->angular_velocity.z;
    this->imu_linear_acc_ = msg->linear_acceleration.x;
}

void MotionFilter::callbackWheels(const sensor_msgs::JointState::ConstPtr &msg)
{
    this->wheel_l_ = msg->position[1]; 
    this->wheel_r_ = msg->position[0]; 
}

void MotionFilter::callbackOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    this->odom_msg_ = *msg;
}

void MotionFilter::run()
{
    if (use_internal_odom_)
    {   
        ros::Rate spinrate(25.0);
        ROS_INFO("[Motion Filter]: Using Internal Odometry. Ground Truth will be used if Simulation");
        ROS_INFO("[Motion Filter]: Waiting for topics");

        while (ros::ok() && nh_.param("trigger_nodes", true) && odom_msg_.header.seq == 0 ) // ros::ok() && nh_.param("run", true) && msg_odom.header.seq == 0
        {
            spinrate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("[Motion Filter]: Starting Motion Filter using Internal Odometry!");
        while (ros::ok() && nh_.param("trigger_nodes",true))
        {
            ros::spinOnce();

            robot_pose_odom_.pose = odom_msg_.pose.pose;
            std_msgs::Float64 speed_msg;
            speed_msg.data = odom_msg_.twist.twist.linear.x;
            pose_pub_.publish(robot_pose_odom_);
            speed_pub_.publish(speed_msg);

            spinrate.sleep();
        }
    }

    else
    {
        ros::Rate spinrate(rate_);
        ROS_INFO("[Motion Filter]: Using Weighted Average Motion Filter!");
        ROS_INFO("[Motion Filter]: Waiting for topics");

        while (ros::ok() && nh_.param("trigger_nodes", true) && (wheel_l_ == 10 || wheel_r_ == 10 || imu_angular_vel_ == -10)) // dependent on imu and wheels
        {
            spinrate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("[Motion Filter]: Starting Motion Filter using Weighted Average Filter!");
        
        //time at previous time step
        double prev_time = ros::Time::now().toSec();
        //size of time step
        double dt = 0;
        //store for left wheel_position @ step t-1
        double wheel_l_prev = wheel_l_;
        //store for right wheel_position @ step t-1
        double wheel_r_prev = wheel_r_;
        //store for linear velocity @ step t-1
        double linear_vel = 0;
        //store for angular velocity @ step t-1
        double angular_vel = 0;

        while(ros::ok() && nh_.param("trigger_nodes", true))
        {
            ros::spinOnce();
            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0)
            {
                continue;
            }

            double wheel_l_current = wheel_l_; //current left wheel rotation from callback (*UPDATE*)
            double wheel_r_current = wheel_r_; //current right wheel rotation from callback (*UPDATE*)
            double delta_wheel_l = wheel_l_current - wheel_l_prev; //delta of left wheel rotation
            double delta_wheel_r = wheel_r_current - wheel_r_prev; //delta of right wheel rotation

            double linear_velocity_odom = (wheel_radius_ / (2 * dt)) * (delta_wheel_r + delta_wheel_l); //odometry based linear velocity 
            double angular_velocity_odom = (wheel_radius_ / (axle_track_ * dt)) * (delta_wheel_r - delta_wheel_l); //odometry based angular velocity
            
            double linear_velocity_imu = linear_vel + (imu_linear_acc_ * dt); //sum of the prev linear velocity + linear_acc * dt from imu
            double angular_velocity_imu = imu_angular_vel_; //the imu reading for angular velocity

            double weighted_linear_velocity = (weight_odom_v_ * linear_velocity_odom) + (weight_imu_v_ * linear_velocity_imu); //fusing linear velocity measurements with a weighted average (*UPDATE*)
            double weighted_angular_velocity = (weight_odom_w_ * angular_velocity_odom) + (weight_imu_w_ * angular_velocity_imu); //fusing angular velocity measurements with a weighted average (*UPDATE*)

            double delta_heading = weighted_angular_velocity * dt; //using the weighted angular velocity * dt to compute the change in heading
            double current_robot_heading = bot_utils::limit_angle(robot_heading_ + delta_heading); //adding the change in heading to the current robot angle (*UPDATE*)
            double turn_radius = weighted_linear_velocity / weighted_angular_velocity; //turn radius computation using the ratio of weighted linear and angular velocities

            double current_position_x = 0;
            double current_position_y = 0;
            if (weighted_angular_velocity > straight_thresh_) //case where robot is not straight
            {
                current_position_x = robot_position_.x + (turn_radius * (-sin(robot_heading_) + sin(current_robot_heading)));
                current_position_y = robot_position_.y + (turn_radius * (cos(robot_heading_) - cos(current_robot_heading)));
            }

            else //case where robot is straight
            {
                current_position_x = robot_position_.x + (weighted_linear_velocity * dt * cos(robot_heading_));
                current_position_y = robot_position_.y + (weighted_linear_velocity * dt * sin(robot_heading_));
            }
            
            //update previous store
            wheel_l_prev = wheel_l_current;
            wheel_r_prev = wheel_r_current;
            linear_vel = weighted_linear_velocity;
            angular_vel = weighted_angular_velocity;
            robot_heading_ = current_robot_heading;
            robot_position_.x = current_position_x;
            robot_position_.y = current_position_y;
            prev_time += dt;

            //robot_pose_odom_.pose = odom_msg_.pose.pose;
            
            robot_pose_mf_.pose.position.x = robot_position_.x;
            robot_pose_mf_.pose.position.y = robot_position_.y;
            robot_pose_mf_.pose.orientation.x = 0;
            robot_pose_mf_.pose.orientation.y = 0;
            robot_pose_mf_.pose.orientation.z = sin(robot_heading_ / 2);
            robot_pose_mf_.pose.orientation.w = cos(robot_heading_ / 2);

            pose_pub_.publish(robot_pose_mf_);

            if (verbose_)
            {
                ROS_INFO_STREAM("Robot Position: " << robot_position_.x << "," << robot_position_.y<<")");
                ROS_INFO_STREAM("Robot Heading: " << robot_heading_);
            }
            std_msgs::Float64 speed_msg;
            speed_msg.data = linear_vel;
            speed_pub_.publish(speed_msg);
            spinrate.sleep();
        }
    }
}

void MotionFilter::loadParams()
{
    if (!nh_.param("use_internal_odom", this->use_internal_odom_, true))
    {
        ROS_WARN(" [Motion Filter] : Param use_internal_odom not found, set to true");
    }
        
    if (!nh_.param("verbose_mf", this->verbose_, false))
    {
        ROS_WARN(" [Motion Filter] : Param verbose_motion not found, set to false");
    }
        
    if (!nh_.param("initial_x", this->initial_x_, 0.0))
    {
        ROS_WARN(" [Motion Filter] : Param initial_x not found, set to 0.0");
    }

    if (!nh_.param("initial_y", this->initial_y_, 0.0))
    {
        ROS_WARN(" [Motion Filter] : Param initial_y not found, set to 0.0");
    }

    if (!nh_.param("initial_x", this->robot_position_.x, 0.0))
    {
        ROS_WARN(" [Motion Filter] : Param initial_x not found, set to 0.0");
    }
        
    if (!nh_.param("initial_y", this->robot_position_.y, 0.0))
    {
        ROS_WARN(" [Motion Filter] : Param initial_y not found, set to 0.0");
    }

    if (!nh_.param("wheel_radius", this->wheel_radius_, 0.033))
    {
        ROS_WARN(" [Motion Filter] : Param wheel_radius not found, set to 0.033");
    }

    if (!nh_.param("axle_track", this->axle_track_, 0.16))
    {
        ROS_WARN(" [Motion Filter] : Param axle_track not found, set to 0.16");
    }
        
    if (!nh_.param("weight_odom_v", this->weight_odom_v_, 0.5))
    {
        ROS_WARN(" [Motion Filter] : Param weight_odom_v not found, set to 0.5");
    }
        
    if (!nh_.param("weight_odom_w", this->weight_odom_w_, 0.5))
    {
        ROS_WARN(" [Motion Filter] : Param weight_odom_w not found, set to 0.5");
    }
        
    if (!nh_.param("straight_thresh", this->straight_thresh_, 0.05))
    {
        ROS_WARN(" [Motion Filter] : Param straight_thresh not found, set to 0.05");
    }
        
    if (!nh_.param("mf_rate", this->rate_, 50.0))
    {
        ROS_WARN(" [Motion Filter] : Param mf_rate not found, set to 50");
    }
}
