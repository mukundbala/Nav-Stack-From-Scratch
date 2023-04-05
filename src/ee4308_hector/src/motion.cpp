#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h> // publish to pose topic
#include <geometry_msgs/Vector3Stamped.h>            // subscribe to magnetic topic
#include <sensor_msgs/Imu.h>                         // subscribe to imu topic
#include <sensor_msgs/NavSatFix.h>                   // subscribe to GPS
#include <hector_uav_msgs/Altimeter.h>               // subscribe to barometer
#include <sensor_msgs/Range.h>                       // subscribe to sonar
#include <nav_msgs/Odometry.h>                       // subscribe to ground truth topic
#include <std_srvs/Empty.h>                          // Service to calrbrate motors
#include <opencv2/core/core.hpp>
#include "common.hpp"

#define NaN std::numeric_limits<double>::quiet_NaN()

// global parameters to be read from ROS PARAMs
bool verbose, use_ground_truth, enable_baro, enable_magnet, enable_sonar, enable_gps, variance;

// others
bool ready = false; // signal to topics to begin

// --------- PREDICTION WITH IMU ----------
const double G = 9.8;
double prev_imu_t = 0;
cv::Matx21d X = {0, 0}, Y = {0, 0}; // see intellisense. This is equivalent to cv::Matx<double, 2, 1>
cv::Matx21d A = {0, 0};                                    
cv::Matx21d Z = {0, 0};                                
cv::Matx22d P_x = cv::Matx22d::ones(), P_y = cv::Matx22d::zeros();
cv::Matx22d P_a = cv::Matx22d::ones();
cv::Matx22d P_z = cv::Matx22d::ones();     

cv::Matx22d Q_x, Q_y;
cv::Matx<double, 1, 1> Q_z, Q_a;
cv::Matx22d F_x, W_x, F_y, W_y, F_z, F_a;
cv::Matx21d W_z, W_a;
cv::Matx21d U_x, U_y;
cv::Matx<double, 1, 1> U_z, U_a;
double ua = NaN, ux = NaN, uy = NaN, uz = NaN;
double qa, qx, qy, qz;
std::vector<double> var_x, var_y, var_z, var_a;

// see https://docs.opencv.org/3.4/de/de1/classcv_1_1Matx.html
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!ready)
    {
        prev_imu_t = msg->header.stamp.toSec();
        return;
    }

    // calculate time
    double imu_t = msg->header.stamp.toSec();
    double imu_dt = imu_t - prev_imu_t;
    prev_imu_t = imu_t;

    // read inputs
    ua = msg->angular_velocity.z;
    ux = msg->linear_acceleration.x;
    uy = msg->linear_acceleration.y;
    uz = msg->linear_acceleration.z;

    //// IMPLEMENT IMU ////

    // Predicting x state
    F_x = {
            1, imu_dt,
            0, 1
          };
    W_x = {
            -0.5 * imu_dt * imu_dt * cos(A(0)), 0.5 * imu_dt * imu_dt * sin(A(0)),
            -imu_dt * cos(A(0))               , imu_dt * sin(A(0))
          };
    U_x = {
            ux,
            uy
          };
    Q_x = {
            qx, 0,
            0, qy
          };
    X = (F_x * X) + (W_x * U_x);
    P_x = (F_x * P_x * (F_x.t())) + (W_x * Q_x * (W_x.t()));
    
    // Predicting y state
    F_y = {
            1, imu_dt,
            0, 1
          };

    W_y = {
            -0.5 * imu_dt * imu_dt * sin(A(0)), -0.5 * imu_dt * imu_dt * cos(A(0)),
            -imu_dt * sin(A(0))               , -imu_dt * cos(A(0))
          };

    U_y = {
            ux,
            uy
          };
    Q_y = {
            qx, 0,
            0, qy
          };

    Y = (F_y * Y) + (W_y * U_y);
    P_y = (F_y * P_y * (F_y.t())) + (W_y * Q_y * (W_y.t()));

    // Predicting z state
    F_z = {
            1, imu_dt,
            0, 1
          };
    W_z = {
            0.5 * (imu_dt * imu_dt), 
            imu_dt
          };
    U_z = {uz - G};
    Q_z = {qz};
    Z = (F_z * Z) + (W_z * U_z);
    P_z = (F_z * P_z * (F_z.t())) + (W_z * Q_z * (W_z.t()));

    // Predicting ang state
    F_a = {
            1, 0,
            0, 0
          };
    W_a = {
            imu_dt,
            1
          };
    U_a = {ua};
    Q_a = {qa};
    A = (F_a * A) + (W_a * U_a);
    P_a = (F_a * P_a * (F_a.t())) + (W_a * Q_a * (W_a.t()));
    
}

// --------- GPS ----------
// https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
cv::Matx31d GPS = {NaN, NaN, NaN};
cv::Matx31d initial_pos = {NaN, NaN, NaN}; // written below in main. no further action needed.
cv::Matx31d ECEF, initial_ECEF = {NaN, NaN, NaN}, NED;
cv::Matx33d R_ECEF2NED;
cv::Matx33d R_NED2GAZEBO = {
                            1, 0, 0,
                            0, -1, 0,
                            0, 0, -1
                           };
cv::Matx21d K_gps_x, K_gps_y, K_gps_z; // Kalman gain for GPS
cv::Matx12d H = {1.0, 0};
cv::Matx<double, 1, 1> V = {1.0};

const double DEG2RAD = M_PI / 180;
const double RAD_POLAR = 6356752.3;
const double RAD_EQUATOR = 6378137;
double e_sq = 1 - (pow(RAD_POLAR/RAD_EQUATOR, 2)); // First numerical eccentricity, e^2
double r_gps_x, r_gps_y, r_gps_z;

cv::Matx<double, 1, 1> r_x = {r_gps_x}, r_y = {r_gps_y}, r_z = {r_gps_z};
cv::Matx<double, 1, 1> gps_x, gps_y, gps_z;         
cv::Matx<double, 1, 1> state_x, state_y, state_z;   
double N;   // prime_radius, N(φ)

double primeVerticalRadius_calc(double lat_measurement)
{
    double prime_rad;
    prime_rad = RAD_EQUATOR/(sqrt(1 - (e_sq * (sin(lat_measurement) * sin(lat_measurement)))));
    return prime_rad;
}
void cbGps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!ready && std::isnan(GPS(0)))
        return;

    
    //// IMPLEMENT GPS /////
    double lat = msg->latitude;  // φ   
    double lon = msg->longitude; // λ
    double alt = msg->altitude;  // h
    // double var_E = msg->position_covariance[0];
    // ROS_INFO("[HM]  Longitude variance: %7.3lf", var_E);
    // double var_N = msg->position_covariance[4];
    // ROS_INFO("[HM]  Latitude variance: %7.3lf", var_N);
    // double var_U = msg->position_covariance[8];
    // ROS_INFO("[HM]  Altitude variance: %7.3lf", var_U);

    // Convert degree to radian
    lat *= DEG2RAD;
    lon *= DEG2RAD;

    // Calculate N(φ)
    N = primeVerticalRadius_calc(lat);

    // Calculate rotation matrix R to transform from ECEF to NED
    R_ECEF2NED = {
                    -(sin(lat) * cos(lon)), -sin(lon), -(cos(lat) * cos(lon)),
                    -(sin(lat) * sin(lon)), cos(lon), -(cos(lat) * sin(lon)),
                    cos(lat)              , 0       , -sin(lat)
                 };

    ECEF = {
            (N + alt) * cos(lat) * cos(lon),
            (N + alt) * cos(lat) * sin(lon),
            (((RAD_POLAR/RAD_EQUATOR) * (RAD_POLAR/RAD_EQUATOR) * N) + alt) * sin(lat)
           };

    // for initial message -- you may need this:
    if (std::isnan(initial_ECEF(0)))
    {   // calculates initial ECEF and returns
        initial_ECEF = ECEF;
        return;
    }

    NED = (R_ECEF2NED.t()) * (ECEF - initial_ECEF);
    GPS = (R_NED2GAZEBO * NED) + initial_pos;

    var_x.push_back(GPS(0));
    var_y.push_back(GPS(1));
    var_z.push_back(GPS(2));

    // --- EKF correction for x gps ---

    // Measurement matrix and predicted state matrix 
    gps_x = {GPS(0)};
    state_x = {X(0)};

    // Calculate Kalman gain
    K_gps_x = (P_x * (H.t())) * (((H * P_x * (H.t())) + (V * r_x * V)).inv());

    // Update state matrix
    X = X + (K_gps_x * (gps_x - state_x));

    // Update state covariance matrix
    P_x = P_x - (K_gps_x * H * P_x);

    // --- EKF correction for y gps ---

    // Measurement matrix and predicted state matrix 
    gps_y = {GPS(1)};
    state_y = {Y(0)};

    // Calculate Kalman gain
    K_gps_y = (P_y * (H.t())) * (((H * P_y * (H.t())) + (V * r_y * V)).inv());

    // Update state matrix
    Y = Y + (K_gps_y * (gps_y - state_y));

    // Update state covariance matrix
    P_y = P_y - (K_gps_y * H * P_y);

    // --- EKF correction for z gps ---

    // Measurement matrix and predicted state matrix 
    gps_z = {GPS(2)};
    state_z = {Z(0)};

    // Calculate Kalman gain
    K_gps_z = (P_z * (H.t())) * (((H * P_z * (H.t())) + (V * r_z * V)).inv());

    // Update state matrix
    Z = Z + (K_gps_z * (gps_z - state_z));

    // Update state covariance matrix
    P_z = P_z - (K_gps_z * H * P_z);
}

// --------- Magnetic ----------
double a_mgn = NaN, intial_mgn;
double r_mgn_a;
cv::Matx<double, 1, 1> r_a = {r_mgn_a}, a_mgn_m, a_mgn_s;
cv::Matx21d K_magn_a;   // Kalman gain for magnetometer
void cbMagnet(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    if (!ready)
        return;
    
    //// IMPLEMENT MAGNETMETER ////
    double mx = msg->vector.x;
    double my = msg->vector.y;

    // if (std::isnan(a_mgn))
    // {
    //     intial_mgn = atan2(-my, mx);
    // }

    a_mgn = atan2(-my, mx);

    var_a.push_back(a_mgn);

    // --- EKF Correction ---

    // Measurement matrix and predicted state matrix 
    a_mgn_m = {a_mgn}; 
    a_mgn_s = {A(0)};

    // Kalman gain
    K_magn_a = P_a * (H.t()) * ((H * P_a * (H.t()) + V * r_a * V).inv());

    // Update state matrix
    A = A + (K_magn_a * (a_mgn_m - a_mgn_s));

    // Update state covariance matrix
    P_a = P_a - (K_magn_a * H * P_a);
}

// --------- Baro ----------
double z_bar = NaN;
double r_bar_z;
double b_bar;
void cbBaro(const hector_uav_msgs::Altimeter::ConstPtr &msg)
{
    if (!ready)
        return;

    
    //// IMPLEMENT BARO ////
    // z_bar = msg->altitude;
    // K_z = (P_z * H.t()) * (((H * P_z * H.t()) + (V * r_bar_z * V.t())).inv());
    // Z = Z + (K_z * (z_bar - Z(0) - b_bar));
    // P_z = P_z - (K_z * H * P_z);
         
}

// --------- Sonar ----------
double z_snr = NaN;
double r_snr_z;
cv::Matx<double, 1, 1> r_snr = {r_snr_z}, z_snr_m, z_snr_s;
cv::Matx21d K_snr_z;    // Kalman gain for sonar
std::vector<double> var_snr;
void cbSonar(const sensor_msgs::Range::ConstPtr &msg)
{
    if (!ready && std::isnan(GPS(0)))
        return;

    //// IMPLEMENT SONAR ////
    z_snr = msg->range;
    
    if (z_snr > 1.5) 
    {
        var_snr.push_back(z_snr);

        // --- EKF Calculation ---

        // Measurement matrix and predicted state matrix
        z_snr_m = {z_snr};
        z_snr_s = {Z(0)};

        // Kalman gain
        K_snr_z = P_z * (H.t()) * ((H * P_z * (H.t()) + V * r_snr * V).inv());

        // Update state matrix
        Z = Z + (K_snr_z * (z_snr_m - z_snr_s));

        // Update state covariance matrix
        P_z = P_z - (K_snr_z * H * P_z);
    }
    return;
}

// --------- GROUND TRUTH ----------
nav_msgs::Odometry msg_true;
void cbTrue(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_true = *msg;
}

// ------ Calculate Variance --------
double calc_variance(std::vector<double> & vec)
{
    int size = vec.size();
    if (size < 100)
    {
        ROS_INFO("Less than 100 measurements collected");
        return 0;
    }
    
    // Add elements in the vector and calculate mean
    double sum;
    for (int i = 0; i < size; i++) 
    {
        sum += vec[i];
    }
    double mean = sum / size;

    // Sum of mean squared errors
    double err = 0, mse = 0, var;
    for (int j = 0; j < size; j++)
    {
        err = vec[j] - mean;
        mse += pow(err, 2);
    }
    var = mse / (size - 1);
    // vec.clear();
    return var;
}

// --------- MEASUREMENT UPDATE WITH GROUND TRUTH ----------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_motion");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    double motion_iter_rate;
    if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
        ROS_WARN("HMOTION: Param motion_iter_rate not found, set to 50.0");
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN("HMOTION: Param verbose_motion not found, set to false");
    if (!nh.param("initial_x", X(0), 0.0))
        ROS_WARN("HMOTION: Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", Y(0), 0.0))
        ROS_WARN("HMOTION: Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", Z(0), 0.178))
        ROS_WARN("HMOTION: Param initial_z not found, set initial_z to 0.178");
    initial_pos = {X(0), Y(0), Z(0)};
    if (!nh.param("use_ground_truth", use_ground_truth, true))
        ROS_WARN("HMOTION: Param use_ground_truth not found, set use_ground_truth to true");
    if (!nh.param("r_gps_x", r_gps_x, 1.0))
        ROS_WARN("HMOTION: Param r_gps_x not found, set to 1.0");
    if (!nh.param("r_gps_y", r_gps_y, 1.0))
        ROS_WARN("HMOTION: Param r_gps_y not found, set to 1.0");
    if (!nh.param("r_gps_z", r_gps_z, 1.0))
        ROS_WARN("HMOTION: Param r_gps_z not found, set to 1.0");
    if (!nh.param("r_mgn_a", r_mgn_a, 1.0))
        ROS_WARN("HMOTION: Param r_mgn_a not found, set to 1.0");
    if (!nh.param("r_bar_z", r_bar_z, 1.0))
        ROS_WARN("HMOTION: Param r_bar_z not found, set to 1.0");
    if (!nh.param("r_snr_z", r_snr_z, 1.0))
        ROS_WARN("HMOTION: Param r_snr_z not found, set to 1.0");
    if (!nh.param("qa", qa, 1.0))
        ROS_WARN("HMOTION: Param qa not found, set to 1.0");
    if (!nh.param("qx", qx, 1.0))
        ROS_WARN("HMOTION: Param qx not found, set to 1.0");
    if (!nh.param("qy", qy, 1.0))
        ROS_WARN("HMOTION: Param qy not found, set to 1.0");
    if (!nh.param("qz", qz, 1.0))
        ROS_WARN("HMOTION: Param qz not found, set to 1.0");
    if (!nh.param("enable_baro", enable_baro, true))
        ROS_WARN("HMOTION: Param enable_baro not found, set to true");
    if (!nh.param("enable_magnet", enable_magnet, true))
        ROS_WARN("HMOTION: Param enable_magnet not found, set to true");
    if (!nh.param("enable_sonar", enable_sonar, true))
        ROS_WARN("HMOTION: Param enable_sonar not found, set to true");
    if (!nh.param("enable_gps", enable_gps, true))
        ROS_WARN("HMOTION: Param enable_gps not found, set to true");
    if (!nh.param("variance", variance, true))
        ROS_WARN("HMOTION: Param variance not found, set to true");

    // --------- Subscribers ----------
    ros::Subscriber sub_true = nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &cbTrue);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("raw_imu", 1, &cbImu);
    ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("fix", 1, &cbGps);
    if (!enable_gps)
        sub_gps.shutdown();
    ros::Subscriber sub_magnet = nh.subscribe<geometry_msgs::Vector3Stamped>("magnetic", 1, &cbMagnet);
    if (!enable_magnet)
        sub_magnet.shutdown();
    ros::Subscriber sub_baro = nh.subscribe<hector_uav_msgs::Altimeter>("altimeter", 1, &cbBaro);
    if (!enable_baro)
        sub_baro.shutdown();
    ros::Subscriber sub_sonar = nh.subscribe<sensor_msgs::Range>("sonar_height", 1, &cbSonar);
    if (!enable_sonar)
        sub_sonar.shutdown();

    // --------- Publishers ----------
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1, true);
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.frame_id = "world";   // for rviz
    msg_pose.pose.pose.orientation.x = 0; // no roll
    msg_pose.pose.pose.orientation.y = 0; // no pitch
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("velocity", 1, true); // publish velocity
    geometry_msgs::Twist msg_vel;

    // --------- Wait for Topics ----------
    ROS_INFO("HMOTION: Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ((std::isnan(ux) && msg_true.header.seq == 0))) // wait for imu and truth only
        ros::spinOnce(); // update subscribers

    if (!ros::ok())
    { // ROS shutdown
        ROS_INFO("HMOTION: ===== END =====");
        return 0;
    }

    // --------- Calibrate Gyro service ----------
    ROS_INFO("HMOTION: Calibrating Gyro...");
    ros::ServiceClient calibrate_gyro = nh.serviceClient<std_srvs::Empty>("raw_imu/calibrate");
    std_srvs::Empty calibrate_gyro_srv;
    if (calibrate_gyro.call(calibrate_gyro_srv))
        ROS_INFO("HMOTION: Calibrated Gyro");
    else
        ROS_WARN("HMOTION: Gyro cannot be calibrated!");

    // --------- Main loop ----------

    ros::Rate rate(motion_iter_rate);
    ROS_INFO("HMOTION: ===== BEGIN =====");
    ready = true;
    while (ros::ok() && nh.param("run", true))
    {
        ros::spinOnce(); // update topics

        // Verbose
        if (verbose)
        {
            auto & tp = msg_true.pose.pose.position;
            auto &q = msg_true.pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            ROS_INFO("[HM] ---------X-------Y-------Z-------A------");
            ROS_INFO("[HM]  TRUE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", tp.x, tp.y, tp.z, atan2(siny_cosp, cosy_cosp));
            ROS_INFO("[HM] STATE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", X(0), Y(0), Z(0), A(0));
            ROS_INFO("[HM]   GPS(%7.3lf,%7.3lf,%7.3lf, ---- )", GPS(0), GPS(1), GPS(2));
            ROS_INFO("[HM] MAGNT( ----- , ----- , ----- ,%6.3lf)", a_mgn);
            ROS_INFO("[HM]  BARO( ----- , ----- ,%7.3lf, ---- )", z_bar);
            ROS_INFO("[HM] BAROB( ----- , ----- ,%7.3lf, ---- )", Z(3));
            ROS_INFO("[HM] SONAR( ----- , ----- ,%7.3lf, ---- )", z_snr);
        }

        // Variance
        if (variance) 
        {
            ROS_INFO("[HM] ---------------- VARIANCE --------------");
            ROS_INFO("[HM] ---------X-------Y-------Z-------A------");
            ROS_INFO("[HM]   GPS(%7.3lf,%7.3lf,%7.3lf, ---- )", calc_variance(var_x), calc_variance(var_y), calc_variance(var_z));
            ROS_INFO("[HM] MAGNT( ----- , ----- , ----- ,%6.3lf)", calc_variance(var_a));
            ROS_INFO("[HM]  BARO( ----- , ----- ,%7.3lf, ---- )", calc_variance(var_z));
            ROS_INFO("[HM] SONAR( ----- , ----- ,%7.3lf, ---- )", calc_variance(var_z));
        }

        //  Publish pose and vel
        if (use_ground_truth)
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position = msg_true.pose.pose.position;
            msg_pose.pose.pose.orientation = msg_true.pose.pose.orientation;
            msg_vel = msg_true.twist.twist;
        }
        else
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position.x = X(0);
            msg_pose.pose.pose.position.y = Y(0);
            msg_pose.pose.pose.position.z = Z(0);
            msg_pose.pose.covariance[0] = P_x(0, 0);  // x cov
            msg_pose.pose.covariance[7] = P_y(0, 0);  // y cov
            msg_pose.pose.covariance[14] = P_z(0, 0); // z cov
            msg_pose.pose.covariance[35] = P_a(0, 0); // a cov
            msg_pose.pose.pose.orientation.w = cos(A(0) / 2);
            msg_pose.pose.pose.orientation.z = sin(A(0) / 2);
            msg_vel.linear.x = X(1);
            msg_vel.linear.y = Y(1);
            msg_vel.linear.z = Z(1);
            msg_vel.angular.z = A(1);
        }
        pub_pose.publish(msg_pose);
        pub_vel.publish(msg_vel);

        rate.sleep();
    }

    ROS_INFO("HMOTION: ===== END =====");
    return 0;
}