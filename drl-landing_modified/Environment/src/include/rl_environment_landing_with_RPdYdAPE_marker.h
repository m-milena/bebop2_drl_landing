//#define HAS_OPENCV3 @OPENCV3_FOUND@

// Comment and uncomment defines
#define REAL_FLIGHT_MODE      // Configures the compilation for real flight test
//#define ENABLE_RELATIVE_YAW     // Enables the extraction of the Yaw from the platform
//#define LOG_DATA              // Logs the position of the UAV and MP in CSV file
//#define LOG_STATISTICS          // Logs statistics of landing for a long period of testing
//#define RUNAWAY                 // Configures the experiment to use the moving platform in runaway mode

#define HAS_OPENCV3 1
#if HAS_OPENCV3 == 1

#ifndef _RL_ENVIRONMENT_LANDING_WITH_RPDYDAPE_MARKER_H_
#define _RL_ENVIRONMENT_LANDING_WITH_RPDYDAPE_MARKER_H_

#include "rl_environment_gazebo_ros.h"
#include "droneMsgsROS/droneCommand.h"
#include "droneMsgsROS/droneStatus.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "ctime"
#include "stdlib.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <aruco/aruco.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <collision_avoid/msgCoordinates.h>

#if defined LOG_DATA || defined LOG_STATISTICS
#include <fstream>
#include <iostream>
#include <chrono>
#endif

#ifdef RUNAWAY
#include "std_msgs/Bool.h"
#endif

class RlEnvironmentLandingWithRPdYdAPEMarker: public RlEnvironmentGazeboRos
{
private:

    std::vector<float> state_;

    enum {
            ACTIONS_DIM = 5,
            STATE_DIM_LOW_DIM = 7,
            NUM_ITERATIONS = 5,
            NUM_EPISODE_ITERATIONS = 1000000000,
            MAX_POSE_X = 3,
            MAX_POSE_Y = 6
         };

#if defined LOG_DATA || defined LOG_STATISTICS
    const std::string EXPERIMENTS_FOLDER = "/home/milena/Pulpit/statics/";
#endif

    const std::string UAV_NAME = "bebop_leader";
    const std::string PLATFORM_NAME = "vision_landing_platform";
    const float Z_STEP = 0.01;
    const float Z_INITIAL = 1.0;
    const float Z_PLATFORM_OFFSET = 0.15;
    const float MAX_ALLOWED_ALTIUDE = 3.0;
    const float PLATFORM_SIZE = 0.15;
//landing on static platform and low speed
    const float SCALE_RP = 0.2;
   const float SCALE_YAW = 0.2;
    const float SCALE_ALTITUDE =0.7;
//high platform speed
//  const float SCALE_RP = 0.2;
//  const float SCALE_YAW = 0.4;
//  const float SCALE_ALTITUDE = 0.9;

    const float MAX_EMERGENCY_ALTITUDE = 0.7;
    const float PLATFORM_IN_SIZE =0.5;

    const float EMERGENCY_THRESHOLD = 0.8;
    const float DESIRED_EMERGENCY_ALTITUDE = 0.4;
    const bool  ENABLE_GUIDED_STATES = false;
    const bool  ENABLE_EMERGENCY_INHIBITION = true;
    const bool  SHOW_IMAGES = true;
    const int   GUIDED_STATES_INTERVAL = 2;
    const int   CAMERA_WIDTH = 856;
    const int   CAMERA_HEIGHT = 480;
    const double MARKER_SIZE = 0.235;
    const double MARKER_SIZE_SMALL = 0.117;

    const double ARUCO_0_OFFSET_X =  0.452;     // Camera frame of reference
    const double ARUCO_0_OFFSET_Y = -0.344;
    const double ARUCO_1_OFFSET_X = -0.410;
    const double ARUCO_1_OFFSET_Y = -0.350;
    const double ARUCO_2_OFFSET_X = -0.419;
    const double ARUCO_2_OFFSET_Y =  0.322;
    const double ARUCO_3_OFFSET_X =  0.470;
    const double ARUCO_3_OFFSET_Y =  0.319;
    const double ARUCO_4_OFFSET_X =  0.000;
    const double ARUCO_4_OFFSET_Y =  0.000;
    const double ARUCO_5_OFFSET_X =  0.323;
    const double ARUCO_5_OFFSET_Y = -0.006;
    const double ARUCO_6_OFFSET_X =  0.013;
    const double ARUCO_6_OFFSET_Y = -0.280;
    const double ARUCO_7_OFFSET_X = -0.300;
    const double ARUCO_7_OFFSET_Y = -0.015;
    const double ARUCO_8_OFFSET_X =  0.006;
    const double ARUCO_8_OFFSET_Y =  0.276;
    const int   ARUCO_0_ID = 13;
    const int   ARUCO_1_ID = 14;
    const int   ARUCO_2_ID = 11;
    const int   ARUCO_3_ID = 12;
    const int   ARUCO_4_ID = 33;
    const int   ARUCO_5_ID = 15;
    const int   ARUCO_6_ID = 16;
    const int   ARUCO_7_ID = 17;
    const int   ARUCO_8_ID = 18;
    const double CAMERA_X =  0.0;
    const double CAMERA_Y = -0.2;
    const double CAMERA_Z =  0.05;

    const float ROOM_X_MIN = -0.9; 
    const float ROOM_X_MAX = 3;
    const float ROOM_Y_MIN = -1.5;
    const float ROOM_Y_MAX = 1.9;
    const float ROOM_Z_MAX = 2.5;

#ifdef ENABLE_RELATIVE_YAW
    float current_relative_yaw_;
#endif

    float current_z_;
    float current_yaw_, current_roll_, current_pitch_, current_bebop_yaw_;
    float prev_x_relative_, prev_y_relative_, prev_x_relative_legacy_, prev_y_relative_legacy_;
    bool landing_executed_, reset_executed_;
    float emergency_altitude_;
    int status_, episode_no_;
    boost::mutex status_mutex_;

    // Ros publishers
    ros::Publisher uav_control_pub;
    ros::Publisher drone_command_publ_, current_pub_;
    image_transport::Publisher aruco_camera_publ_;

    // Ros subscribers
    ros::Subscriber image_sub_;
    ros::Subscriber bebop_gt_sub_;
    ros::Subscriber bebop_odom_sub;

    // Other members
    float uav_pose_x_, uav_pose_y_, uav_pose_z_, uav_velocity_x_, uav_velocity_y_;
    boost::mutex uav_mutex_;

    float platform_pose_x_, platform_pose_y_, platform_pose_z_, platform_velocity_x_, platform_velocity_y_;
    boost::mutex platform_mutex_;

    double relative_pose_x_, relative_pose_y_, relative_pose_z_, relative_velocity_x_, relative_velocity_y_;
    boost::mutex relative_mutex_;


    int bumper_state_;
    boost::mutex bumper_states_mutex_;

    int position_lost_;

    bool initial_yaw_;
    float initial_yaw_value_;

#ifdef RUNAWAY
    ros::Time initial_time_;
    const double RUNAWAY_TIME = 5;     // s
    const double SETTELING_TIME = 3;    // s
#endif

#if defined LOG_DATA || defined LOG_STATISTICS
    std::ofstream outFile_, outFile2_;
#endif

//     Initialize Aruco parameters

    //Real Bebop 2 camera
    float camera_params_[9] = { 536.774231, 0.000000, 432.798910, 0.000000, 527.838517, 222.208294, 0.000000, 0.000000, 1.000000 };
    cv::Mat camera_parameters_ = cv::Mat(3, 3, CV_32F, camera_params_);
    float distortion_coeff_[5] = { 0.002459, 0.005719, -0.010226, 0.004865, 0.000000 };
    cv::Mat distortion_coefficients_ = cv::Mat(1, 5, CV_32F, distortion_coeff_);
    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    double bebop_gt_x_, bebop_gt_y_, bebop_gt_z_, platform_gt_x_, platform_gt_y_, platform_gt_z_;
    double bebop_x, bebop_y, bebop_z;


public:
    // Polymorphic member functions
    bool Step(rl_srvs::AgentSrv::Request &request, rl_srvs::AgentSrv::Response &response);
    bool EnvDimensionality(rl_srvs::EnvDimensionalitySrv::Request &request, rl_srvs::EnvDimensionalitySrv::Response &response);
    bool ResetSrv(rl_srvs::ResetEnvSrv::Request &request, rl_srvs::ResetEnvSrv::Response &response);
    bool Reset();
    void InitChild(ros::NodeHandle n);

    // Member functions
    void StatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void generateRotx(double phi, cv::Mat &R);
    void generateRoty(double theta, cv::Mat &R);
    void generateRotz(double chi, cv::Mat &R);
    void generateHomogeneousTransMatrix(cv::Vec3d p, cv::Mat &T);
    void generateHomogeneousTransMatrixFromAruco(cv::Vec3d tvec, cv::Vec3d rvec, cv::Mat &T);
    void platformGTCallback(const nav_msgs::Odometry& msg);
    void bebopGTCallback(const nav_msgs::Odometry& msg);
    void bebopPoseCallback(const collision_avoid::msgCoordinates::ConstPtr& msg);



    // Getters and setters
    void SetUavState(float x, float y, float z, float dx, float dy){
        uav_mutex_.lock();
        uav_pose_x_ = x;
        uav_pose_y_ = y;
        uav_pose_z_ = z;
        uav_velocity_x_ = dx;
        uav_velocity_y_ = dy;
        uav_mutex_.unlock();
    }

    void GetUavState(float &x, float &y, float &z, float &dx, float &dy){
        uav_mutex_.lock();
        x = uav_pose_x_;
        y = uav_pose_y_;
        z = uav_pose_z_;
        dx = uav_velocity_x_;
        dy = uav_velocity_y_;
        uav_mutex_.unlock();
    }

    void SetPlatformState(float x, float y, float z, float dx, float dy){
        platform_mutex_.lock();
        platform_pose_x_ = x;
        platform_pose_y_ = y;
        platform_pose_z_ = z;
        platform_velocity_x_ = dx;
        platform_velocity_y_ = dy;
        platform_mutex_.unlock();
    }

    void GetPlatformState(float &x, float &y, float &z, float &dx, float &dy){
        platform_mutex_.lock();
        x = platform_pose_x_;
        y = platform_pose_y_;
        z = platform_pose_z_;
        dx = platform_velocity_x_;
        dy = platform_velocity_y_;
        platform_mutex_.unlock();
    }

    void SetRelativeState(double x, double y, double z){
        relative_mutex_.lock();
        relative_pose_x_ = x;
        relative_pose_y_ = y;
        relative_pose_z_ = z;
        relative_mutex_.unlock();
    }

    void GetRelativeState(float &x, float &y, float &z){
        relative_mutex_.lock();
        x = (float) relative_pose_x_;
        y = (float) relative_pose_y_;
        z = (float) relative_pose_z_;
        relative_mutex_.unlock();
    }


    void SetBumperState(int state){
        bumper_states_mutex_.lock();
        bumper_state_ = state;
        bumper_states_mutex_.unlock();
    }

    void GetBumperState(int &state){
        bumper_states_mutex_.lock();
        state = bumper_state_;
        bumper_states_mutex_.unlock();
    }

    void SetStatus(int status){
        status_mutex_.lock();
        status_ =  status;
        status_mutex_.unlock();
    }

    void GetStatus(int &status){
        status_mutex_.lock();
        status = status_;
        status_mutex_.unlock();
    }

    RlEnvironmentLandingWithRPdYdAPEMarker(){}
    ~RlEnvironmentLandingWithRPdYdAPEMarker(){}
};

#endif
#endif
