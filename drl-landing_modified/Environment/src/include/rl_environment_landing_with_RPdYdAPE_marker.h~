//#define HAS_OPENCV3 @OPENCV3_FOUND@

// Comment and uncomment defines
//#define REAL_FLIGHT_MODE      // Configures the compilation for real flight test
//#define PBVS_MODE               // Configures the compilation for PBVS mode
//#define RUNAWAY                 // Configures the experiment to use the moving platform in runaway mode
#define ENABLE_RELATIVE_YAW     // Enables the extraction of the Yaw from the platform
//#define LOG_DATA              // Logs the position of the UAV and MP in CSV file
//#define PUBLISH_TF            // Publish tf of UAV and MP in world frame
//#define LOG_STATISTICS          // Logs statistics of landing for a long period of testing

#define HAS_OPENCV3 1
#if HAS_OPENCV3 == 1

#ifndef _RL_ENVIRONMENT_LANDING_WITH_RPDYDAPE_MARKER_H_
#define _RL_ENVIRONMENT_LANDING_WITH_RPDYDAPE_MARKER_H_


#include "rl_environment_gazebo_ros.h"
#include "droneMsgsROS/droneSpeeds.h"
#include "droneMsgsROS/dronePositionRefCommandStamped.h"
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneCommand.h"
#include "droneMsgsROS/droneStatus.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ContactsState.h"
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
#include <aruco.h>
#include <tf/transform_datatypes.h>
#ifdef REAL_FLIGHT_MODE
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#endif

#if defined LOG_DATA || defined LOG_STATISTICS
#include <fstream>
#include <iostream>
#include <chrono>
#endif

#ifdef PUBLISH_TF
#include <tf/transform_broadcaster.h>
#endif

#ifdef PBVS_MODE
#include "droneMsgsROS/droneSpeeds.h"
#include "control_toolbox/pid.h"
#include </usr/include/eigen3/Eigen/Dense>
#endif

#ifdef RUNAWAY
#include "std_msgs/Bool.h"
#endif

class RlEnvironmentLandingWithRPdYdAPEMarker: public RlEnvironmentGazeboRos
{
private:

    // Test configuration parameters
    const bool ENABLE_PAUSED_SIMULATION = false;

    std::vector<float> state_;

    enum {
            ACTIONS_DIM = 5,
            STATE_DIM_LOW_DIM = 7,
            NUM_ITERATIONS = 5,
#ifdef REAL_FLIGHT_MODE
            NUM_EPISODE_ITERATIONS = 1000000000,
#else
            NUM_EPISODE_ITERATIONS = 1100,
#endif
            MAX_POSE_X = 3,
            MAX_POSE_Y = 6
         };

#if defined LOG_DATA || defined LOG_STATISTICS
    const std::string EXPERIMENTS_FOLDER = "/media/alejandro/DATA/Shared/paper_experiments/paper-landing-iros-2018/";
//    const std::string EXPERIMENTS_FOLDER = "/home/alejo/paper_experiments/paper-landing-iros-2018/";

#endif

    const std::string UAV_NAME = "hummingbird1";
    const std::string PLATFORM_NAME = "vision_landing_platform";
    const float Z_STEP = 0.01;
    const float Z_INITIAL = 3.0;
    const float Z_PLATFORM_OFFSET = 0.15;
    const float MAX_ALLOWED_ALTIUDE = 4.0;
#ifdef REAL_FLIGHT_MODE
    const float PLATFORM_SIZE = 0.487;
    // Soft
//    const float SCALE_RP = 0.08;
//    const float SCALE_YAW = 0.1;
//    const float SCALE_ALTITUDE = 0.1;
    // Aggressive
    const float SCALE_RP = 0.23;
    const float SCALE_YAW = 0.1;
    const float SCALE_ALTITUDE = 0.4;

    const float MAX_EMERGENCY_ALTITUDE = 1.05;
    const float PLATFORM_IN_SIZE = 0.10;
#else
    const float PLATFORM_SIZE = 0.3;
    // Soft
//    const float SCALE_RP = 0.18;
//    const float SCALE_YAW = 0.8;
//    const float SCALE_ALTITUDE = 0.9;
    // Aggressive
//    const float SCALE_RP = 0.23;
//    const float SCALE_YAW = 0.9;
//    const float SCALE_ALTITUDE = 0.8;

    // 8-Shape
//    const float SCALE_RP = 0.2;
//    const float SCALE_YAW = 0.2;
//    const float SCALE_ALTITUDE = 0.7;

    // Runaway
    const float SCALE_RP = 0.2;
    const float SCALE_YAW = 0.2;
    const float SCALE_ALTITUDE = 0.7;

    const float MAX_EMERGENCY_ALTITUDE = 1.0;
    const float PLATFORM_IN_SIZE = 0.5;
#endif
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
#ifdef REAL_FLIGHT_MODE
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
#ifdef PBVS_MODE
    const double CAMERA_X =  0.0;
    const double CAMERA_Y = -0.1;
    const double CAMERA_Z =  0.0;
#else
    const double CAMERA_X =  0.0;
    const double CAMERA_Y =  0.1;
    const double CAMERA_Z =  0.0;
#endif
#else
    const double ARUCO_0_OFFSET_X =  0.4525;     // Camera frame of reference
    const double ARUCO_0_OFFSET_Y = -0.4525;
    const double ARUCO_1_OFFSET_X = -0.4525;
    const double ARUCO_1_OFFSET_Y = -0.4525;
    const double ARUCO_2_OFFSET_X = -0.4525;
    const double ARUCO_2_OFFSET_Y =  0.4525;
    const double ARUCO_3_OFFSET_X =  0.4525;
    const double ARUCO_3_OFFSET_Y =  0.4525;
    const double ARUCO_4_OFFSET_X =  0.0;
    const double ARUCO_4_OFFSET_Y =  0.0;
    const double ARUCO_5_OFFSET_X =  0.2262;
    const double ARUCO_5_OFFSET_Y = -0.2262;
    const double ARUCO_6_OFFSET_X = -0.2262;
    const double ARUCO_6_OFFSET_Y = -0.2262;
    const double ARUCO_7_OFFSET_X = -0.2262;
    const double ARUCO_7_OFFSET_Y =  0.2262;
    const double ARUCO_8_OFFSET_X =  0.2262;
    const double ARUCO_8_OFFSET_Y =  0.2262;
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
#endif

#ifdef ENABLE_RELATIVE_YAW
    float current_relative_yaw_;
#endif

    float current_z_;
    float current_yaw_, current_gazebo_yaw_, current_roll_, current_pitch_, current_bebop_yaw_;
    float prev_x_relative_, prev_y_relative_, prev_x_relative_legacy_, prev_y_relative_legacy_;
    bool landing_executed_, reset_executed_;
    float emergency_altitude_;
    int status_, episode_no_;
    boost::mutex status_mutex_;

    // Ros publishers
    ros::Publisher roll_pitch_ref_pub_, dYaw_ref_pub_, dAltitude_ref_pub_;
    ros::Publisher uav_altitude_ref_pub_, drone_command_publ_, legacy_pub_, current_pub_, camera_control_pub_, moving_platform_publ_;
    image_transport::Publisher aruco_camera_publ_;

    // Ros subscribers
    ros::Subscriber uav_pose_velocity_sub_;
    ros::Subscriber bumper_states_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber bebop_attitude_sub_;
    ros::Subscriber bebop_gt_sub_;
    ros::Subscriber platform_gt_sub_;

    // Ros service clients
    ros::ServiceClient gazebo_client_;
    ros::ServiceClient estimator_client_;

    // Other members
    float uav_pose_x_, uav_pose_y_, uav_pose_z_, uav_velocity_x_, uav_velocity_y_;
    boost::mutex uav_mutex_;

    float platform_pose_x_, platform_pose_y_, platform_pose_z_, platform_velocity_x_, platform_velocity_y_;
    boost::mutex platform_mutex_;

    double relative_pose_x_, relative_pose_y_, relative_pose_z_, relative_velocity_x_, relative_velocity_y_;
    boost::mutex relative_mutex_;

#ifdef PBVS_MODE
    double relative_pose_pbvs_x_, relative_pose_pbvs_y_, relative_pose_pbvs_z_, relative_velocity_pbvs_x_, relative_velocity_pbvs_y_;
    boost::mutex relative_mutex_pbvs_;
#endif

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

#ifdef PBVS_MODE
    cv::Vec3f traslacion_;
    double lambda_, kpx_, kpy_, kpz_, kpwx_, kpwy_, kpwz_;
    double vximu_, vyimu_, vzimu_;
    ros::Time last_time_;
    ros::Subscriber pbvs_speeds_sub_;
    double bodyframeantx,bodyframeanty,bodyframeantz;
    control_toolbox::Pid pidZ_;
    control_toolbox::Pid pidX_;
    control_toolbox::Pid pidY_;
    cv::Vec3f traslaciondeseada_;
#endif

#if defined LOG_DATA || defined LOG_STATISTICS
    std::ofstream outFile_, outFile2_;
#endif


    // Initialize Aruco parameters
#ifdef REAL_FLIGHT_MODE
//     Real Bebop 2 camera
    float camera_params_[9] = { 536.774231, 0.000000, 432.798910, 0.000000, 527.838517, 222.208294, 0.000000, 0.000000, 1.000000 };
    cv::Mat camera_parameters_ = cv::Mat(3, 3, CV_32F, camera_params_);
    float distortion_coeff_[5] = { 0.002459, 0.005719, -0.010226, 0.004865, 0.000000 };
    cv::Mat distortion_coefficients_ = cv::Mat(1, 5, CV_32F, distortion_coeff_);
    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    double bebop_gt_x_, bebop_gt_y_, bebop_gt_z_, platform_gt_x_, platform_gt_y_, platform_gt_z_;

#else

    // Simulated camera FOV 2.3
//    float camera_params_[9] = { 191.54199, 0.00, 428.5, 0.00, 191.5419, 240.5, 0.00, 0.000000, 1.00 };
//    cv::Mat camera_parameters_ = cv::Mat(3, 3, CV_32F, camera_params_);
//    float distortion_coeff_[5] = { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 };
//    cv::Mat distortion_coefficients_ = cv::Mat(1, 5, CV_32F, distortion_coeff_);
//    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    // Simulated camera FOV 1.3463
    float camera_params_[9] = { 536.748344, 0.00, 428.5, 0.00, 536.748344, 240.5, 0.00, 0.000000, 1.00 };
    cv::Mat camera_parameters_ = cv::Mat(3, 3, CV_32F, camera_params_);
    float distortion_coeff_[5] = { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 };
    cv::Mat distortion_coefficients_ = cv::Mat(1, 5, CV_32F, distortion_coeff_);
    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
#endif


    // Experiment recorder
//    ExperimentRecorder exp_rec_;

public:
    // Polymorphic member functions
    bool Step(rl_srvs::AgentSrv::Request &request, rl_srvs::AgentSrv::Response &response);
    bool EnvDimensionality(rl_srvs::EnvDimensionalitySrv::Request &request, rl_srvs::EnvDimensionalitySrv::Response &response);
    bool ResetSrv(rl_srvs::ResetEnvSrv::Request &request, rl_srvs::ResetEnvSrv::Response &response);
    bool Reset();
    void InitChild(ros::NodeHandle n);

    // Member functions
    void PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);    
    void BumperStatesCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
    void PoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg);
    void StatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void attitudeCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
    void generateRotx(double phi, cv::Mat &R);
    void generateRoty(double theta, cv::Mat &R);
    void generateRotz(double chi, cv::Mat &R);
    void generateHomogeneousTransMatrix(cv::Vec3d p, cv::Mat &T);
    void generateHomogeneousTransMatrixFromAruco(cv::Vec3d tvec, cv::Vec3d rvec, cv::Mat &T);
    void alturaCbSim(const droneMsgsROS::droneSpeeds& msg);
    void platformGTCallback(const nav_msgs::Odometry& msg);
    void bebopGTCallback(const nav_msgs::Odometry& msg);



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

#ifdef PBVS_MODE
    void SetRelativeStatePBVS(double x, double y, double z){
        relative_mutex_pbvs_.lock();
        relative_pose_pbvs_x_ = x;
        relative_pose_pbvs_y_ = y;
        relative_pose_pbvs_z_ = z;
        relative_mutex_pbvs_.unlock();
    }

    void GetRelativeStatePBVS(float &x, float &y, float &z){
        relative_mutex_pbvs_.lock();
        x = (float) relative_pose_pbvs_x_;
        y = (float) relative_pose_pbvs_y_;
        z = (float) relative_pose_pbvs_z_;
        relative_mutex_pbvs_.unlock();
    }
#endif

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
