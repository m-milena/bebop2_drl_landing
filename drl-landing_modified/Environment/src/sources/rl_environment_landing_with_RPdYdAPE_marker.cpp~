#include "rl_environment_landing_with_RPdYdAPE_marker.h"

#if HAS_OPENCV3 == 1

bool RlEnvironmentLandingWithRPdYdAPEMarker::Step(rl_srvs::AgentSrv::Request &request, rl_srvs::AgentSrv::Response &response){
#ifdef PBVS_MODE
    // Desired rotations and translation
    cv::Vec3f rotaciondeseadaeuler, rotacioneuler;
    traslaciondeseada_[0] = 0;
    traslaciondeseada_[1] = 0;
    rotaciondeseadaeuler[0] = 0.0;
    rotaciondeseadaeuler[1] = 0.0;
    rotaciondeseadaeuler[2] = 0.0;

    double yaw_pbvs;

#ifdef ENABLE_RELATIVE_YAW
    yaw_pbvs = current_relative_yaw_;
#else
    yaw_pbvs = current_gazebo_yaw_;
#endif

    // Provisional
    rotacioneuler[0] = yaw_pbvs;
    rotacioneuler[1] = 0.0;
    rotacioneuler[2] = 0.0;

    // Get traslation
    float x, y, z;
    GetRelativeStatePBVS(x, y, z);
    traslacion_[0] = x;
    traslacion_[1] = y;
    traslacion_[2] = z;

    // PBVS control law
    cv::Vec6f velocity;
    velocity[0]=-lambda_* ((traslaciondeseada_[0]-traslacion_[0])+ traslacion_[0]*(-rotacioneuler[2]+rotaciondeseadaeuler[2]));
    velocity[1]=-lambda_* ((traslaciondeseada_[1]-traslacion_[1])+ traslacion_[1]*(-rotacioneuler[1]+rotaciondeseadaeuler[1]));
    velocity[2]=-lambda_* ((traslaciondeseada_[2]-traslacion_[2])+ traslacion_[2]*(-rotacioneuler[0]+rotaciondeseadaeuler[0]));
    velocity[3]=-lambda_* (-rotacioneuler[2]+rotaciondeseadaeuler[2]);
    velocity[4]=-lambda_* (-rotacioneuler[1]+rotaciondeseadaeuler[1]);
    velocity[5]=-lambda_* (-rotacioneuler[0]+rotaciondeseadaeuler[0]);

    //            std::cout << "Velocities: " << velocity[0] << std::endl;
    //            std::cout << "Velocities: " << velocity[1] << std::endl;
    //            std::cout << "Velocities: " << velocity[2] << std::endl;

    // Compute velocities in body frame
    double vy = kpy_ * velocity[1];
    double vx = kpx_ * velocity[0];
    double vz = kpz_ * velocity[2];
    double wx = kpwx_ * velocity[3];
    double wy = kpwy_ * velocity[4];
    double wz = kpwz_ * velocity[5];

    Eigen::Vector3f BodyFrame;
    Eigen::Vector3f GlobalFrame;
    Eigen::Vector3f BodyFrameangularpitch;
    Eigen::Vector3f BodyFrameangularyaw;
    Eigen::Vector3f GlobalFrameangular;
    Eigen::Matrix3f RotationMatcameragimbal;

    GlobalFrame(0) = (+1) * vx;
    GlobalFrame(1) = (+1) * vy;
    GlobalFrame(2) = (+1) * vz;

    GlobalFrameangular(0) = wx;
    GlobalFrameangular(1) = wy;
    GlobalFrameangular(2) = wz;

    RotationMatcameragimbal(0,0) = 0;
    RotationMatcameragimbal(1,0) = -1;
    RotationMatcameragimbal(2,0) = 0;

    RotationMatcameragimbal(0,1) =-1;
    RotationMatcameragimbal(1,1) = 0;
    RotationMatcameragimbal(2,1) = 0;

    RotationMatcameragimbal(0,2) = 0;
    RotationMatcameragimbal(1,2) = 0;
    RotationMatcameragimbal(2,2) = -1;

    //BodyFrame =RotationMatdronworld*RotationMatgimbaldron*RotationMatcameragimbal*GlobalFrame;
    BodyFrame =RotationMatcameragimbal*GlobalFrame;
//    BodyFrameangularpitch =RotationMatcameragimbal*GlobalFrameangular;
//    BodyFrameangularyaw= RotationMatcameragimbal*GlobalFrameangular;

//    // PID velocity controller
//    double commandX=pidX_.computeCommand(BodyFrame(0)-vximu_,ros::Time::now()-last_time_);
//    double commandY=pidY_.computeCommand(BodyFrame(1)-vyimu_,ros::Time::now()-last_time_);
//    double commandZ=pidZ_.computeCommand(BodyFrame(2)-vzimu_,ros::Time::now()-last_time_);

      // PID velocity controller
    double commandX=1.45*( 0.05 * BodyFrame(0)+2.6*(BodyFrame(0)-bodyframeantx));
    double commandY=1.45*( 0.05 * BodyFrame(1)+2.6*(BodyFrame(1)-bodyframeanty));
//    double commandX=1.45*(2.1*(BodyFrame(0)-bodyframeantx));
//    double commandY=1.45*(2.*(BodyFrame(1)-bodyframeanty));
    double commandZ= BodyFrame(2);
    double commandDYaw = wz;

    bodyframeantx=BodyFrame(0);
    bodyframeanty=BodyFrame(1);
    std::cout<<"pitch"<<commandX<<std::endl;
    std::cout<<"roll"<<commandY<<std::endl;


    // Store last time variable
    last_time_=ros::Time::now();
 #endif

    // Send roll pitch
    droneMsgsROS::dronePitchRollCmd roll_pitch_msg;
    droneMsgsROS::droneDAltitudeCmd daltitude_msg;
    droneMsgsROS::droneDYawCmd dyaw_msg;

if(!position_lost_){
#ifndef PBVS_MODE
    roll_pitch_msg.rollCmd = SCALE_RP * request.action[0];  // Roll
    roll_pitch_msg.pitchCmd = SCALE_RP * request.action[1];  // Pitch
    // Send yaw
    dyaw_msg.dYawCmd = SCALE_YAW * request.action[2];

#ifdef RUNAWAY
    if ((ros::Time::now() - initial_time_).toSec() < SETTELING_TIME){
        // Send altitude
        daltitude_msg.dAltitudeCmd = SCALE_ALTITUDE * request.action[3];  // dAltitude
    }
    else{
        if ((ros::Time::now() - initial_time_).toSec() > RUNAWAY_TIME){
            // Send altitude
            daltitude_msg.dAltitudeCmd = SCALE_ALTITUDE * request.action[3];  // dAltitude
        }
        else
            // Send altitude
            daltitude_msg.dAltitudeCmd = 0;  // dAltitude
    }
#else
    // Send altitude
    daltitude_msg.dAltitudeCmd = SCALE_ALTITUDE * request.action[3];  // dAltitude
#endif

#else
    roll_pitch_msg.rollCmd = - commandY;  // Roll
    roll_pitch_msg.pitchCmd = - commandX;  // Pitch
    // Send yaw
    dyaw_msg.dYawCmd = commandDYaw;

#ifdef RUNAWAY
    if ((ros::Time::now() - initial_time_).toSec() < SETTELING_TIME){
        // Send altitude
        daltitude_msg.dAltitudeCmd = commandZ;  // dAltitude
    }
    else{
        if ((ros::Time::now() - initial_time_).toSec() > RUNAWAY_TIME){
            // Send altitude
            daltitude_msg.dAltitudeCmd = commandZ;  // dAltitude
            if (fabs(traslaciondeseada_[2]) > 0.5)
                traslaciondeseada_[2] -= 0.01;
        }
        else
            // Send altitude
            daltitude_msg.dAltitudeCmd = 0;  // dAltitude
    }
#else
    // Send altitude
    daltitude_msg.dAltitudeCmd = commandZ;  // dAltitude

    if (fabs(traslaciondeseada_[2]) > 0.5)
        traslaciondeseada_[2] -= 0.01;
#endif


#endif
}
else{
    roll_pitch_msg.rollCmd = 0;  // Roll
    roll_pitch_msg.pitchCmd = 0;  // Pitch
    // Send yaw
    //    dyaw_msg.dYawCmd = 0; // No dYaw
    dyaw_msg.dYawCmd = 0;
    // Send altitude
//        daltitude_msg.dAltitudeCmd = 0;  // dAltitude
    daltitude_msg.dAltitudeCmd = 0.3;  // dAltitude
}

    roll_pitch_ref_pub_.publish(roll_pitch_msg);
    dYaw_ref_pub_.publish(dyaw_msg);
    dAltitude_ref_pub_.publish(daltitude_msg);


//    std::cout << "\troll: " << roll_pitch_msg.rollCmd << "\tpitch: " << roll_pitch_msg.pitchCmd << "\tyaw: " << dyaw_msg.dYawCmd << std::endl;
#ifndef REAL_FLIGHT_MODE
    // Getting state
    float x_uav, y_uav, z_uav, dx_uav, dy_uav;
    GetUavState(x_uav, y_uav, z_uav, dx_uav, dy_uav);

#endif

    float x_relative, y_relative, z_relative, dx_relative, dy_relative;
    GetRelativeState(x_relative, y_relative, z_relative);
    dx_relative = (prev_x_relative_ - x_relative ) / (float) MAX_POSE_X;
    dy_relative = (prev_y_relative_ - y_relative) / (float) MAX_POSE_Y;
    z_relative *= (-1);

    // NOTE: normalization is carried out here
    x_relative /= (float)  MAX_POSE_X;
    y_relative /= (float)  MAX_POSE_Y;
    z_relative = (z_relative + (PLATFORM_SIZE)) / (float)  Z_INITIAL;    // z_relative is slightly different
#ifndef ENABLE_RELATIVE_YAW
#ifdef REAL_FLIGHT_MODE
    float yaw = current_bebop_yaw_;
#else
    float yaw = current_gazebo_yaw_;
#endif
#else
    float yaw = current_relative_yaw_;
#endif


//    std::cout << "z_relative: " << z_relative << std::endl;

    /// Current
    nav_msgs::Odometry current_msg;
    current_msg.header.stamp = ros::Time::now();
    current_msg.pose.pose.position.x = x_relative * (float) MAX_POSE_X;
    current_msg.pose.pose.position.y = y_relative * (float) MAX_POSE_Y;
    current_msg.pose.pose.position.z = z_relative * (float)  Z_INITIAL;
    current_msg.twist.twist.linear.x = dx_relative;
    current_msg.twist.twist.linear.y = dy_relative;
    current_msg.twist.twist.linear.z = yaw;

    current_pub_.publish(current_msg);
    ///

    // Is the UAV on top of the platform?
    int state;
    GetBumperState(state);

#ifndef REAL_FLIGHT_MODE
    /// Ground truth
    float x_uav_legacy, y_uav_legacy, z_uav_legacy, dx_uav_legacy, dy_uav_legacy;
    float x_platform_legacy, y_platform_legacy, z_platform_legacy, dx_platform_legacy, dy_platform_legacy;
    float x_relative_legacy, y_relative_legacy, z_relative_legacy, dx_relative_legacy, dy_relative_legacy;
    GetUavState(x_uav_legacy, y_uav_legacy, z_uav_legacy, dx_uav_legacy, dy_uav_legacy);
    GetPlatformState(x_platform_legacy, y_platform_legacy, z_platform_legacy, dx_platform_legacy, dy_platform_legacy);
    x_relative_legacy = x_platform_legacy - x_uav_legacy;
    y_relative_legacy = y_platform_legacy - y_uav_legacy;
    z_relative_legacy = z_platform_legacy - z_uav_legacy;
    dx_relative_legacy = (prev_x_relative_legacy_ - x_relative_legacy ) / (float) MAX_POSE_X;
    dy_relative_legacy = (prev_y_relative_legacy_ - y_relative_legacy) / (float) MAX_POSE_Y;

    prev_x_relative_legacy_ = x_relative_legacy;
    prev_y_relative_legacy_ = y_relative_legacy;

    // NOTE: normalization is carried out here
    x_relative_legacy /= (float)  MAX_POSE_X;
    y_relative_legacy /= (float)  MAX_POSE_Y;
    z_relative_legacy = z_uav_legacy / (float)  Z_INITIAL;    // z_relative is slightly different

    float yaw_legacy = current_gazebo_yaw_;


    nav_msgs::Odometry legacy_msg;
    legacy_msg.header.stamp = ros::Time::now();
    legacy_msg.pose.pose.position.x = x_relative_legacy * (float) MAX_POSE_X;
    legacy_msg.pose.pose.position.y = y_relative_legacy * (float) MAX_POSE_Y;
    legacy_msg.pose.pose.position.z = z_relative_legacy;
    legacy_msg.twist.twist.linear.x = dx_relative_legacy;
    legacy_msg.twist.twist.linear.y = dy_relative_legacy;
    legacy_msg.twist.twist.linear.z = yaw_legacy;

    legacy_pub_.publish(legacy_msg);
    ///
 #endif

    // Print info
//        std::cout << "RL_ENV_DEBUG: UAV x_relative " << x_relative << " y_relative " << y_relative << std::endl;

    float shaping;

//    if (position_lost_){
//        shaping = - 100 * sqrt(pow(yaw,2)) - 100 * (1 - sqrt(pow(request.action[3], 2)) / SCALE_ALTITUDE);
//    }
//    else{
        // Compute reward
        shaping = - 100 * sqrt(pow(x_relative,2) + pow(y_relative,2) + pow(z_relative,2) + pow(yaw,2))
                - 10 * sqrt(pow(dx_relative, 2) + pow(dy_relative, 2))
                - sqrt(pow(request.action[0], 2) + pow(request.action[1], 2) + pow(request.action[2], 2) + pow(request.action[3], 2));
//    }


    float reward = shaping - prev_shaping_;
    prev_shaping_ = shaping;
    response.reward = reward;

    //    std::cout << "ENV DEBUG: Emergency action " << request.action[4] << std::endl;

    if (reset_executed_ && (request.action[4] < EMERGENCY_THRESHOLD)){
        landing_executed_ = false;
        reset_executed_ = false;
    }
#ifdef REAL_FLIGHT_MODE
    if ((request.action[4] > EMERGENCY_THRESHOLD) && !landing_executed_ && !position_lost_ && (fabs(x_relative * (float) MAX_POSE_X) < PLATFORM_IN_SIZE) && (fabs(y_relative * (float) MAX_POSE_Y) < PLATFORM_IN_SIZE)){
        if (!ENABLE_EMERGENCY_INHIBITION || ((z_relative * (float) Z_INITIAL) < MAX_EMERGENCY_ALTITUDE)){
            landing_executed_ = true;
            emergency_altitude_ = z_relative * (float) Z_INITIAL;
            std::cout << "ENV DEBUG: Emergency altitude " << emergency_altitude_ << std::endl;
            droneMsgsROS::droneCommand msg_command;
            msg_command.command = droneMsgsROS::droneCommand::RESET;
            drone_command_publ_.publish(msg_command);
        }
    }
#else
    if ((request.action[4] > EMERGENCY_THRESHOLD) && !landing_executed_&& (fabs(x_relative * (float) MAX_POSE_X) < PLATFORM_IN_SIZE) && (fabs(y_relative * (float) MAX_POSE_Y) < PLATFORM_IN_SIZE)){
        if (!ENABLE_EMERGENCY_INHIBITION || ((z_relative * (float) Z_INITIAL) < MAX_EMERGENCY_ALTITUDE)){
            landing_executed_ = true;
            emergency_altitude_ = z_relative * (float) Z_INITIAL;
            std::cout << "ENV DEBUG: Emergency altitude " << emergency_altitude_ << std::endl;
            droneMsgsROS::droneCommand msg_command;
            msg_command.command = droneMsgsROS::droneCommand::RESET;
            drone_command_publ_.publish(msg_command);
        }
    }
#endif

    // If aruco position is lost below a certain altitude it does not matter
//    if (((z_relative * (float) Z_INITIAL) < MAX_EMERGENCY_ALTITUDE)){
//        position_lost_ = 0;
//    }

#ifndef REAL_FLIGHT_MODE
#ifndef RUNAWAY
#if !defined LOG_DATA && !defined LOG_STATISTICS
    // Is it terminal?
    if ((fabs(x_uav / ((float) MAX_POSE_X / 2)) > 1) || (fabs(y_uav / ((float) MAX_POSE_Y / 2)) > 1) || (z_uav) > MAX_ALLOWED_ALTIUDE /*|| position_lost_*/){
#else
    // Is it terminal?
    if ((fabs(x_uav / ((float) MAX_POSE_X)) > 1) || (fabs(y_uav / ((float) MAX_POSE_Y)) > 1) || (z_uav) > MAX_ALLOWED_ALTIUDE /*|| position_lost_*/){
#endif
#else
    // Is it terminal?
    if ((z_uav) > MAX_ALLOWED_ALTIUDE /*|| position_lost_*/){
#endif
        response.reward = -50;
        response.terminal_state = true;
        reset_env_ = true;
#ifdef LOG_STATISTICS
    // Log experiment stats
    outFile2_.open (EXPERIMENTS_FOLDER + "iros_2018_landing_stats.csv", std::ios::out | std::ios::ate | std::ios::app);
    std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    );
    outFile2_ << episode_no_ << "," << ms.count() << "," << 0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << std::endl;
    outFile2_.close();
#endif
    }
    else if(((z_uav) <= 0.1) && (state == 0)){
        response.reward = -50;
        response.terminal_state = true;
        reset_env_ = true;

#ifdef LOG_STATISTICS
    // Log experiment stats
    outFile2_.open (EXPERIMENTS_FOLDER + "iros_2018_landing_stats.csv", std::ios::out | std::ios::ate | std::ios::app);
    std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    );
    outFile2_ << episode_no_ << "," << ms.count() << "," << 0 << "," << 0.0 << "," << 0.0 << "," << emergency_altitude_ << std::endl;
    outFile2_.close();
#endif
    }
    else if((landing_executed_) && (state == 1)){
        if (emergency_altitude_ < MAX_EMERGENCY_ALTITUDE){
            float MAX_REWARD = 100.0;
            float MIN_REWARD = 50.0;
            float K = std::log(MIN_REWARD / MAX_REWARD) / (DESIRED_EMERGENCY_ALTITUDE - MAX_EMERGENCY_ALTITUDE);
            response.reward = MAX_REWARD * std::exp(K * (DESIRED_EMERGENCY_ALTITUDE - emergency_altitude_));
            std::cout << "ENV DEBUG: reward " << response.reward << std::endl;
        }
        else{
            response.reward = -50;
        }

        response.terminal_state = true;
        reset_env_ = true;

#ifdef LOG_STATISTICS
    // Log experiment stats
    outFile2_.open (EXPERIMENTS_FOLDER + "iros_2018_landing_stats.csv", std::ios::out | std::ios::ate | std::ios::app);
    std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    );
    outFile2_ << episode_no_ << "," << ms.count() << "," << 1 << "," << x_relative * (float) MAX_POSE_X << "," << y_relative * (float) MAX_POSE_Y << "," << emergency_altitude_ << std::endl;
    outFile2_.close();
#endif

    }
    else
#endif
        response.terminal_state = false;

    // Wait for environment to render
    if (ENABLE_PAUSED_SIMULATION)
        WaitForEnv();

    // Read next state
    std::vector<float> state_next;
    GetRelativeState(x_relative, y_relative, z_relative);

    dx_relative = (prev_x_relative_ - x_relative) / (float) MAX_POSE_X;
    dy_relative = (prev_y_relative_ - y_relative) / (float) MAX_POSE_Y;
    z_relative *= (-1);

    // Save previous state
    prev_x_relative_ = x_relative;
    prev_y_relative_ = y_relative;

    // NOTE: normalization is carried out here
    x_relative /= (float)  MAX_POSE_X;
    y_relative /= (float)  MAX_POSE_Y;
    z_relative = (z_relative + (PLATFORM_SIZE)) / (float)  Z_INITIAL;    // z_relative is slightly different

    state_next.push_back(x_relative);
    state_next.push_back(y_relative);
    state_next.push_back(dx_relative);
    state_next.push_back(dy_relative);
    state_next.push_back(z_relative);
    state_next.push_back((float) state);
    state_next.push_back(yaw);
//    state_next.push_back((float) position_lost_);


    if (!std::isfinite(state_next[0]) /*|| fabs(s[0]) > 1*/){
        std::cout << "rSTATE 0 LIANDOLA!" << state_next[0] << std::endl;
        state_next[0] = 0;
    }
    if (!std::isfinite(state_next[1]) /*|| fabs(s[1]) > 1*/ ){
        std::cout << "rSTATE 1 LIANDOLA!" << state_next[1] << std::endl;
        state_next[1] = 0;
    }
    if (!std::isfinite(state_next[2]) /*|| fabs(s[2]) > 1*/ ){
        std::cout << "rSTATE 2 LIANDOLA!" << state_next[2] << std::endl;
        state_next[2] = 0;
    }
    if (!std::isfinite(state_next[3]) /*|| fabs(s[3]) > 1*/ ){
        std::cout << "rSTATE 3 LIANDOLA!" << state_next[3] << std::endl;
        state_next[3] = 0;
    }
    if (!std::isfinite(state_next[4]) /*|| fabs(s[4]) > 1*/ ){
        std::cout << "rSTATE 4 LIANDOLA!" << state_next[4] << std::endl;
        state_next[4] = 0;
    }
    if (!std::isfinite(state_next[5]) /*|| fabs(s[5]) > 1*/ ){
        std::cout << "rSTATE 5 LIANDOLA!" << state_next[5] << std::endl;
        state_next[5] = 0;
    }
    if (!std::isfinite(state_next[6])/* || fabs(s[6]) > 1 */){
        std::cout << "rSTATE 6 LIANDOLA!" << state_next[6] << std::endl;
        state_next[6] = 0;
    }
//    if (!std::isfinite(state_next[7]) /*|| fabs(s[7]) > 1*/ ){
//        std::cout << "rSTATE 7 LIANDOLA!" << state_next[7] << std::endl;
//        state_next[7] = 0;
//    }
//    if (!std::isfinite(state_next[8]) /*|| fabs(s[8]) > 1*/ ){
//        std::cout << "rSTATE 8 LIANDOLA!" << state_next[8] << std::endl;
//        state_next[8] = 0;
//    }

    response.obs_real = state_next;

//        std::cout << x_relative << "\t" << y_relative << "\t" << dx_relative << "\t" << dy_relative << std::endl;

    // Successful return
    return true;
}

bool RlEnvironmentLandingWithRPdYdAPEMarker::EnvDimensionality(rl_srvs::EnvDimensionalitySrv::Request &request, rl_srvs::EnvDimensionalitySrv::Response &response){
    // Print info
    std::cout << "RL_ENV_INFO: EnvDimensionality service called" << std::endl;

    // Action dimensionality
    response.action_dim = ACTIONS_DIM;

    // Action max
    std::vector<float> actions_max = {1.0, 1.0, 1.0, 1.0, 1.0};
    response.action_max = actions_max;

    // Action min
    std::vector<float> actions_min = {-1.0, -1.0, -1.0, -1.0, -1.0};
    response.action_min = actions_min;

    // States dimensionality
    response.state_dim_lowdim = STATE_DIM_LOW_DIM;

    // Number of iterations
    response.num_iterations = NUM_EPISODE_ITERATIONS;


    // Service succesfully executed
    return true;
}

bool RlEnvironmentLandingWithRPdYdAPEMarker::ResetSrv(rl_srvs::ResetEnvSrv::Request &request, rl_srvs::ResetEnvSrv::Response &response){
    // Print info
    std::cout << "RL_ENV_INFO: ResetSrv service called" << std::endl;

    // Enable reset
    if (ENABLE_PAUSED_SIMULATION)
        reset_env_ = true;
    else
        Reset();

    // Is it long iteration?
    //    if (exp_rec_.getRecording())
    //        long_iteration_ = true;


    int state;
    GetBumperState(state);

    std::vector<float> s;
    float x_relative, y_relative, z_relative, dx_relative, dy_relative;
    GetRelativeState(x_relative, y_relative, z_relative);
    dx_relative = (prev_x_relative_ - x_relative ) / (float) MAX_POSE_X;
    dy_relative = (prev_y_relative_ - y_relative) / (float) MAX_POSE_Y;
    z_relative *= (-1);
    // NOTE: normalization is carried out here
    x_relative /= (float)  MAX_POSE_X;
    y_relative /= (float)  MAX_POSE_Y;
    z_relative = (z_relative + (PLATFORM_SIZE)) / (float)  Z_INITIAL;    // z_relative is slightly different

    dx_relative = (x_relative - prev_x_relative_) / (float) MAX_POSE_X;
    dy_relative = (y_relative - prev_y_relative_) / (float) MAX_POSE_Y;

    // Save previous state
    prev_x_relative_ = x_relative;
    prev_y_relative_ = y_relative;

    // NOTE: normalization is carried out here
    x_relative /= (float)  MAX_POSE_X;
    y_relative /= (float)  MAX_POSE_Y;
    z_relative = (z_relative + (PLATFORM_SIZE)) / (float)  Z_INITIAL;    // z_relative is slightly different
    s.push_back(x_relative);
    s.push_back(y_relative);
    s.push_back(dx_relative);
    s.push_back(dy_relative);
    s.push_back(z_relative);
    s.push_back((float) state);
#ifndef ENABLE_RELATIVE_YAW
#ifdef REAL_FLIGHT_MODE
    s.push_back(current_bebop_yaw_);
#else
    s.push_back(current_gazebo_yaw_);
#endif
#else
    s.push_back(current_relative_yaw_);
#endif
//    s.push_back((float) position_lost_);


    if (!std::isfinite(s[0]) /*|| fabs(s[0]) > 1*/){
        std::cout << "rSTATE 0 LIANDOLA!" << s[0] << std::endl;
        s[0] = 0;
    }
    if (!std::isfinite(s[1]) /*|| fabs(s[1]) > 1*/ ){
        std::cout << "rSTATE 1 LIANDOLA!" << s[1] << std::endl;
        s[1] = 0;
    }
    if (!std::isfinite(s[2]) /*|| fabs(s[2]) > 1*/ ){
        std::cout << "rSTATE 2 LIANDOLA!" << s[2] << std::endl;
        s[2] = 0;
    }
    if (!std::isfinite(s[3]) /*|| fabs(s[3]) > 1*/ ){
        std::cout << "rSTATE 3 LIANDOLA!" << s[3] << std::endl;
        s[3] = 0;
    }
    if (!std::isfinite(s[4]) /*|| fabs(s[4]) > 1*/ ){
        std::cout << "rSTATE 4 LIANDOLA!" << s[4] << std::endl;
        s[4] = 0;
    }
    if (!std::isfinite(s[5]) /*|| fabs(s[5]) > 1*/ ){
        std::cout << "rSTATE 5 LIANDOLA!" << s[5] << std::endl;
        s[5] = 0;
    }
    if (!std::isfinite(s[6])/* || fabs(s[6]) > 1 */){
        std::cout << "rSTATE 6 LIANDOLA!" << s[6] << std::endl;
        s[6] = 0;
    }
//    if (!std::isfinite(s[7]) /*|| fabs(s[7]) > 1*/ ){
//        std::cout << "rSTATE 7 LIANDOLA!" << s[7] << std::endl;
//        s[7] = 0;
//    }
//    if (!std::isfinite(s[8]) /*|| fabs(s[8]) > 1*/ ){
//        std::cout << "rSTATE 8 LIANDOLA!" << s[8] << std::endl;
//        s[8] = 0;
//    }

    response.state = s;

    return true;
}

void RlEnvironmentLandingWithRPdYdAPEMarker::PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    // Get position of the uav in the vector
    int i_position;
    for (int i=0; i<msg->name.size(); i++){
        if (msg->name[i].compare(UAV_NAME) == 0){
            i_position = i;
            break;
        }
    }
    int i_uav = i_position;

    // Set uav state
    SetUavState(msg->pose[i_position].position.x /*/ (float) MAX_POSE_X*/, msg->pose[i_position].position.y /*/ (float) MAX_POSE_Y*/, msg->pose[i_position].position.z, msg->twist[i_position].linear.x, msg->twist[i_position].linear.y);

    // Compute yaw
    // Calculating Roll, Pitch, Yaw
    tf::Quaternion q(msg->pose[i_position].orientation.x, msg->pose[i_position].orientation.y, msg->pose[i_position].orientation.z, msg->pose[i_position].orientation.w);
    tf::Matrix3x3 m(q);

    //convert quaternion to euler angels
    double y, p, r;
    m.getEulerYPR(y, p, r);

    current_gazebo_yaw_ = ((float) y /*+ (float) M_PI / 2.0*/)/ (float) M_PI; // With Noise
    current_roll_ = (float) r / (float) M_PI;
    current_pitch_ = (float) p / (float) M_PI;

//    std::cout << "Yaw: " << current_gazebo_yaw_ * (float) M_PI << "\tRoll: " << current_roll_ * (float) M_PI << "\tPitch: " << current_pitch_ * (float) M_PI << std::endl;

#ifdef PUBLISH_TF
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose[i_position].position.x, msg->pose[i_position].position.y, msg->pose[i_position].position.z) );
    tf::Quaternion q2;
    q2.setX(msg->pose[i_position].orientation.x);
    q2.setY(msg->pose[i_position].orientation.y);
    q2.setZ(msg->pose[i_position].orientation.z);
    q2.setW(msg->pose[i_position].orientation.w);
    transform.setRotation(q2);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav_gazebo"));
#endif

#ifdef LOG_DATA
    // Log experiment data
    outFile_.open (EXPERIMENTS_FOLDER  + "iros_2018_ground_truth.csv", std::ios::out | std::ios::ate | std::ios::app);
    std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    );
    outFile_ << episode_no_ << "," << ms.count() << "," << msg->pose[i_position].position.x << "," << msg->pose[i_position].position.y << "," << msg->pose[i_position].position.z;
    outFile_ << "," << msg->pose[i_position].orientation.x << "," << msg->pose[i_position].orientation.y << "," << msg->pose[i_position].orientation.z << "," << msg->pose[i_position].orientation.w;

#endif


    // Get position of the platform in the vector
    for (int i=0; i<msg->name.size(); i++){
        if (msg->name[i].compare(PLATFORM_NAME) == 0){
            i_position = i;
            break;
        }
    }

    // Set uav state
    SetPlatformState(msg->pose[i_position].position.x /*/ (float) MAX_POSE_X*/, msg->pose[i_position].position.y /*/ (float) MAX_POSE_Y*/, msg->pose[i_position].position.z + Z_PLATFORM_OFFSET, msg->twist[i_position].linear.x, msg->twist[i_position].linear.y);

#ifdef LOG_DATA
    outFile_ << "," << msg->pose[i_position].position.x << "," << msg->pose[i_position].position.y << "," << msg->pose[i_position].position.z;
    outFile_ << "," << msg->pose[i_position].orientation.x << "," << msg->pose[i_position].orientation.y << "," << msg->pose[i_position].orientation.z << "," << msg->pose[i_position].orientation.w << std::endl;
    outFile_.close();
#endif

#ifdef PUBLISH_TF
    transform.setOrigin( tf::Vector3(msg->pose[i_position].position.x, msg->pose[i_position].position.y, msg->pose[i_position].position.z) );
    q2.setX(msg->pose[i_position].orientation.x);
    q2.setY(msg->pose[i_position].orientation.y);
    q2.setZ(msg->pose[i_position].orientation.z);
    q2.setW(msg->pose[i_position].orientation.w);
    transform.setRotation(q2);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "platform_gazebo"));
#endif

}

void RlEnvironmentLandingWithRPdYdAPEMarker::BumperStatesCallback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    if (msg->states.size() > 0){
        float force = msg->states[0].wrenches[0].force.z;
        int state = 0;
        if (msg->states.size() == 2){
            std::string collision = msg->states[0].collision1_name;
            std::size_t found = collision.find(UAV_NAME);
            if (found!=std::string::npos){
                if (force > 3){
                    state = 1;
                }
            }
        }

        if (state == 1)     std::cout << "RL_ENV_DEBUG: Bumper state: " << state << std::endl;

        // Set uav state
        SetBumperState(state);

    }
}

void RlEnvironmentLandingWithRPdYdAPEMarker::PoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg){
    current_yaw_ = (float) msg->yaw / (float) M_PI;
//    current_roll_ = (float) msg->roll / (float) M_PI;
//    current_pitch_ = (float) msg->pitch / (float) M_PI;

//    std::cout << "Yaw: " << current_yaw_ * (float) M_PI << "\tRoll: " << current_roll_ * (float) M_PI << "\tPitch: " << current_pitch_ * (float) M_PI << std::endl;

}

void RlEnvironmentLandingWithRPdYdAPEMarker::StatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg){
    //    std::cout << msg->status << std::endl;
    SetStatus((int) msg->status);
}

void RlEnvironmentLandingWithRPdYdAPEMarker::attitudeCallback(const geometry_msgs::Vector3StampedConstPtr& msg){
    if(initial_yaw_){
        initial_yaw_ = false;
        initial_yaw_value_ = (float) (-1) * msg->vector.z / 180.0;

#ifdef REAL_FLIGHT_MODE
    // Move gimbal
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = -90;
    msg.angular.z = 0;
    camera_control_pub_.publish(msg);
#endif
    }
    current_bebop_yaw_ = (float) (-1) * msg->vector.z / 180.0;
    current_bebop_yaw_ -= initial_yaw_value_;

//    std::cout << "yaw: " << current_bebop_yaw_ << std::endl;
}

void RlEnvironmentLandingWithRPdYdAPEMarker::InitChild(ros::NodeHandle n){
    // Print info
    std::cout << "RL_ENV_INFO: LANDING MARKER ENVIRONMENT" << std::endl;

    int drone_namespace = -1;
    ros::param::get("~droneId", drone_namespace);
    if(drone_namespace == -1)
    {
        ROS_ERROR("FATAL: Namespace not found");
        return;
    }

    // Init subscribers
    std::string param_string;
    ros::param::get("~model_states", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    uav_pose_velocity_sub_ = n.subscribe(param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::PoseVelocityCallback, this);

    ros::param::get("~bumper_states", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    bumper_states_sub_ = n.subscribe(param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::BumperStatesCallback, this);

    ros::param::get("~uav_pose", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    pose_sub_ = n.subscribe("/drone" + std::to_string(drone_namespace) + "/" + param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::PoseCallback, this);

    ros::param::get("~status", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    status_sub_ = n.subscribe("/drone" + std::to_string(drone_namespace) + "/" + param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::StatusCallback, this);
#ifdef REAL_FLIGHT_MODE
    ros::param::get("~camera", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    image_sub_ = n.subscribe("/drone" + std::to_string(drone_namespace) + "/" + param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::imageCallback, this);
#else
    ros::param::get("~camera", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    image_sub_ = n.subscribe(param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::imageCallback, this);
#endif

#ifdef REAL_FLIGHT_MODE
    ros::param::get("~bebop_attitude", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    bebop_attitude_sub_ = n.subscribe("/drone" + std::to_string(drone_namespace) + "/" + param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::attitudeCallback, this);

    ros::param::get("~bebop_gt", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    bebop_gt_sub_ = n.subscribe(param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::bebopGTCallback, this);

    ros::param::get("~platform_gt", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    platform_gt_sub_ = n.subscribe(param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::platformGTCallback, this);

#endif

    // Init publishers
    ros::param::get("~roll_pitch", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    roll_pitch_ref_pub_ = n.advertise<droneMsgsROS::dronePitchRollCmd>("/drone" + std::to_string(drone_namespace) + "/" + param_string, 1000);

    ros::param::get("~dYaw", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    dYaw_ref_pub_ = n.advertise<droneMsgsROS::droneDYawCmd>("/drone" + std::to_string(drone_namespace) + "/" + param_string, 1000);

    ros::param::get("~dAltitude", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    dAltitude_ref_pub_ = n.advertise<droneMsgsROS::droneDAltitudeCmd>("/drone" + std::to_string(drone_namespace) + "/" + param_string, 1000);

    ros::param::get("~position_refs", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    uav_altitude_ref_pub_ = n.advertise<droneMsgsROS::dronePositionRefCommandStamped>("/drone" + std::to_string(drone_namespace) + "/" + param_string, 1000);

    ros::param::get("~control_mode", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    drone_command_publ_= n.advertise<droneMsgsROS::droneCommand>("/drone" + std::to_string(drone_namespace) + "/" + param_string, 1000);

    ros::param::get("~aruco_camera", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    image_transport::ImageTransport it(n);
    aruco_camera_publ_ = it.advertise("/drone" + std::to_string(drone_namespace) + "/" + param_string, 1000);

#if !defined REAL_FLIGHT_MODE && defined RUNAWAY
    ros::param::get("~moving_platform", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    moving_platform_publ_= n.advertise<std_msgs::Bool>(param_string, 1000);
#endif

#ifdef REAL_FLIGHT_MODE
    ros::param::get("~camera_control", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    camera_control_pub_= n.advertise<geometry_msgs::Twist>("/drone" + std::to_string(drone_namespace) + "/" + param_string, 1000);
#endif

#ifdef PBVS_MODE
    ros::param::get("~lambda", lambda_);
    ros::param::get("~kpx", kpx_);
    ros::param::get("~kpy", kpy_);
    ros::param::get("~kpz", kpz_);
    ros::param::get("~kpwx", kpwx_);
    ros::param::get("~kpwy", kpwy_);
    ros::param::get("~kpwz", kpwz_);
    bodyframeantx=0;
    bodyframeanty=0;
    std::cout << "RL_ENV_INFO: LAMBDA: " << lambda_ << std::endl;

    ros::param::get("~uav_speed", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    pbvs_speeds_sub_ = n.subscribe("/drone" + std::to_string(drone_namespace) + "/" + param_string, 10, &RlEnvironmentLandingWithRPdYdAPEMarker::alturaCbSim, this);
#endif

    /// Ground truth
    legacy_pub_ = n.advertise<nav_msgs::Odometry>("legacy_pose", 1000);
    current_pub_ = n.advertise<nav_msgs::Odometry>("current_pose", 1000);
    ///


    // Init service
    ros::param::get("~set_model_state", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    gazebo_client_ = n.serviceClient<gazebo_msgs::SetModelState>(param_string);

    ros::param::get("~reset_estimator", param_string);
    if(param_string.length() == 0)
    {
        ROS_ERROR("FATAL: Topic not found");
        return;
    }
    estimator_client_ = n.serviceClient<std_srvs::Empty>("/drone" + std::to_string(drone_namespace) + "/" + param_string);

    // Set number of iterations of Gazebo
    //    this->data_->num_iterations = (unsigned int) NUM_ITERATIONS;

    //    // Print
    //    std::cout << "RL_ENV_INFO: for each step, Gazebo iterations are " << this->data_->num_iterations << " iterations" << std::endl;

    // Init recording experiment
    //    exp_rec_.Open(n);

    SetStatus(droneMsgsROS::droneStatus::FLYING);

    episode_no_ = 0;
    prev_x_relative_ = 0;
    prev_y_relative_ = 0;
    initial_yaw_ = true;
#ifdef ENABLE_RELATIVE_YAW
    current_relative_yaw_ = 0;
#endif

#ifdef LOG_DATA
#ifdef REAL_FLIGHT_MODE
    outFile_.open (EXPERIMENTS_FOLDER + "iros_2018_ground_truth.csv", std::ios::out | std::ios::ate | std::ios::app);
    outFile_ << "episode,time_ms,x_uav,y_uav,z_uav,x_mp,y_mp,z_mp" << std::endl;
    outFile_.close();
#else
    outFile_.open (EXPERIMENTS_FOLDER + "iros_2018_ground_truth.csv", std::ios::out | std::ios::ate | std::ios::app);
    outFile_ << "episode,time_ms,x_uav,y_uav,z_uav,x_ori_uav,y_ori_uav,z_ori_uav,w_ori_uav,x_mp,y_mp,z_mp,x_ori_mp,y_ori_mp,z_ori_mp,w_ori_mp" << std::endl;
    outFile_.close();
#endif
#endif

#ifdef LOG_STATISTICS
    outFile2_.open (EXPERIMENTS_FOLDER + "iros_2018_landing_stats.csv", std::ios::out | std::ios::ate | std::ios::app);
    outFile2_ << "episode,time_ms,success,x_relative,y_relative,emergency_altitude" << std::endl;
    outFile2_.close();
#endif

#ifdef PBVS_MODE
    last_time_ = ros::Time::now();

    pidX_.initPid(0.45,0.002,0.0015,0.5,-0.5);
    pidY_.initPid(0.75,0.002,0.0015,0.4,-0.4);
    pidZ_.initPid(0.62,0.002,0.002,0.6,-0.6);
    traslaciondeseada_[2] = Z_INITIAL;

#endif
    // Reset environment
    Reset();
}

bool RlEnvironmentLandingWithRPdYdAPEMarker::Reset(){

#ifndef REAL_FLIGHT_MODE

    // Check experiment recorder
    //    if(exp_rec_.getRecording()){
    //        exp_rec_.RecordExperiment();
    //        exp_rec_.setRecording(false);
    //    }

    // Take off
    if (landing_executed_){
        droneMsgsROS::droneCommand msg_command;
        msg_command.command = droneMsgsROS::droneCommand::TAKE_OFF;
        drone_command_publ_.publish(msg_command);
    }

    // Variables init
    prev_x_relative_ = 0;
    prev_y_relative_= 0;
    prev_x_relative_legacy_ = 0;
    prev_y_relative_legacy_= 0;


    // Get state
    float x, y, z, dx, dy;
    GetUavState(x, y, z, dx, dy);


    // Send null action
    // Send roll pitch
    droneMsgsROS::dronePitchRollCmd roll_pitch_msg;
    roll_pitch_msg.rollCmd = 0;  // Roll
    roll_pitch_msg.pitchCmd = 0;  // Pitch
    roll_pitch_ref_pub_.publish(roll_pitch_msg);

    // Send yaw
    droneMsgsROS::droneDYawCmd dyaw_msg;
    dyaw_msg.dYawCmd = 0; // No dYaw
    dYaw_ref_pub_.publish(dyaw_msg);

    // Send yaw
    droneMsgsROS::droneDAltitudeCmd daltitude_msg;
    daltitude_msg.dAltitudeCmd = 0;  // dAltitude
    dAltitude_ref_pub_.publish(daltitude_msg);

    // Random initial point
    std::srand(time(NULL));
    int seed = std::rand() % 10;
    int r_x = ((-1 * (((int) MAX_POSE_X - 1) * 100 / 2)) + (std::rand() % (((int) MAX_POSE_X - 1) * 100 / 2)));
    int r_y = ((-1 * (((int) MAX_POSE_Y - 1) * 100 / 2)) + (std::rand() % (((int) MAX_POSE_Y - 1)* 100 / 2)));
    if (seed < 3){
        r_x *= -1;
        r_y *= -1;
    }
    else if(seed < 6){
        r_x *= -1;
        r_y *= 1;
    }
    else if(seed < 8){
        r_x *= 1;
        r_y *= -1;
    }
    else{
        r_x *= 1;
        r_y *= 1;
    }

    float init_x = (float) r_x / 100.f;
    float init_y = (float) r_y / 100.f;

    init_x = 0.0;
    init_y = 0.0;

    // Set initial altitude
    //    droneMsgsROS::dronePositionRefCommandStamped altitude_msg;
    //    altitude_msg.header.stamp.sec = 0;
    //    altitude_msg.header.stamp.nsec = 0;
    //    altitude_msg.header.frame_id = "";
    //    altitude_msg.header.seq = 0;
    //    altitude_msg.position_command.x = 0;
    //    altitude_msg.position_command.y = 0;
    //    altitude_msg.position_command.z = Z_INITIAL;
    //    uav_altitude_ref_pub_.publish(altitude_msg);

    // Reset variable
    current_z_ = Z_INITIAL;
    current_yaw_ = 0;
    current_gazebo_yaw_ = 0;

    float init_z = Z_INITIAL;

    // If enable guided states
    if(ENABLE_GUIDED_STATES && (episode_no_ % GUIDED_STATES_INTERVAL == 0)){
        float x_platform, y_platform, z_platform, dx_platform, dy_platform;
        GetPlatformState(x_platform, y_platform, z_platform, dx_platform, dy_platform);
        init_x = x_platform;
        init_y = y_platform;
        init_z = 1.7;
    }

    // Reseting environemnt
    std::cout << "RL_ENV_INFO: initial altitude: " << Z_INITIAL << std::endl;

    // Spawn UAV to origin
    gazebo_msgs::SetModelState model_msg;
    model_msg.request.model_state.model_name = UAV_NAME;
    model_msg.request.model_state.pose.position.x = init_x;
    model_msg.request.model_state.pose.position.y = init_y;
    model_msg.request.model_state.pose.position.z = init_z /*+ 2.0*/;

    // Reseting environemnt
    std::cout << "RL_ENV_INFO: reseting environment in x: " << init_x << " and y " << init_y << std::endl;

    if (gazebo_client_.call(model_msg)){
        //        return true;
    }
    else{
        ROS_ERROR("RL_ENV_INFO: Failed to call set model state");
        //        return false;
    }

    // Spawn model to origin
    std_srvs::Empty estimator_msg;

    if (estimator_client_.call(estimator_msg)){
        ROS_INFO("RL_ENV_INFO: Reseting estimator..");
    }
    else{
        ROS_ERROR("RL_ENV_INFO: Failed to call estimator");
        //        return false;
    }

    // Move
    if (landing_executed_){
        droneMsgsROS::droneCommand msg_command;
        msg_command.command = droneMsgsROS::droneCommand::MOVE;
        drone_command_publ_.publish(msg_command);
    }

#ifdef RUNAWAY
    // Reset moving platform
    std_msgs::Bool bool_msg;
    bool_msg.data = true;
    moving_platform_publ_.publish(bool_msg);

    // Init time
    initial_time_ = ros::Time::now();
#endif

#ifdef PBVS_MODE
    traslaciondeseada_[2] = Z_INITIAL;
    bodyframeantx=0;
    bodyframeanty=0;
#endif

#else
    // Set uav state
    SetBumperState(0);

    // Variables init
    prev_x_relative_ = 0;
    prev_y_relative_= 0;
    prev_x_relative_legacy_ = 0;
    prev_y_relative_legacy_= 0;

    // Reset variable
    current_z_ = Z_INITIAL;
    current_yaw_ = 0;
    current_gazebo_yaw_ = 0;
    current_bebop_yaw_ = 0;

#ifdef RUNAWAY
    // Init time
    initial_time_ = ros::Time::now();
#endif

#endif
    // Init variable
    reset_executed_ = true;
    episode_no_++;
    position_lost_ = 0;
    SetRelativeState(0.0, 0.0, -Z_INITIAL);

    // Show episode information
    std::cout << "RL_ENV_INFO: Episode number " << episode_no_ << std::endl;


    return true;
}

void RlEnvironmentLandingWithRPdYdAPEMarker::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);


        // if at least one marker detected
        if (ids.size() > 0){
            if (SHOW_IMAGES){
                cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
            }
            std::vector<cv::Vec3d> rvecs, tvecs, rvecs_small, tvecs_small;
            cv::Vec3d accum_tvec(0.0,0.0,0.0);
#ifdef PBVS_MODE
            cv::Vec3d accum_tvec_pbvs(0.0,0.0,0.0);
#endif
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_parameters_, distortion_coefficients_, rvecs, tvecs);
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE_SMALL, camera_parameters_, distortion_coefficients_, rvecs_small, tvecs_small);

            // Generate Rotation matrices
            cv::Mat Rotx180 = cv::Mat(4, 4, CV_64F);
#ifdef REAL_FLIGHT_MODE
            generateRotx(3.037, Rotx180);
#else
            generateRotx(M_PI, Rotx180);
#endif

            cv::Mat Rotz90 = cv::Mat(4, 4, CV_64F);
            generateRotz(M_PI / 2, Rotz90);

#ifdef REAL_FLIGHT_MODE
            cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
            generateRotz((double) (current_bebop_yaw_ * M_PI), RotzYaw);
#else
            cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
            generateRotz((double) (current_gazebo_yaw_ * M_PI), RotzYaw);

            cv::Mat RotyPitch = cv::Mat(4, 4, CV_64F);
            generateRoty((double) (current_pitch_ * M_PI), RotyPitch);

            cv::Mat RotxRoll = cv::Mat(4, 4, CV_64F);
            generateRotx((double) (current_roll_ * M_PI), RotxRoll);
#endif

            cv::Vec3d camera_position(CAMERA_X,CAMERA_Y,CAMERA_Z);
            cv::Mat TCamera = cv::Mat(4, 4, CV_64F);
            generateHomogeneousTransMatrix(camera_position, TCamera);

            if (tvecs.size() > 0){
                double cnt = 0;
#ifdef ENABLE_RELATIVE_YAW
                double accum_relative_yaw = 0;
#endif
                // Compute relative position of each Aruco
                for (uint i = 0; i < tvecs.size(); i++){
                    if (ids[i] == ARUCO_0_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs[i], rvecs[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_0_OFFSET_X, ARUCO_0_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif

                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else if (ids[i] == ARUCO_1_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs[i], rvecs[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_1_OFFSET_X, ARUCO_1_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif

                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else if (ids[i] == ARUCO_2_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs[i], rvecs[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_2_OFFSET_X, ARUCO_2_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif

                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else if (ids[i] == ARUCO_3_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs[i], rvecs[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_3_OFFSET_X, ARUCO_3_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif

                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else if (ids[i] == ARUCO_4_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs[i], rvecs[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_4_OFFSET_X, ARUCO_4_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif

                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else if (ids[i] == ARUCO_5_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs_small[i], rvecs_small[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_5_OFFSET_X, ARUCO_5_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif

                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else if (ids[i] == ARUCO_6_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs_small[i], rvecs_small[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_6_OFFSET_X, ARUCO_6_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif

                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else if (ids[i] == ARUCO_7_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs_small[i], rvecs_small[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_7_OFFSET_X, ARUCO_7_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif

                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else if (ids[i] == ARUCO_8_ID){
                        // Compute transformations
                        cv::Mat TAruco = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrixFromAruco(tvecs_small[i], rvecs_small[i], TAruco);

                        // Offset in position
                        cv::Vec3d object_position(ARUCO_8_OFFSET_X, ARUCO_8_OFFSET_Y, 0.0);
                        cv::Mat TObject = cv::Mat(4, 4, CV_64F);
                        generateHomogeneousTransMatrix(object_position, TObject);

                        cv::Mat OO = cv::Mat(4, 1, CV_64F);
                        OO.at<double>(0,0) = 0.0;
                        OO.at<double>(1,0) = 0.0;
                        OO.at<double>(2,0) = 0.0;
                        OO.at<double>(3,0) = 1.0;

                        cv::Mat SS = cv::Mat(4, 1, CV_64F);

#ifdef ENABLE_RELATIVE_YAW
                        float relative_yaw = atan2(TAruco.at<double>(1,0), TAruco.at<double>(0,0)) / M_PI;
                        accum_relative_yaw += relative_yaw;

                        cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
                        generateRotz((double) (relative_yaw * M_PI), RotzYaw);
#endif

#ifdef REAL_FLIGHT_MODE
                        // Compute tranformations
                        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#else
                        // Compute tranformations
                        SS = RotzYaw * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

#ifdef PBVS_MODE
                        cv::Mat PP = cv::Mat(4, 1, CV_64F);

                        // Compute transformation
                        PP = Rotx180 * Rotz90 * RotyPitch * RotxRoll * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;

                        // Compute position
                        cv::Vec3d position_pbvs(0.0,0.0,0.0);
                        position_pbvs[0] = PP.at<double>(0);
                        position_pbvs[1] = PP.at<double>(1);
                        position_pbvs[2] = PP.at<double>(2);

                        accum_tvec_pbvs += position_pbvs;
#endif
#endif


                        // Compute result
                        cv::Vec3d position(0.0,0.0,0.0);
                        position[0] = SS.at<double>(0);
                        position[1] = SS.at<double>(1);
                        position[2] = SS.at<double>(2);

                        // Accumulate position
                        accum_tvec += position;
                        cnt++;

                        // Position not lost
                        position_lost_ = 0;
                    }
                    else{
//                        std::cout << "RL_ENV_ERROR: Aruco not found" << std::endl;
                    }

                    // Draw axes
                    cv::aruco::drawAxis(cv_ptr->image, camera_parameters_, distortion_coefficients_, rvecs[i], tvecs[i], 0.1);

                }

                if (cnt){
#ifdef ENABLE_RELATIVE_YAW
                    current_relative_yaw_ = (float) accum_relative_yaw / (float) cnt;
#endif

#ifdef REAL_FLIGHT_MODE
                    // Store relative state
                    SetRelativeState(((double) accum_tvec[0] / cnt), ((double) accum_tvec[1] / cnt), (double) accum_tvec[2]  / cnt);
#ifdef PBVS_MODE
                    // Store relative state
                    SetRelativeStatePBVS(((double) accum_tvec_pbvs[0] / cnt), ((double) accum_tvec_pbvs[1] / cnt), (double) accum_tvec_pbvs[2]  / cnt);
#endif
#else
                    SetRelativeState(((double) accum_tvec[0] / cnt), ((double) accum_tvec[1] / cnt), (double) accum_tvec[2]  / cnt);
#ifdef PBVS_MODE
                    // Store relative state
                    SetRelativeStatePBVS(((double) accum_tvec_pbvs[0] / cnt), ((double) accum_tvec_pbvs[1] / cnt), (double) accum_tvec_pbvs[2]  / cnt);
#endif
#endif
                }

                if (SHOW_IMAGES){
                    char str[20];
                    sprintf(str,"x: %f", accum_tvec[0] / cnt);
                    putText(cv_ptr->image, str, cv::Point2f(10, 20), cv::FONT_HERSHEY_PLAIN, 1.3,  cv::Scalar(100, 200, 200));

                    sprintf(str,"y: %f", accum_tvec[1] / cnt);
                    putText(cv_ptr->image, str, cv::Point2f(10, 40), cv::FONT_HERSHEY_PLAIN, 1.3,  cv::Scalar(100, 200, 200));

                    sprintf(str,"z: %f", accum_tvec[2] / cnt);
                    putText(cv_ptr->image, str, cv::Point2f(10, 60), cv::FONT_HERSHEY_PLAIN, 1.3,  cv::Scalar(100, 200, 200));
                }
            }
        }
        else{
            position_lost_ = 1;
        }
        if (SHOW_IMAGES){
            if (position_lost_){
                putText(cv_ptr->image, "POSITION LOST", cv::Point2f(10, 80), cv::FONT_HERSHEY_PLAIN, 1.3,  cv::Scalar(100, 100, 250));
            }
            else{
                putText(cv_ptr->image, "WORKING", cv::Point2f(10, 80), cv::FONT_HERSHEY_PLAIN, 1.3,  cv::Scalar(100, 250, 100));

            }


            // Update GUI Window
            cv::imshow("Aruco Readings", cv_ptr->image);
            cv::waitKey(3);

            // Publish image in ROS
            sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
            aruco_camera_publ_.publish(msg_image);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


void RlEnvironmentLandingWithRPdYdAPEMarker::generateRotx(double phi, cv::Mat &R){
    R.at<double>(0,0) = 1.0;
    R.at<double>(0,1) = 0.0;
    R.at<double>(0,2) = 0.0;
    R.at<double>(0,3) = 0.0;
    R.at<double>(1,0) = 0.0;
    R.at<double>(1,1) = cos(phi);
    R.at<double>(1,2) = -sin(phi);
    R.at<double>(1,3) = 0.0;
    R.at<double>(2,0) = 0.0;
    R.at<double>(2,1) = sin(phi);
    R.at<double>(2,2) = cos(phi);
    R.at<double>(2,3) = 0.0;
    R.at<double>(3,0) = 0.0;
    R.at<double>(3,1) = 0.0;
    R.at<double>(3,2) = 0.0;
    R.at<double>(3,3) = 1.0;
}

void RlEnvironmentLandingWithRPdYdAPEMarker::generateRoty(double theta, cv::Mat &R){
    R.at<double>(0,0) = cos(theta);
    R.at<double>(0,1) = 0.0;
    R.at<double>(0,2) = sin(theta);
    R.at<double>(0,3) = 0.0;
    R.at<double>(1,0) = 0.0;
    R.at<double>(1,1) = 1.0;
    R.at<double>(1,2) = 0.0;
    R.at<double>(1,3) = 0.0;
    R.at<double>(2,0) = -sin(theta);
    R.at<double>(2,1) = 0.0;
    R.at<double>(2,2) = cos(theta);
    R.at<double>(2,3) = 0.0;
    R.at<double>(3,0) = 0.0;
    R.at<double>(3,1) = 0.0;
    R.at<double>(3,2) = 0.0;
    R.at<double>(3,3) = 1.0;
}

void RlEnvironmentLandingWithRPdYdAPEMarker::generateRotz(double chi, cv::Mat &R){
    R.at<double>(0,0) = cos(chi);
    R.at<double>(0,1) = -sin(chi);
    R.at<double>(0,2) = 0.0;
    R.at<double>(0,3) = 0.0;
    R.at<double>(1,0) = sin(chi);
    R.at<double>(1,1) = cos(chi);
    R.at<double>(1,2) = 0.0;
    R.at<double>(1,3) = 0.0;
    R.at<double>(2,0) = 0.0;
    R.at<double>(2,1) = 0.0;
    R.at<double>(2,2) = 1.0;
    R.at<double>(2,3) = 0.0;
    R.at<double>(3,0) = 0.0;
    R.at<double>(3,1) = 0.0;
    R.at<double>(3,2) = 0.0;
    R.at<double>(3,3) = 1.0;
}

void RlEnvironmentLandingWithRPdYdAPEMarker::generateHomogeneousTransMatrix(cv::Vec3d p, cv::Mat &T){
    T.at<double>(0,0) = 1.0;
    T.at<double>(0,1) = 0.0;
    T.at<double>(0,2) = 0.0;
    T.at<double>(0,3) = p[0];
    T.at<double>(1,0) = 0.0;
    T.at<double>(1,1) = 1.0;
    T.at<double>(1,2) = 0.0;
    T.at<double>(1,3) = p[1];
    T.at<double>(2,0) = 0.0;
    T.at<double>(2,1) = 0.0;
    T.at<double>(2,2) = 1.0;
    T.at<double>(2,3) = p[2];
    T.at<double>(3,0) = 0.0;
    T.at<double>(3,1) = 0.0;
    T.at<double>(3,2) = 0.0;
    T.at<double>(3,3) = 1.0;
}

void RlEnvironmentLandingWithRPdYdAPEMarker::generateHomogeneousTransMatrixFromAruco(cv::Vec3d tvec, cv::Vec3d rvec, cv::Mat &T){
    cv::Mat R = cv::Mat(3, 3, CV_64F);
    cv::Rodrigues(rvec, R);

    T.at<double>(0,0) = R.at<double>(0,0);
    T.at<double>(0,1) = R.at<double>(0,1);
    T.at<double>(0,2) = R.at<double>(0,2);
    T.at<double>(0,3) = tvec[0];
    T.at<double>(1,0) = R.at<double>(1,0);
    T.at<double>(1,1) = R.at<double>(1,1);
    T.at<double>(1,2) = R.at<double>(1,2);
    T.at<double>(1,3) = tvec[1];
    T.at<double>(2,0) = R.at<double>(2,0);
    T.at<double>(2,1) = R.at<double>(2,1);
    T.at<double>(2,2) = R.at<double>(2,2);
    T.at<double>(2,3) = tvec[2];
    T.at<double>(3,0) = 0.0;
    T.at<double>(3,1) = 0.0;
    T.at<double>(3,2) = 0.0;
    T.at<double>(3,3) = 1.0;
}

#ifdef PBVS_MODE
void RlEnvironmentLandingWithRPdYdAPEMarker::alturaCbSim(const droneMsgsROS::droneSpeeds& msg){
    vximu_ = msg.dx;
    vyimu_ = msg.dy;
    vzimu_ = msg.dz;
}
#endif

#ifdef REAL_FLIGHT_MODE
    void RlEnvironmentLandingWithRPdYdAPEMarker::bebopGTCallback(const nav_msgs::Odometry& msg){
        bebop_gt_x_ = msg.pose.pose.position.x;
        bebop_gt_y_ = msg.pose.pose.position.y;
        bebop_gt_z_ = msg.pose.pose.position.z;

#ifdef PUBLISH_TF
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
    tf::Quaternion q2;
    q2.setX(msg.pose.pose.orientation.x);
    q2.setY(msg.pose.pose.orientation.y);
    q2.setZ(msg.pose.pose.orientation.z);
    q2.setW(msg.pose.pose.orientation.w);
    transform.setRotation(q2);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav_real"));
#endif
    }

    void RlEnvironmentLandingWithRPdYdAPEMarker::platformGTCallback(const nav_msgs::Odometry& msg){
        platform_gt_x_ = msg.pose.pose.position.x;
        platform_gt_y_ = msg.pose.pose.position.y;
        platform_gt_z_ = msg.pose.pose.position.z;

#ifdef LOG_DATA
        // Log experiment data
        outFile_.open (EXPERIMENTS_FOLDER  + "iros_2018_ground_truth.csv", std::ios::out | std::ios::ate | std::ios::app);
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
        );
        outFile_ << episode_no_ << "," << ms.count();
        outFile_ << "," << bebop_gt_x_ << "," << bebop_gt_y_ << "," << bebop_gt_z_;
        outFile_ << "," << platform_gt_x_ << "," << platform_gt_y_ << "," << platform_gt_z_ << std::endl;
        outFile_.close();
#endif

#ifdef PUBLISH_TF
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
    tf::Quaternion q2;
    q2.setX(msg.pose.pose.orientation.x);
    q2.setY(msg.pose.pose.orientation.y);
    q2.setZ(msg.pose.pose.orientation.z);
    q2.setW(msg.pose.pose.orientation.w);
    transform.setRotation(q2);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "platform_real"));
#endif

    }
#endif



#endif // Opencv 3

