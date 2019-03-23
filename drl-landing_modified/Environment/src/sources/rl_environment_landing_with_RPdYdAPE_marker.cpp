#include "rl_environment_landing_with_RPdYdAPE_marker.h"

#include <ros/ros.h>

#if HAS_OPENCV3 == 1

// Step function - agent move scaling values
bool RlEnvironmentLandingWithRPdYdAPEMarker::Step(rl_srvs::AgentSrv::Request &request, rl_srvs::AgentSrv::Response &response){

	geometry_msgs::Twist control;

	if(!position_lost_){
		control.linear.y = - SCALE_RP * request.action[0];  // Roll
		control.linear.x = - SCALE_RP * request.action[1];  // Pitch
		control.angular.z = SCALE_YAW * request.action[2];  // Yaw
		#ifdef RUNAWAY
		if ((ros::Time::now() - initial_time_).toSec() < SETTELING_TIME){
			control.linear.z = SCALE_ALTITUDE * request.action[3];  // Altitude
		}
		else{
			if ((ros::Time::now() - initial_time_).toSec() > RUNAWAY_TIME){
			control.linear.z = SCALE_ALTITUDE * request.action[3];  // Altitude
			}
			else {
			control.linear.z = 0; // Altitude
			}
		}
		#else
   		control.linear.z = SCALE_ALTITUDE * request.action[3]; // Altitude
		#endif
	}
	else{
		control.linear.y = 0;    // Roll
		control.linear.x = 0;    // Pitch
		control.angular.z = 0;   // Yaw
		control.linear.z = 0.3;  // Altitude 
	}


	// Publishing velocities to cmd_vel
	uav_control_pub.publish(control);

	float x_relative, y_relative, z_relative, dx_relative, dy_relative;
	GetRelativeState(x_relative, y_relative, z_relative);
	dx_relative = (prev_x_relative_ - x_relative ) / (float) MAX_POSE_X;
	dy_relative = (prev_y_relative_ - y_relative) / (float) MAX_POSE_Y;
	z_relative *= (-1);

	// NOTE: normalization is carried out here
	x_relative /= (float)  MAX_POSE_X;
	y_relative /= (float)  MAX_POSE_Y;
	z_relative = (z_relative + (PLATFORM_SIZE)) / (float)  Z_INITIAL;  // z_relative is slightly different

	#ifndef ENABLE_RELATIVE_YAW
	#ifdef REAL_FLIGHT_MODE
    	float yaw = current_bebop_yaw_;
	#else
	float yaw = current_gazebo_yaw_;
	#endif
	#else
	float yaw = current_relative_yaw_;
	#endif

	// Checking bumper state
	int state;
	GetBumperState(state);

	float shaping;
	shaping = - 100 * sqrt(pow(x_relative,2) + pow(y_relative,2) + pow(z_relative,2) + pow(yaw,2))
                - 10 * sqrt(pow(dx_relative, 2) + pow(dy_relative, 2))
                - sqrt(pow(request.action[0], 2) + pow(request.action[1], 2) + pow(request.action[2], 2) 			+ pow(request.action[3], 2));


	float reward = shaping - prev_shaping_;
	prev_shaping_ = shaping;
	response.reward = reward;


	if (reset_executed_ && (request.action[4] < EMERGENCY_THRESHOLD)){
		landing_executed_ = false;
		reset_executed_ = false;
	}

	if ((request.action[4] > EMERGENCY_THRESHOLD) && !landing_executed_ && !position_lost_ && (fabs(x_relative * (float) MAX_POSE_X) < PLATFORM_IN_SIZE) && (fabs(y_relative * (float) MAX_POSE_Y) < PLATFORM_IN_SIZE)){
		if (!ENABLE_EMERGENCY_INHIBITION || ((z_relative * (float) Z_INITIAL) < 		MAX_EMERGENCY_ALTITUDE)){
			landing_executed_ = true;
			SetBumperState(1);
			response.terminal_state = true;
			emergency_altitude_ = z_relative * (float) Z_INITIAL;
			std::cout << "ENV DEBUG: Emergency altitude " << emergency_altitude_ << std::endl;
			droneMsgsROS::droneCommand msg_command;
			msg_command.command = droneMsgsROS::droneCommand::RESET;
			drone_command_publ_.publish(msg_command);
			SetBumperState(1);	
			response.terminal_state = true;
		}
	}

	int st;
	GetBumperState(st);
	if(!st){
		response.terminal_state = false;
	}


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
	z_relative = (z_relative - (PLATFORM_SIZE)) / (float)  Z_INITIAL;    // z_relative is slightly different

	state_next.push_back(x_relative);
	state_next.push_back(y_relative);
	state_next.push_back(dx_relative);
	state_next.push_back(dy_relative);
	state_next.push_back(z_relative);
	state_next.push_back((float) state);
	state_next.push_back(yaw);
   
    	response.obs_real = state_next;
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

	Reset();

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
	#endif
	#else
	s.push_back(current_relative_yaw_);
	#endif

	response.state = s;

	return true;
}


void RlEnvironmentLandingWithRPdYdAPEMarker::InitChild(ros::NodeHandle n){
	// Print info
	std::cout << "RL_ENV_INFO: LANDING MARKER ENVIRONMENT" << std::endl;

	// Init subscribers
	image_sub_ = n.subscribe("/bebop_leader/image_raw", 10, &RlEnvironmentLandingWithRPdYdAPEMarker::imageCallback, this);
	bebop_odom_sub = n.subscribe("/bebop_leader/odom_conv", 10, &RlEnvironmentLandingWithRPdYdAPEMarker::bebopPoseCallback, this);

	//Init publishers
	uav_control_pub = n.advertise<geometry_msgs::Twist>("/bebop_leader/cmd_vel", 1000);
	drone_command_publ_= n.advertise<droneMsgsROS::droneCommand>("/bebop_leader/command", 1000);
	image_transport::ImageTransport it(n);
	aruco_camera_publ_ = it.advertise("/bebop_leader/aruco_camera", 1000);

	episode_no_ = 0;
	prev_x_relative_ = 0;
	prev_y_relative_ = 0;
	initial_yaw_ = true;
	#ifdef ENABLE_RELATIVE_YAW
	current_relative_yaw_ = 0;
	#endif

	#ifdef LOG_DATA
	outFile_.open (EXPERIMENTS_FOLDER + "drone_pose.csv", std::ios::out | std::ios::ate | std::ios::app);
	outFile_ << "time_ms,x_uav,y_uav,z_uav" << std::endl;
	outFile_.close();
	#endif

	// Reset environment
	Reset();
}

bool RlEnvironmentLandingWithRPdYdAPEMarker::Reset(){

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
	current_bebop_yaw_ = 0;

	#ifdef RUNAWAY
	// Init time
	initial_time_ = ros::Time::now();
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


// Platform detection and calculation position
void RlEnvironmentLandingWithRPdYdAPEMarker::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
	try{
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

            		cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_parameters_, distortion_coefficients_, rvecs, tvecs);
            		cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE_SMALL, camera_parameters_, distortion_coefficients_, rvecs_small, tvecs_small);

            		// Generate Rotation matrices
            		cv::Mat Rotx180 = cv::Mat(4, 4, CV_64F);

            		generateRotx(3.037, Rotx180);

            		cv::Mat Rotz90 = cv::Mat(4, 4, CV_64F);
            		generateRotz(M_PI / 2, Rotz90);


            		cv::Mat RotzYaw = cv::Mat(4, 4, CV_64F);
           		 generateRotz((double) (current_bebop_yaw_ * M_PI), RotzYaw);

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

				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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

				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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

				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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

				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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

				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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

				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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

				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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


				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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

				        // Compute tranformations
				        SS = RotzYaw * Rotx180 * Rotz90 * TCamera * TAruco * TObject * OO;


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

				// Draw axes
				cv::aruco::drawAxis(cv_ptr->image, camera_parameters_, distortion_coefficients_, rvecs[i], tvecs[i], 0.1);
                	}

                	if (cnt){
				#ifdef ENABLE_RELATIVE_YAW
				current_relative_yaw_ = (float) accum_relative_yaw / (float) cnt;
				#endif
				// Store relative state
				SetRelativeState(((double) accum_tvec[0] / cnt), ((double) accum_tvec[1] / cnt), (double) accum_tvec[2]  / cnt);
                	}
		    
		// Print position message on sreen
                	if (SHOW_IMAGES){
				char str[20];
				sprintf(str,"x: %f", accum_tvec[0] / cnt);
				putText(cv_ptr->image, str, cv::Point2f(10, 20), cv::FONT_HERSHEY_PLAIN, 1.3,  cv::Scalar(210, 65, 100));

				sprintf(str,"y: %f", accum_tvec[1] / cnt);
				putText(cv_ptr->image, str, cv::Point2f(10, 40), cv::FONT_HERSHEY_PLAIN, 1.3,  cv::Scalar(210, 65, 100));

				sprintf(str,"z: %f", accum_tvec[2] / cnt);
				putText(cv_ptr->image, str, cv::Point2f(10, 60), cv::FONT_HERSHEY_PLAIN, 1.3,  cv::Scalar(210, 65, 100));
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


void RlEnvironmentLandingWithRPdYdAPEMarker::bebopPoseCallback(const collision_avoid::msgCoordinates::ConstPtr& msg){

	bebop_x=msg->x;
	bebop_y=msg->y;
	bebop_z=msg->z;
	#ifdef LOG_DATA
        // Log experiment data
        outFile_.open (EXPERIMENTS_FOLDER  + "drone_pose.csv", std::ios::out | std::ios::ate | std::ios::app);
        outFile_ << bebop_x<< "," << bebop_y<< "," << bebop_z<<std::endl;
        outFile_.close();
	#endif
}


#endif // Opencv 3

