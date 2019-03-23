#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tfMessage.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TransformStamped.h"
#include <dynamic_reconfigure/server.h>
#include <collision_avoid/PIDConfig.h>
#include <collision_avoid/msgData.h>
#include <collision_avoid/msgCoordinates.h>
#include <math.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <fstream>
#include <iostream>
using namespace std;

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/poses_stamped.pb.h>
#include <gazebo/gazebo_client.hh>

#define PI 3.14159265358979
#define LOG_DATA
struct sCoordinates
{
	double x,y,z,yaw;
};

class PositionController
{
	private:
		string drone_name;
		bool state_flying=false;;
		sCoordinates target_pose, actual_pose, error, error_prev, integral, velocity, prev_vel;
		sCoordinates leader_pose, leader_last_pose;
		sCoordinates Kp, Ki, Kd;
		double max_vel, max_vel_z, max_vel_yaw;  // maximum velocities
		double error_ctrl_x, error_ctrl_y; 
		geometry_msgs::Twist cmd;  // velocity command
		geometry_msgs::Quaternion q;
		geometry_msgs::TransformStamped frame;
		geometry_msgs::Twist cmd_avoid;
		
	public:	
		PositionController(sCoordinates z, string name): actual_pose(z), error(z), error_prev(z), integral(z), velocity(z), prev_vel(z), drone_name(name)
		{		
		}
		
		~PositionController();
		double time_now, last_time;
		ros::Subscriber odom_sub, target_sub, land_sub, takeoff_sub, reset_sub, leader_pose_sub;
		ros::Publisher vel_pub, data_pub,  odom_conv_pub, state_pub, model_state_pub;
		gazebo::transport::SubscriberPtr odom_sub_gz;
		float lspeed_x, lspeed_y;
		float time=0;
		int save_time=0;
		int speed_time = 0;
		void change_state_land(const std_msgs::Empty::ConstPtr& msg);
		void change_state_takeoff(const std_msgs::Empty::ConstPtr& msg);
		void update_pose_gz(ConstPosesStampedPtr &msg);
		void update_target(const collision_avoid::msgCoordinates::ConstPtr& msg);
		void control();
		void save_data();
		void speed_read();
		void read_leader_pose(const collision_avoid::msgCoordinates::ConstPtr& msg);
		void reconfig_callback(position_controller::PIDConfig &config);
		collision_avoid::msgData NewData();		
		bool flying();
		void collision_check();
		void target_after_takeoff(double altitude);
};


PositionController::~PositionController()
{
}

// Checking collision between Follower and Leader
void PositionController::collision_check(){
	speed_time++;
	if(speed_time == 2){
		time_now =ros::Time::now().toSec();
		speed_read();
		speed_time=0;
		last_time=time_now;
		leader_last_pose=leader_pose;
	}
	float x_difference = actual_pose.x - leader_pose.x;
	float y_difference = actual_pose.y - leader_pose.y;
	float z_difference = actual_pose.z - leader_pose.z;
	float collision_distance = 1.5;
	float distance = sqrt( pow(x_difference,2) + pow(y_difference,2) + pow(z_difference,2) );
	
	if(distance<collision_distance){
		if(lspeed_y > lspeed_x){
			if(actual_pose.x>=leader_pose.x && actual_pose.x>=0){
				target_pose.x = actual_pose.x+(collision_distance-distance); 
				}
			else if(actual_pose.x>=leader_pose.x && actual_pose.x<0){
				target_pose.x = actual_pose.x+(collision_distance-distance); }
			else if(actual_pose.x<leader_pose.x && actual_pose.x>=0){
				target_pose.x = actual_pose.x-(collision_distance-distance); }
			else if (actual_pose.x<leader_pose.x && actual_pose.x<0){
				target_pose.x = actual_pose.x-(collision_distance-distance);}
			}
		else if(lspeed_y <= lspeed_x){
			if(actual_pose.y>=leader_pose.y && actual_pose.y>=0){
				target_pose.y = actual_pose.y+(collision_distance-distance); }
			else if(actual_pose.y>=leader_pose.y && actual_pose.y<0){
				target_pose.y = actual_pose.y+(collision_distance-distance); }
			else if(actual_pose.y<leader_pose.y && actual_pose.y>=0){
				target_pose.y = actual_pose.y-(collision_distance-distance); }
			else if(actual_pose.y<leader_pose.y && actual_pose.y<0){
				target_pose.y = actual_pose.y-(collision_distance-distance);}	
		}
	}

#ifdef LOG_DATA
	save_time++;
	if(save_time == 10){
	save_data();
	}
#endif
}

// Saving trajectories Leader and Follower to *.csv file
void PositionController::save_data(){

	std::ofstream leader_file("/home/milena/Pulpit/statics/leader_pose.csv", std::ios::out | std::ios::ate | std::ios::app);	
	leader_file<<time<<","<<leader_pose.x<<","<<leader_pose.y<<","<<leader_pose.z<<std::endl;
	leader_file.close();

	std::ofstream follower_file("/home/milena/Pulpit/statics/follower_pose.csv", std::ios::out | std::ios::ate | std::ios::app);	
	follower_file<<time<<","<<actual_pose.x<<","<<actual_pose.y<<","<<actual_pose.z<<std::endl;
	follower_file.close();
	time++;
	save_time=0;
}

// Calculating Leader velocities
void PositionController::speed_read(){
	float s_x=abs(leader_last_pose.x - leader_pose.x);
	float s_y=abs(leader_last_pose.y - leader_pose.y);
	float time=time_now - last_time;
	
	lspeed_x=s_x/time;
	lspeed_y=s_y/time;
	//std::cout<<"Czas: "<<time<<std::endl;
	//std::cout<<"Droga x: "<<s_x<<std::endl;
	//std::cout<<"Droga y: "<<s_y<<std::endl;
	//std::cout<<"Predkosc w x: "<<lspeed_x<<std::endl<<"Predkosc w y: "<<lspeed_y<<std::endl;

}

void PositionController::change_state_land(const std_msgs::Empty::ConstPtr& msg)
{
	std_msgs::Bool s;
	s.data = false;
	state_flying = false;
	state_pub.publish(s);
}

void PositionController::change_state_takeoff(const std_msgs::Empty::ConstPtr& msg)
{
	ros::Duration time(0.5);  // in seconds
	time.sleep();
	std_msgs::Bool s;
	s.data = true;
	state_flying = true;
	state_pub.publish(s);
}
bool PositionController::flying()
{
	return state_flying;
}

// Reading Leader position
void PositionController::read_leader_pose(const collision_avoid::msgCoordinates::ConstPtr& msg){
	leader_pose.x=msg->x;
	leader_pose.y=msg->y;
	leader_pose.z=msg->z;
	leader_pose.yaw=msg->yaw;
}

// Updating Follower position
void PositionController::update_target(const collision_avoid::msgCoordinates::ConstPtr& msg)
{
	target_pose.x = msg->x;
	target_pose.y = msg->y;
	target_pose.z = msg->z;
	target_pose.yaw = msg->yaw;
}

void PositionController::target_after_takeoff(double altitude)
{	
	target_pose.x = actual_pose.x;
	target_pose.y = actual_pose.y;
	target_pose.z = altitude;
	target_pose.yaw = actual_pose.yaw;
}

// Updating Follower position from Gazebo
void PositionController::update_pose_gz(ConstPosesStampedPtr &msg)
{
	collision_avoid::msgCoordinates conv;
	 for (int i =0; i < msg->pose_size(); ++i)
    {
		const ::gazebo::msgs::Pose &pose = msg->pose(i);
		if (pose.name() == drone_name)
		{
			const ::gazebo::msgs::Vector3d &position = pose.position();
			const ::gazebo::msgs::Quaternion q = pose.orientation();
			actual_pose.yaw = conv.yaw = atan2(  (2*(q.w()*q.z()+q.x()*q.y())) , (q.w()*q.w() - q.x()*q.x() - q.y()*q.y() - q.z()*q.z()) )  * 180 / PI;
			actual_pose.x = conv.x = position.x();
			actual_pose.y = conv.y = position.y();
			actual_pose.z = conv.z = position.z();
		}
	}
	odom_conv_pub.publish(conv);
}

// PID regulator values
void PositionController::reconfig_callback(position_controller::PIDConfig &config)
{
	Kp.x = config.Kp_x;
	Ki.x = config.Ki_x;
	Kd.x = config.Kd_x;
	Kp.y = config.Kp_y;
	Ki.y = config.Ki_y;
	Kd.y = config.Kd_y;
	Kp.z = config.Kp_z;
	Ki.z = config.Ki_z;
	Kd.z = config.Kd_z;
	Kp.yaw = config.Kp_yaw;
	Ki.yaw = config.Ki_yaw;
	Kd.yaw = config.Kd_yaw;
	max_vel = config.max_vel_xy;
	max_vel_z = config.max_vel_z;
	max_vel_yaw = config.max_vel_yaw;
}

// Prepare data to send it to topic position_controller_data
collision_avoid::msgData PositionController::NewData()
{
   collision_avoid::msgData msgNewData;
	
	msgNewData.target_pos.x = target_pose.x;
	msgNewData.target_pos.y = target_pose.y;
	msgNewData.target_pos.z = target_pose.z;
	msgNewData.target_pos.yaw = target_pose.yaw;
	
	msgNewData.actual_pos.x = actual_pose.x;
    	msgNewData.actual_pos.y = actual_pose.y;
	msgNewData.actual_pos.z = actual_pose.z;
	msgNewData.actual_pos.yaw = actual_pose.yaw;
	
	msgNewData.error_pos.x = error.x;
	msgNewData.error_pos.y = error.y;
	msgNewData.error_pos.z = error.z;
	msgNewData.error_pos.yaw = error.yaw;
	
	msgNewData.error_ctrl_x = error_ctrl_x;
	msgNewData.error_ctrl_y = error_ctrl_y;
		
	msgNewData.velocity_cmd.x = velocity.x;
	msgNewData.velocity_cmd.y = velocity.y;
	msgNewData.velocity_cmd.z = velocity.z;
	msgNewData.velocity_cmd.yaw = velocity.yaw;
	
	msgNewData.max_vel = max_vel;
	msgNewData.min_vel = -max_vel;
	
	msgNewData.Kp.x = Kp.x;
	msgNewData.Ki.x = Ki.x;
	msgNewData.Kd.x = Kd.x;
	msgNewData.Kp.y = Kp.y;
	msgNewData.Ki.y = Ki.y;
	msgNewData.Kd.y = Kd.y;
	msgNewData.Kp.z = Kp.z;
	msgNewData.Ki.z = Ki.z;
	msgNewData.Kd.z = Kd.z;
	msgNewData.Kp.yaw = Kp.yaw;
	msgNewData.Ki.yaw = Ki.yaw;
	msgNewData.Kd.yaw = Kd.yaw;
	
	return msgNewData;
}	

// Main function of PID regulator
void PositionController::control()
{
// Control errors
	error.x = target_pose.x - actual_pose.x;
	error.y = target_pose.y - actual_pose.y;
	error.z = target_pose.z - actual_pose.z;
	error.yaw = target_pose.yaw - actual_pose.yaw;
	
// Correct Yaw value checking
	if(error.yaw < -180.0)  
		error.yaw = error.yaw + 360.0;
	else if(error.yaw > 180.0)
		error.yaw = error.yaw - 360.0;
	
	error_ctrl_x = error.x * cos(actual_pose.yaw*PI/180.0) + error.y * sin(actual_pose.yaw*PI/180.0);
	error_ctrl_y = error.y * cos(actual_pose.yaw*PI/180.0) - error.x * sin(actual_pose.yaw*PI/180.0);
	
// anty-windup
	if( ((prev_vel.x > max_vel) && (error_ctrl_x >0)) || ((prev_vel.x < -max_vel) && (error_ctrl_x <0)) )
		integral.x = 0;
	else
		integral.x += error_ctrl_x;
	
	if( ((prev_vel.y > max_vel) && (error_ctrl_y >0)) || ((prev_vel.y < -max_vel) && (error_ctrl_y <0)) )
		integral.y = 0;
	else
		integral.y += error_ctrl_y;
	
	if( ((prev_vel.z > max_vel_z) && (error.z >0)) || ((prev_vel.z < -max_vel_z) && (error.z <0)) )
		integral.z = 0;
	else
		integral.z += error.z;
	
	if( ((prev_vel.yaw > max_vel_yaw) && (error.yaw >0)) || ((prev_vel.yaw < -max_vel_yaw) && (error.yaw <0)) )
		integral.yaw = 0;
	else
		integral.yaw += error.yaw;
	
// Calculate outputs
	velocity.x  =  	  Kp.x * error_ctrl_x   + Ki.x * integral.x 		 + Kd.x * (error_ctrl_x - error_prev.x);
	velocity.y  = 	  Kp.y * error_ctrl_y  + Ki.y * integral.y 		 + Kd.y * (error_ctrl_y - error_prev.y);
	velocity.z  =	  Kp.z * error.z 	  	   + Ki.z * integral.z 		 + Kd.z * (error.z - error_prev.z);
	velocity.yaw = Kp.yaw * error.yaw + Ki.yaw * integral.yaw + Kd.yaw * (error.yaw - error_prev.yaw);
	
// Save last output and error
	prev_vel.x = velocity.x;
	prev_vel.y = velocity.y;
	prev_vel.z = velocity.z;
	prev_vel.yaw = velocity.yaw;

	error_prev.x = error_ctrl_x;
	error_prev.y = error_ctrl_y;
	error_prev.z = error.z;
	error_prev.yaw = error.yaw;
	
// Saturation 
	if (velocity.x < -max_vel)
		velocity.x = -max_vel;
	else if (velocity.x > max_vel)
		velocity.x = max_vel;
	if (velocity.y < -max_vel)
		velocity.y = -max_vel;
	else if (velocity.y > max_vel)
		velocity.y = max_vel;
	if (velocity.z < -max_vel_z)
		velocity.z = -max_vel_z;
	else if (velocity.z > max_vel_z)
		velocity.z = max_vel_z;
// velocity in ,,z" axis is also limited by SpeedSettingsMaxVerticalSpeedCurrent parameter (default 1 m/s)
	if (velocity.yaw < -max_vel_yaw)
		velocity.yaw = -max_vel_yaw;
	else if (velocity.yaw > max_vel_yaw)
		velocity.yaw = max_vel_yaw;
	
// Prepare command
	cmd.linear.x = velocity.x;	
	cmd.linear.y = velocity.y;	
	cmd.linear.z = velocity.z;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = velocity.yaw;	
	
// Publish command 
	vel_pub.publish(cmd);
	data_pub.publish(this->NewData()); // send data to topic position_controller_data
}


// Main function
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "collision_avoid");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);  // in Hz	

	sCoordinates zeros = {0,0,0,0};
	double init_altitude = 1;
	
	PositionController bebop_follower = PositionController(zeros, "bebop_follower");

// Create files to save trajectories
#ifdef LOG_DATA
	std::ofstream leader_file("/home/milena/Pulpit/statics/leader_pose.csv", std::ios::out | std::ios::ate | std::ios::app);	
	leader_file<< "x_uav,y_uav,z_uav"<<std::endl;
	leader_file.close();
	std::ofstream follower_file("/home/milena/Pulpit/statics/follower_pose.csv", std::ios::out | std::ios::ate | std::ios::app);	
	follower_file<< "time,x_uav,y_uav,z_uav"<<std::endl;
	follower_file.close();
#endif


//            < Gazebo >
	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	
//            < Publishers >
	bebop_follower.vel_pub = n.advertise<geometry_msgs::Twist>("/bebop_follower/cmd_vel", 100);
	bebop_follower.data_pub = n.advertise<collision_avoid::msgData>("/position_controller_leader_data", 100);
	bebop_follower.state_pub = n.advertise<std_msgs::Bool>("/bebop_follower/state", 100);
	bebop_follower.odom_conv_pub = n.advertise<collision_avoid::msgCoordinates>("/bebop_follower/odom_conv", 100);

	
//            < Subscribers >
	bebop_follower.odom_sub_gz = node->Subscribe("/gazebo/default/pose/info", &PositionController::update_pose_gz, &bebop_follower);
	bebop_follower.target_sub = n.subscribe("/bebop_follower/target", 100, &PositionController::update_target, &bebop_follower);
	bebop_follower.land_sub = n.subscribe("/bebop_follower/land", 100, &PositionController::change_state_land, &bebop_follower);
	bebop_follower.reset_sub = n.subscribe("/bebop_follower/reset", 100, &PositionController::change_state_land, &bebop_follower);
	bebop_follower.takeoff_sub = n.subscribe("/bebop_follower/takeoff", 100, &PositionController::change_state_takeoff, &bebop_follower);
	bebop_follower.leader_pose_sub = n.subscribe("bebop_leader/odom_conv", 100, &PositionController::read_leader_pose, &bebop_follower);
	

	dynamic_reconfigure::Server<position_controller::PIDConfig> server;
	dynamic_reconfigure::Server<position_controller::PIDConfig>::CallbackType f;
	f = boost::bind(&PositionController::reconfig_callback, boost::ref(bebop_follower), _1);
	server.setCallback(f);
	
	bebop_follower.target_after_takeoff(init_altitude);

	while (ros::ok())
	{
		if(bebop_follower.flying())
			bebop_follower.control();
			bebop_follower.collision_check();
		ros::spinOnce();
		loop_rate.sleep();
	}
	gazebo::client::shutdown();
	return 0;
}
