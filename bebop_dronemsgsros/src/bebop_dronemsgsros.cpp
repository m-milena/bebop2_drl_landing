#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "droneMsgsROS/droneCommand.h"
#include "droneMsgsROS/droneStatus.h"

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/poses_stamped.pb.h>
#include <collision_avoid/msgCoordinates.h>

#include <string>
#include <sstream>

#define PI 3.14159265358979

struct sCoordinates
{
	double x,y,z,yaw;
};

class bebop_dronemsgsros{
	private:
	std::string drone_name;
	droneMsgsROS::droneCommand command;
	sCoordinates actual_pose;
	std_msgs::Empty empty_msg;
	geometry_msgs::Twist camera;
	droneMsgsROS::droneStatus status;	
	bool state_flying=false;
	bool once=false;
	bool land=false;
	
	public:
	bebop_dronemsgsros(std::string name, sCoordinates z): drone_name(name), actual_pose(z){
	camera.linear.x=camera.linear.y=camera.linear.z=0;
	camera.angular.x=camera.angular.z=0;
	camera.angular.y=-90;
	}
	void update_pose(ConstPosesStampedPtr &msg);
	void read_command(const droneMsgsROS::droneCommand::ConstPtr& msg);
	void camera_pose();
	void platform_landing();
	bool flying();
	bool landing();
	void change_state_takeoff(const std_msgs::Empty::ConstPtr& msg);
        ros::Subscriber command_sub, takeoff_sub;
	ros::Publisher land_pub, takeoff_pub, camera_control_pub, status_pub, odom_conv_pub;
	gazebo::transport::SubscriberPtr odom_gazebo_sub;
};

// Reading drone status
void bebop_dronemsgsros::read_command(const droneMsgsROS::droneCommand::ConstPtr& msg)
{	
	if(msg->command == 5)
	{
		std::cout<<"RESET"<<std::endl;
		state_flying=false;
		land=true;
	}
		else if(msg->command == 1)
	{
		std::cout<<"TAKEOFF"<<std::endl;
	}
		else if(msg->command == 1)
	{
		std::cout<<"MOVE"<<std::endl;
	}
		
}
// Reading Follower pose from Gazebo
void bebop_dronemsgsros::update_pose(ConstPosesStampedPtr &msg){

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

//Change camera rotation to -90 in Y axis.
void bebop_dronemsgsros::camera_pose(){
	std::cout<<"Try change camera pose"<<std::endl;
	std::cout<<camera<<std::endl;
	camera_control_pub.publish(camera);
	takeoff_pub.publish(empty_msg);
}

//Landing on platform command
void bebop_dronemsgsros::platform_landing(){
	std::cout<<"Landing on platform"<<std::endl;
	land_pub.publish(empty_msg);
	status.status=2;	
	status_pub.publish(status);
	state_flying=false;
}

//Take off UAV
void bebop_dronemsgsros::change_state_takeoff(const std_msgs::Empty::ConstPtr& msg)
{
	ros::Duration time(0.5);  // in seconds
	time.sleep();
	std_msgs::Bool s;
	s.data = true;
	state_flying=true;
	if(once==false){
		once=true;
		camera_pose();
		std::cout<<"Flying"<<std::endl;
	}
	status.status=3;	
	status_pub.publish(status);
}

//Return true, if the drone is flying
bool bebop_dronemsgsros::flying()
{
	return state_flying;
}

//Return true, if the drone is landing
bool bebop_dronemsgsros::landing()
{
	return land;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bebop_dronemsgsros");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);  // in Hz

	sCoordinates zeros = {0,0,0,0};
	bebop_dronemsgsros bebop_leader = bebop_dronemsgsros("bebop_leader", zeros);

	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	
//*************************< Subscribers >**************************
	bebop_leader.command_sub=n.subscribe("/bebop_leader/command", 100, &bebop_dronemsgsros::read_command, &bebop_leader);
	bebop_leader.takeoff_sub=n.subscribe("/bebop_leader/takeoff", 100, &bebop_dronemsgsros::change_state_takeoff, &bebop_leader);
	bebop_leader.odom_gazebo_sub = node->Subscribe("/gazebo/default/pose/info", &bebop_dronemsgsros::update_pose, &bebop_leader);
	ROS_INFO("BEBOP: Topic including command subscribed!");


//*************************< Publishers >****************************
	bebop_leader.land_pub = n.advertise<std_msgs::Empty>("/bebop_leader/land", 100);
	bebop_leader.takeoff_pub = n.advertise<std_msgs::Empty>("/bebop_leader/takeoff", 100);
	bebop_leader.camera_control_pub=n.advertise<geometry_msgs::Twist>("/bebop_leader/camera_control", 100);
	bebop_leader.status_pub = n.advertise<droneMsgsROS::droneStatus>("/bebop_leader/status", 100);
	bebop_leader.odom_conv_pub = n.advertise<collision_avoid::msgCoordinates>("/bebop_leader/odom_conv", 100);	

	bebop_leader.camera_pose();
	

	while (ros::ok())
	{
		if(!bebop_leader.flying() && bebop_leader.landing()){
		 bebop_leader.platform_landing();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	gazebo::client::shutdown();
	return 0;
}


