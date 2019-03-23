/*
 *
 *
 *  Created on: May 26th 2017
 *      Author: Alejandro Rodríguez
 */


#include "rl_environment_gazebo_ros.h"
#include "rl_environment_landing_with_RPdYdAPE_marker.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "rl_environment_gazebo_ros");
    ros::NodeHandle n;


    std::string environment_name;
    ros::param::get("~environment_name", environment_name);
    if(environment_name.length() == 0)
    {
        environment_name = "RlEnvironmentLandingWithRPdYdAPEMarker";
    }
    std::cout<<"environment_name = "<<environment_name<<std::endl;


    // Polymorphism
    RlEnvironmentGazeboRos* rl_environment_gazebo_ros;
#if HAS_OPENCV3 == 1
    if(environment_name == "RlEnvironmentLandingWithRPdYdAPEMarker"){
        rl_environment_gazebo_ros = new RlEnvironmentLandingWithRPdYdAPEMarker;
	std::cout << "RL_ENV_INFO: RlEnvironmentLandingWithRPdYdAPEMarker" << std::endl;
	}
#endif



    // Init
    rl_environment_gazebo_ros->Init(n);

    std::cout << "RL_ENV_INFO: Sucessfuly initiated" << std::endl;

    // Ros spin
    ros::spin();

    return 1;

}

