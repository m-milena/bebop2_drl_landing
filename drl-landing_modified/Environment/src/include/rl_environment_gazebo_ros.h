
#ifndef _RL_ENVIRONMENT_GAZEBO_ROS_H_
#define _RL_ENVIRONMENT_GAZEBO_ROS_H_

#include "opencv2/opencv.hpp"
#include "boost/thread.hpp"
#include "boost/bind.hpp"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include "rl_srvs/AgentSrv.h"
#include "rl_srvs/EnvDimensionalitySrv.h"
#include "rl_srvs/ResetEnvSrv.h"
#include <ros/console.h>

using namespace boost::interprocess;

class RlEnvironmentGazeboRos
{
private:
    const bool ENABLE_PAUSED_SIMULATION = false;
protected:

    enum { RESET_ITERATIONS = 5,
           LONG_ITERATIONS = 50};

    struct shared_memory_buffer
    {
//       enum { NUM_ITERATIONS = 2 };

       shared_memory_buffer()
          : check_action_(0), iterate_(0), iteration_finished_(0)
       {}

       // Semaphores to protect and synchronize access
       boost::interprocess::interprocess_semaphore
          check_action_, iterate_, iteration_finished_;

       // Items to fill
//       char items[NumItems];
       unsigned int num_iterations;

    };

    shared_memory_buffer * data_;
    mapped_region region_;

    // Flags to communicate
    bool return_observation_;
    bool reset_env_;
    bool long_iteration_;
    boost::condition_variable cond_;
    boost::mutex mut_;

    // Rl-related
    float prev_shaping_;

private:


    // Create a shared memory object.
    shared_memory_object shm_;
    void * addr_;

    // Check Action thread
    void CheckAction(){
        if (ENABLE_PAUSED_SIMULATION){
            // Print info
            std::cout << "RL_ENV_INFO: Check action thread initiated" << std::endl;
            bool exit = false;
            while(!exit){
                // Print info
                //            std::cout << "RL_ENV_INFO: Waiting for action.." << std::endl;
                // Wait for the action from the agent
                data_->check_action_.wait();
                // Print info
                //            std::cout << "RL_ENV_INFO: Action received" << std::endl;
                // Raise flag to iterate
                data_->iterate_.post();
                // Wait for iteration to finish
                data_->iteration_finished_.wait();
                // Print info
                //            std::cout << "RL_ENV_INFO: iteration finished" << std::endl;
                // Set number of iterations of Gazebo
                unsigned int previous_num_iterations = this->data_->num_iterations;
                // Reset service if required
                if (reset_env_){
                    // Restore flag
                    reset_env_ = false;
                    if (!long_iteration_){
                        this->data_->num_iterations = (unsigned int) RESET_ITERATIONS;
                    }
                    else
                        this->data_->num_iterations = (unsigned int) LONG_ITERATIONS;

                    // Reset long interations
                    long_iteration_ = false;

                    // Raise flag to iterate
                    data_->iterate_.post();
                    // Reset
                    Reset();
                    // Wait for iteration to finish
                    data_->iteration_finished_.wait();
                    // Restore number of iterations
                    this->data_->num_iterations = previous_num_iterations;
                }
                // Enable service to return observation
                boost::lock_guard<boost::mutex> lock(mut_);
                return_observation_=true;
                cond_.notify_one();
            }
        }
    }

protected:
    boost::thread check_action_thread_;

public:
    // Ros members
    ros::ServiceServer srv_server_step_;
    ros::ServiceServer srv_server_env_dimensionality_;
    ros::ServiceServer srv_reset_env_;

    // Pure virtual member functions
    virtual bool Step(rl_srvs::AgentSrv::Request &request, rl_srvs::AgentSrv::Response &response)=0;
    virtual bool EnvDimensionality(rl_srvs::EnvDimensionalitySrv::Request &request, rl_srvs::EnvDimensionalitySrv::Response &response)=0;
    virtual bool ResetSrv(rl_srvs::ResetEnvSrv::Request &request, rl_srvs::ResetEnvSrv::Response &response)=0;
    virtual void InitChild(ros::NodeHandle n)=0;
    virtual bool Reset()=0;

    void Init(ros::NodeHandle n){
        // Initialize services
        this->srv_server_step_ = n.advertiseService("/rl_env_step_srv", &RlEnvironmentGazeboRos::Step, this);
        this->srv_server_env_dimensionality_ = n.advertiseService("/rl_env_dimensionality_srv", &RlEnvironmentGazeboRos::EnvDimensionality, this);
        this->srv_reset_env_ = n.advertiseService("/rl_env_reset_srv", &RlEnvironmentGazeboRos::ResetSrv, this);

        // Initiate CheckAction thread
        check_action_thread_ = boost::thread(&RlEnvironmentGazeboRos::CheckAction, this);

        // Init return observation
        return_observation_=false;
        reset_env_ = false;
        long_iteration_ = false;

        // Init Rl-related variables
        prev_shaping_ = 0;

        // Init child
        InitChild(n);
    }

    void WaitForEnv(){
        boost::unique_lock<boost::mutex> lock(mut_);
        while(!return_observation_)
        {
            cond_.wait(lock);
        }
        return_observation_=false;
    }

    RlEnvironmentGazeboRos(){
        if (ENABLE_PAUSED_SIMULATION){
            // Init semaphore
            return_observation_ = false;

            // Open a shared memory object.
            shm_ = shared_memory_object
                    (open_only                   //only open
                     ,"CheckActionShm"            //name
                     ,read_write                  //read-write mode
                     );

            // Map the whole shared memory in this process
            region_ = mapped_region
                    (shm_                   //What to map
                     ,read_write            //Map it as read-write
                     );

            // Get the address of the mapped region
            addr_ = region_.get_address();

            // Construct the shared structure in memory
            data_ = static_cast<shared_memory_buffer*>(addr_);

            // Print info
            std::cout << "RL_ENV_INFO: Shared memory successfully opened" << std::endl;
        }
    }
    ~RlEnvironmentGazeboRos(){}
};

#endif

