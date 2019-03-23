cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME RlEnvironmentGazeboRos)
project(${PROJECT_NAME})

add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


set(RL_ENVIRONMENT_GAZEBO_ROS_SOURCE_DIR src/sources)
set(RL_ENVIRONMENT_GAZEBO_ROS_INCLUDE_DIR src/include)
set(RL_ENVIRONMENT_GAZEBO_ROS_TEST_DIR src/test)

FILE(GLOB_RECURSE RL_ENVIRONMENT_GAZEBO_ROS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.hpp)

FILE(GLOB_RECURSE RL_ENVIRONMENT_GAZEBO_ROS_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sources/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sources/*.cpp)

FILE(GLOB_RECURSE RL_ENVIRONMENT_GAZEBO_ROS_TEST_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/test/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/test/*.cpp)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE_MODULE_PATH is: " ${CMAKE_MODULE_PATH})


find_package(catkin REQUIRED
                COMPONENTS roscpp droneMsgsROS droneModuleROS std_msgs nav_msgs rl_srvs rosbag cv_bridge image_transport tf control_toolbox roscpp tf geometry_msgs message_generation dynamic_reconfigure collision_avoid)

catkin_package(
        CATKIN_DEPENDS roscpp droneMsgsROS droneModuleROS std_msgs nav_msgs rl_srvs rosbag cv_bridge image_transport tf control_toolbox tf geometry_msgs message_generation dynamic_reconfigure collision_avoid
  )

#find_package( OpenCV 3 REQUIRED)
#set(OPENCV3_FOUND ${OpenCV_FOUND})
#add_definitions(-DHAS_OPENCV3=${OPENCV3_FOUND} )

# Remove before pushing
set(OpenCV_DIR /home/alejandro/sources/opencv-3.2.0/build)
#find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED PATHS /home/alejandro/sources/opencv-3.2.0/cmake)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Failed to find OpenCV")
else()
    include_directories(${OpenCV_INCLUDE_DIRS})
    MESSAGE(STATUS "OpenCV 3 DIR is: " ${OpenCV_INCLUDE_DIRS})
    MESSAGE(STATUS "OpenCV 3 LIBS DIR is: " ${OpenCV_LIBS})
endif(NOT OpenCV_FOUND)


#if(NOT OpenCV_FOUND)
#    find_package( OpenCV REQUIRED )
#endif(NOT OpenCV_FOUND)

set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package(Boost 1.54 REQUIRED)
find_package(Aruco REQUIRED )
if(NOT Aruco_FOUND)
    #message(FATAL_ERROR "Failed to find Aruco")
    set(Aruco_LIBRARY "")
else()
    include_directories(${Aruco_INCLUDE_DIR})
    MESSAGE(STATUS "Aruco_DIR is: " ${Aruco_INCLUDE_DIR})
endif(NOT Aruco_FOUND)


include_directories(${RL_ENVIRONMENT_GAZEBO_ROS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library(RlEnvironmentGazeboRos_lib ${RL_ENVIRONMENT_GAZEBO_ROS_SOURCE_FILES} ${RL_ENVIRONMENT_GAZEBO_ROS_HEADER_FILES})
add_dependencies(RlEnvironmentGazeboRos_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(RlEnvironmentGazeboRos_lib ${catkin_LIBRARIES})
target_link_libraries(RlEnvironmentGazeboRos_lib ${OpenCV_LIBS} ${Boost_LIBRARIES} rt ${Aruco_LIBRARY})

add_executable(RlEnvironmentGazeboRos ${RL_ENVIRONMENT_GAZEBO_ROS_TEST_DIR}/rl_environment_gazebo_ros_node.cpp)
add_dependencies(RlEnvironmentGazeboRos ${catkin_EXPORTED_TARGETS})
target_link_libraries(RlEnvironmentGazeboRos RlEnvironmentGazeboRos_lib)
target_link_libraries(RlEnvironmentGazeboRos ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} rt ${Aruco_LIBRARY})
