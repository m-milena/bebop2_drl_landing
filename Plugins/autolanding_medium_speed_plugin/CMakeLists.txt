cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(animated_platform)

# Find packages

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create libraries and executables

add_library(animated_platform SHARED animated_platform.cc)
target_link_libraries(animated_platform ${GAZEBO_LIBRARIES})

