# - Try to find ARUCO library
# Once done this will define
#  ARUCO_FOUND - System has the library
#  ARUCO_INCLUDE_DIR - The include directories
#  ARUCO_LIBRARY - The library(ies)

MESSAGE(STATUS "Aruco_DIR is : " ${Aruco_DIR})

find_path(Aruco_INCLUDE_DIR aruco.h cvdrawingutils.h
			 HINTS  ~/bebop_ws/src/code-rl-environment-gazebo/rl_libs/aruco304/src /usr/local/include /usr/include usr/
          PATH_SUFFIXES Aruco)
MESSAGE(STATUS "Aruco_INCLUDE_DIR is : " ${Aruco_INCLUDE_DIR})

find_library(Aruco_LIBRARY
	NAMES
		libaruco.so
	PATHS
		${Aruco_INCLUDE_DIR}/build/
		/usr/bin
		/usr/lib
		/usr/local/lib
		~/bebop_ws/src/code-rl-environment-gazebo/rl_libs/aruco304/build/src
)

MESSAGE(STATUS "Aruco_LIBRARY is : " ${Aruco_LIBRARY})

			
if(Aruco_LIBRARY AND Aruco_INCLUDE_DIR)
	mark_as_advanced(Aruco_LIBRARY Aruco_INCLUDE_DIR)
	set(Aruco_FOUND 1)
else()
	set(Aruco_FOUND 0)
endif()

# ----------------------------------------------------------------------
# Display status
# ----------------------------------------------------------------------
if(NOT Aruco_FOUND)
	if(Aruco_FIND_REQUIRED)
		#message(FATAL_ERROR "Aruco was not found. Please build dependencies first, or specify the path manually.")
	endif()
endif()
