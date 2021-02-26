cmake_minimum_required(VERSION 2.8)

# Define the name of the reference name
set(PROG nao_dynamics_imitation)

# Define the name of the project
project(${PROG})

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)


# This include enables the use of qibuild framework
find_package(qibuild)
find_package(qimodule)


# Make sure the generated header can be found:
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(".")

# Create a list of source files
set(_srcs
	xsens_imitation_v7.cpp
	xsens_tool.cpp
	xsens_tool.h
	robot_v3.h
	robot_v3.cpp

	# Jacobian Files
	Jacobian/Jacobian.h
	Jacobian/DGM.cpp
	  Jacobian/DKM.cpp
	  #Jacobian/DGM_rectified.cpp
	  #Jacobian/DKM_rectified.cpp
	  #Jacobian/DGM_rectified2.cpp
	  #Jacobian/DKM_rectified2.cpp
	Jacobian/DGM_wrt_RF.cpp
	Jacobian/DKM_wrt_RF.cpp
)

# Create an executable
qi_create_bin(${PROG} ${_srcs} DEPENDS QI)

# Tell CMake that imitation_robot depends on ALCOMMON and
# ALPROXIES.
# This will set the libraries to link imitation_robot with,
# the include paths, and so on

# qi_use_lib(${PROG} ALCOMMON ALPROXIES BOOST )

#Compiling in Release
#set(CMAKE_BUILD_TYPE Release)
