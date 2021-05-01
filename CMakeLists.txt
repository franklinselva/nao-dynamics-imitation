cmake_minimum_required(VERSION 3.0.2)

# Define the name of the reference name
set(PROG nao_dynamics_imitation)

# Define the name of the project
project(${PROG})

include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)


# NAOqi SDK is available only on Mach 64 with architecture -x86_64
set(CMAKE_REQUIRED_LINK_OPTIONS "-arch;x86_64")
check_c_compiler_flag("-arch x86_64" x86_64Supported)
check_cxx_compiler_flag("-arch x86_64" x86_64Supported)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
set(CMAKE_OSX_ARCHITECTURES x86_64) # Mac Specific Variables
set(CMAKE_BUILD_TYPE Release)

# This include enables the use of qibuild framework
find_package(qibuild REQUIRED)

# Make sure the generated header can be found:
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(.)
include_directories(include)

############################################################
#################### BUILD LIBRARIES #######################
############################################################

#### qpOASES####

set(qpOASES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libs/qpOASES)
FILE(GLOB qpOASES_include ${qpOASES_PATH}/include/*.hpp ${qpOASES_PATH}/include/qpOASES/)
FILE(GLOB qpOASES_src ${qpOASES_PATH}/src/*.cpp)

# set(QPOASES_LIBRARIES ${qpOASES_PATH}/libs/libqpOASES.dylib)
set(QPOASES_INCLUDE_DIRS ${qpOASES_PATH}/include)

qi_create_lib(qpOASES SHARED ${qpOASES_src} ${qpOASES_include})
qi_stage_lib(qpOASES)


# headers
INSTALL(FILES ${QPOASES_INCLUDE_DIRS}/include/qpOASES.hpp
  DESTINATION include)

INSTALL(DIRECTORY ${QPOASES_INCLUDE_DIRS}/include/qpOASES
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*.ipp"
  PATTERN ".svn" EXCLUDE)


#### JACOBIAN ####
set(JACOBIAN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libs/Jacobian)
FILE(GLOB JACOBIAN_include ${JACOBIAN_PATH}/include/Jacobian.h)
FILE(GLOB JACOBIAN_src ${JACOBIAN_PATH}/src/*.cpp)

set(JACOBIAN_INCLUDE_DIRS ${JACOBIAN_PATH}/include)

qi_create_lib(JACOBIAN SHARED ${JACOBIAN_include} ${JACOBIAN_src})
qi_stage_lib(JACOBIAN)


############################################################
#################### Create APPLICATION ####################
############################################################

# Create a list of source files
set(_srcs
	src/xsens_imitation_v7.cpp
	src/xsens_tool.cpp
	src/robot.cpp
)
set(_include
	include/xsens_tool.h
	include/robot.h
)

# Create an executable
qi_create_bin(${PROG} ${_srcs} ${_include} DEPENDS QI qpOASES JACOBIAN)

# Tell CMake that imitation_robot depends on ALCOMMON and
# ALPROXIES.
# This will set the libraries to link imitation_robot with,
# the include paths, and so on

qi_use_lib(${PROG} ALCOMMON ALPROXIES BOOST QI qpOASES JACOBIAN )
