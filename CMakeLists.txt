cmake_minimum_required(VERSION 3.0.2)

# Define the name of the reference name
set(PROG nao_dynamics_imitation)

# Define the name of the project
project(${PROG})


############################################################
#################### FLAGS AND VARIABLES ###################
############################################################

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

# VREP Simulation Specific Variables
set(VREP_CFLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D__linux")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${VREP_CFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VREP_CFLAGS} -std=gnu++11")

# This include enables the use of qibuild framework
find_package(qibuild REQUIRED)

# Make sure the generated header can be found:
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(.)
include_directories(include)

############################################################
######################### VREP #############################
############################################################

set(VREP_API_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libs/vrep)

set(vrep_include
	${VREP_API_PATH}/include/extApi.h
  ${VREP_API_PATH}/include/extApiInternal.h
  ${VREP_API_PATH}/include/extApiPlatform.h
  ${VREP_API_PATH}/include/v_repConst.h
  ${VREP_API_PATH}/include/vrep_driver.h
)

set(vrep_src
	${VREP_API_PATH}/src/extApi.c
  ${VREP_API_PATH}/src/extApiPlatform.c
)

set(VREPAPI_INCLUDE_DIRS ${VREP_API_PATH}/include)

qi_create_lib(vrepAPI SHARED ${vrep_include} ${vrep_src})
qi_stage_lib(vrepAPI)

############################################################
####################### qpOASES ############################
############################################################

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


############################################################
####################### JACOBIAN ###########################
############################################################

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
	src/xsens_imitation.cpp
	src/dynamics_param.cpp
	src/xsens_tool.cpp
	src/robot.cpp
	src/balance_control.cpp
)
set(_include
	include/xsens_tool.h
	include/dynamics_param.h
	include/robot.h
	include/balance_control.h
)

set(_sim_srcs
	src/balance_control.cpp
	src/xsens_imitation_sim.cpp
	src/xsens_tool.cpp
  src/vrep_sim_handler.cpp
	src/robot.cpp
)
set(_sim_include
	include/balance_control.h
	include/xsens_tool.h
	include/robot.h
	include/vrep_sim_handler.h
)

# Imitation Executable
qi_create_bin(${PROG} ${_srcs} ${_include} DEPENDS QI qpOASES JACOBIAN)
qi_use_lib(${PROG} ALCOMMON ALPROXIES BOOST QI qpOASES JACOBIAN )

# Simulation Test Executable
qi_create_bin(vrep_simulation_test src/simulation_test_2.cpp DEPENDS QI vrepAPI)
qi_use_lib(vrep_simulation_test ALPROXIES ${ALPROXIES_DEPENDS} ALVISION ${ALVISION_DEPENDS} vrepAPI)

# Imitation Simulator Executable
qi_create_bin(${PROG}_sim ${_sim_srcs} ${_sim_include} DEPENDS QI qpOASES JACOBIAN vrepAPI)
qi_use_lib(${PROG}_sim ALCOMMON ALPROXIES BOOST QI qpOASES JACOBIAN vrepAPI)
