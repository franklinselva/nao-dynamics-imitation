cmake_minimum_required(VERSION 3.0)

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
include_directories(.)


############################################################
#################### BUILD LIBRARIES #######################
############################################################

#### qpOASES####

set(qpOASES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libs/qpOASES)
FILE(GLOB qpOASES_include ${qpOASES_PATH}/include/*.hpp ${qpOASES_PATH}/include/qpOASES/)
FILE(GLOB qpOASES_src ${qpOASES_PATH}/src/*.cpp)

set(QPOASES_LIBRARIES ${qpOASES_PATH}/libs)
set(QPOASES_INCLUDE_DIRS ${qpOASES_PATH}/include)

add_library(qpOASES SHARED ${qpOASES_src} ${qpOASES_include})

INSTALL(TARGETS qpOASES
  LIBRARY DESTINATION ${QPOASES_LIBRARIES}
  ARCHIVE DESTINATION ${QPOASES_LIBRARIES}
  RUNTIME DESTINATION ${QPOASES_LIBRARIES}
)

# lib files
INSTALL(FILES ${QPOASES_LIBRARIES}/libqpOASES.a 
	DESTINATION lib)

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

set(JACOBIAN_LIBRARIES ${JACOBIAN_PATH}/libs)
set(JACOBIAN_INCLUDE_DIRS ${JACOBIAN_PATH}/include)

qi_create_lib(JACOBIAN ${JACOBIAN_include} ${JACOBIAN_src})
qi_stage_lib(JACOBIAN)

# add_library(JACOBIAN SHARED ${JACOBIAN_src} ${JACOBIAN_include})

# INSTALL(TARGETS JACOBIAN
#   LIBRARY DESTINATION ${JACOBIAN_LIBRARIES}
#   ARCHIVE DESTINATION ${JACOBIAN_LIBRARIES}
#   RUNTIME DESTINATION ${JACOBIAN_LIBRARIES}
# )

# # headers
# INSTALL(FILES ${JACOBIAN_INCLUDE_DIRS}/include/Jacobian.h
#   DESTINATION include)

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

# qi_use_lib(${PROG} ALCOMMON ALPROXIES BOOST )

#Compiling in Release
#set(CMAKE_BUILD_TYPE Release)
