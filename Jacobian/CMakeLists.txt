## Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

cmake_minimum_required(VERSION 2.8)

set(PROG JacobianNao)

project(${PROG})
find_package(qibuild)
find_package(qimodule)

# Make sure the generated header can be found:
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(".")

qi_create_lib(JacobianNao SHARED
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

qi_use_lib(${PROG} ALCOMMON ALPROXIES BOOST )

# Tell CMake that imitation_robot depends on ALCOMMON and
# ALPROXIES.


 
qi_install_header(Jacobian/Jacobian.h SUBFOLDER JacobianNao)

qi_stage_lib(JacobianNao)

