cmake_minimum_required(VERSION 3.0)

project(JacobianNAO VERSION 2.0.1)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)

FILE(GLOB _include include/Jacobian.h)
FILE(GLOB _src src/DGM.cpp)

add_library(JACOBIAN STATIC ${_src} ${_include})

INSTALL(TARGETS JACOBIAN
  LIBRARY DESTINATION ${JACOBIAN_LIBRARIES}
  ARCHIVE DESTINATION ${JACOBIAN_LIBRARIES}
  RUNTIME DESTINATION ${JACOBIAN_LIBRARIES}
)

# headers
INSTALL(FILES ${JACOBIAN_INCLUDE_DIRS}/include/Jacobian.h
  DESTINATION include)

INSTALL(DIRECTORY ${JACOBIAN_INCLUDE_DIRS}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
#   PATTERN "*.ipp"
#   PATTERN ".svn" EXCLUDE)
)