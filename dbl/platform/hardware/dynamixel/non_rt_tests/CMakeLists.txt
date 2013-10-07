#
# CMake build file for dynamixel-tests
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 30-03-2010 (wcaarls): Initial revision
# 01-04-2010 (evanbreda): changed way xenomai is linked to this project
# 11-08-2011 (evanbreda): nonrt version

ADD_DEFINITIONS(-DDEBUG)

# Preamble
PROJECT(dynamixel-tests-nonrt)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_BUILD_TYPE Release)



# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../../..)


# 3mxl-all
SET(TARGET 3mxl-all)
ADD_EXECUTABLE(${TARGET} ${BASE_DIR}/3mxl-all.cpp)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)


