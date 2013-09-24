#
# CMake build file for serial library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 30-03-2010 (wcaarls): Initial revision
# 16-06-2010 (evanbreda): can choose between rt an non rt now

# Preamble
PROJECT(serial)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../..)
SET(TARGET serial)

# Specify sources
ADD_LIBRARY(${TARGET} STATIC ${BASE_DIR}/LxSerial.cpp)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)
