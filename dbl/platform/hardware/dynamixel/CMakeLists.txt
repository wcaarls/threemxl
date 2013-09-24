#
# CMake build file for dynamixel library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 30-03-2010 (wcaarls): Initial revision
# 15-04-2010 (evbreda): Devided code over several files
# 16-06-2010 (evanbreda): can choose between rt an non rt now
#

#SET(XENOMAI 1)

# Preamble
PROJECT(dynamixel)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../..)
SET(TARGET dynamixel)

ADD_LIBRARY(${TARGET} STATIC ${BASE_DIR}/CDxlCom.cpp
                      ${BASE_DIR}/CDxlSerialPacketHandler.cpp
                      ${BASE_DIR}/CDxlGeneric.cpp
                      ${BASE_DIR}/dynamixel/Dynamixel.cpp
                      ${BASE_DIR}/CDxlConfig.cpp
                      ${BASE_DIR}/CDxlGroup.cpp
                      ${BASE_DIR}/3mxl/3mxl.cpp
           )

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/serial/serial.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/externals/half/half.cmake)
