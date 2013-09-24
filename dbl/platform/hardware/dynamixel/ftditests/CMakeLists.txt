#
# CMake build file for dynamixel-tests using FTDI serial communication
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 30-03-2010 (wcaarls): Initial revision
#

# Preamble
PROJECT(dynamixel-tests)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_BUILD_TYPE Release)



# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../../..)

# dxl-all
SET(TARGET dxl-all)
ADD_EXECUTABLE(${TARGET} ${BASE_DIR}/dxl-all.cpp)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/include.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)

# dxl-goto
SET(TARGET dxl-goto)
ADD_EXECUTABLE(${TARGET} ${BASE_DIR}/dxl-goto.cpp)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/include.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)

# dxl-voltage
SET(TARGET dxl-voltage)
ADD_EXECUTABLE(${TARGET} ${BASE_DIR}/dxl-voltage.cpp)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/include.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/math/math.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)

# dxl-changeid
SET(TARGET dxl-changeid)
ADD_EXECUTABLE(${TARGET} ${BASE_DIR}/dxl-changeid.cpp)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/include.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)

# testdxlmultiport
SET(TARGET testdxlmultiport)
ADD_EXECUTABLE(${TARGET} ${BASE_DIR}/testdxlmultiport.cpp)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/threading/threading.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/time/time.cmake)


