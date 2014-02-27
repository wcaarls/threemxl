#
# CMake include file for dynamixel library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 30-03-2010 (wcaarls): Initial revision
#

INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/serial/serial.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)

TARGET_LINK_LIBRARIES(${TARGET} dynamixel)
ADD_DEPENDENCIES(${TARGET} dynamixel)

IF (NOT __DYNAMIXEL_CMAKE_INCLUDED)
  SET(__DYNAMIXEL_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel)
ENDIF (NOT __DYNAMIXEL_CMAKE_INCLUDED)
