#
# CMake include file for serial library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 30-03-2010 (wcaarls): Initial revision
#

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	TARGET_LINK_LIBRARIES(${TARGET} serial -force_load)
else()
	TARGET_LINK_LIBRARIES(${TARGET} serial)
endif()

IF (NOT __SERIAL_CMAKE_INCLUDED)
  SET(__SERIAL_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/hardware/serial)
ENDIF (NOT __SERIAL_CMAKE_INCLUDED)
