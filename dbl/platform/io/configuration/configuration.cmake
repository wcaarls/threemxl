#
# CMake include file for configuration library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	TARGET_LINK_LIBRARIES(${TARGET} configuration -force_load)
else()
	TARGET_LINK_LIBRARIES(${TARGET} configuration)
endif()

IF (NOT __CONFIGURATION_CMAKE_INCLUDED)
  SET(__CONFIGURATION_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/io/configuration)
ENDIF (NOT __CONFIGURATION_CMAKE_INCLUDED)
