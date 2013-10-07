#
# CMake include file for half library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

TARGET_LINK_LIBRARIES(${TARGET} half)

IF (NOT __HALF_CMAKE_INCLUDED)
  SET(__HALF_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/externals/half half)
ENDIF (NOT __HALF_CMAKE_INCLUDED)
