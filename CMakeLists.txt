cmake_minimum_required(VERSION 2.8.3)
project(threemxl)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin   REQUIRED COMPONENTS roscpp shared_serial)
find_package(Readline REQUIRED)

# Check for libftdipp
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFTDIPP QUIET libftdipp)
if (LIBFTDIPP_FOUND)
  set(FTDI_SOURCE src/LxFTDI.cpp)
  message("-- using FTDI")
endif()

# Get tinyxml and muparser
pkg_check_modules(TINYXML REQUIRED tinyxml)
pkg_check_modules(MUPARSER REQUIRED muparser)

catkin_package(
  INCLUDE_DIRS   include
  LIBRARIES      ${PROJECT_NAME} ${TINYXML_LIBRARIES} ${MUPARSER_LIBRARIES} ${LIBFTDIPP_LIBRARIES}
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${READLINE_INCLUDE_DIR} ${TINYXML_INCLUDE_DIRS} ${MUPARSER_INCLUDE_DIRS} ${LIBFTDIPP_INCLUDE_DIRS})

# Setup DBL repository stuff
set(WORKSPACE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Build relevant DBL repo things
set(TARGET ${PROJECT_NAME})
add_library(${PROJECT_NAME} src/CDxlROSPacketHandler.cpp ${FTDI_SOURCE})

if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_link_libraries(${PROJECT_NAME} -Wl,-whole-archive)
endif()

include(${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)
include(${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)

if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_link_libraries(${PROJECT_NAME} -Wl,-no-whole-archive)
endif()

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBFTDIPP_LDFLAGS} ${TINYXML_LDFLAGS} ${MUPARSER_LDFLAGS})

# Console
add_executable(console src/console.cpp)
target_link_libraries(console ${PROJECT_NAME} ${READLINE_LIBRARY})

# Example program for hand setup
add_executable(example src/example.cpp)
target_link_libraries(example ${PROJECT_NAME})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} console example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION     ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN         "*.h"
  PATTERN         "*.hpp"
  PATTERN         ".svn" EXCLUDE
  PATTERN         "console.h" EXCLUDE
  PATTERN         "example.h" EXCLUDE
)
