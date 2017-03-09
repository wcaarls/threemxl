cmake_minimum_required(VERSION 2.8.3)
project(threemxl)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Readline REQUIRED)

# Check for libftdipp
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFTDIPP libftdipp)
if (LIBFTDIPP_FOUND)
  set(FTDI_SOURCE src/LxFTDI.cpp)
  message("-- using FTDI (libftdipp1)")
  add_definitions(-DWITH_FTDIPP)
else()
pkg_check_modules(LIBFTDIPP libftdipp1)
if (LIBFTDIPP_FOUND)
  set(FTDI_SOURCE src/LxFTDI.cpp)
  message("-- using FTDI (libftdipp)")
  add_definitions(-DWITH_FTDIPP)
endif()
endif()

# Get tinyxml and muparser
pkg_check_modules(TINYXML REQUIRED tinyxml)
pkg_check_modules(MUPARSER REQUIRED muparser)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${READLINE_INCLUDE_DIR} ${TINYXML_INCLUDE_DIRS} ${MUPARSER_INCLUDE_DIRS} ${LIBFTDIPP_INCLUDE_DIRS})

# Setup DBL repository stuff
set(WORKSPACE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Console
set(TARGET console)
add_executable(${TARGET} src/console.cpp ${FTDI_SOURCE})
include(${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)
include(${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
target_link_libraries(${TARGET} ${READLINE_LIBRARY} ${TINYXML_LDFLAGS} ${MUPARSER_LDFLAGS} ${LIBFTDIPP_LDFLAGS} -lpthread)
