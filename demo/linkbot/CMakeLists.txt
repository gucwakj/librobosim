# Build LinkbotDemo
project (LINBOTDEMO)

# Set minimum cmake version
cmake_minimum_required (VERSION 2.8)

# verbose ouptut
set (CMAKE_VERBOSE_MAKEFILE off)

# set build type
set (CMAKE_BUILD_TYPE "Release")

# set the build postfix extension according to what configuration is being built
if (CMAKE_BUILD_TYPE MATCHES "Release")
    set (CMAKE_BUILD_POSTFIX "")
elseif (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
    set (CMAKE_BUILD_POSTFIX "s")
elseif (CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
    set (CMAKE_BUILD_POSTFIX "rd")
elseif (CMAKE_BUILD_TYPE MATCHES "Debug")
    set (CMAKE_BUILD_POSTFIX "d")
else()
    set (CMAKE_BUILD_POSTFIX "")
endif()

# supress developer warnings
cmake_policy (SET CMP0015 NEW)

# set developer warnings
if (MSVC)
	add_definitions (-D_CRT_SECURE_NO_WARNINGS)
else (MSVC)
	set (CMAKE_CXX_FLAGS "-Wno-write-strings")
endif (MSVC)

# include directory
include_directories (
	"${PROJECT_SOURCE_DIR}"
	"${PROJECT_BINARY_DIR}"
)

# add ODE headers
if (MSVC)
	include_directories ("/home/kgucwa/projects/robot-sim/src/opende/include")
else (MSVC)
	include_directories ("/home/kgucwa/projects/robot-sim/src/opende/sys/include")
endif (MSVC)

# ODE options
add_definitions (-DdDOUBLE)
add_definitions (-DdNODEBUG)

# add OpenSceneGraph headers
include_directories ("/home/kgucwa/projects/robot-sim/src/osg/build/include")
include_directories ("/home/kgucwa/projects/robot-sim/src/osg/include")

# add tinyxml2 headers
include_directories ("/home/kgucwa/projects/robot-sim/src/tinyxml2")

# add librs library
include_directories ("/home/kgucwa/projects/playground/librs/inc")
link_directories ("/home/kgucwa/projects/playground/librs/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsSim debug rsSimd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsScene debug rsScened)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsXML debug rsXMLd)

# add librobosim library
include_directories ("/home/kgucwa/projects/playground/librobosim")
link_directories ("/home/kgucwa/projects/playground/librobosim/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized robosim debug robosimd)

# add source files to library
set (SRCS ${SRCS} robosim.cpp)

# add executable
add_executable (test test.cpp)

# link library against dependencies
target_link_libraries (test ${EXTRA_LIBS})

