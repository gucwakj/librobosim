# project: libRoboSim
project (LIBROBOSIM)

# Set minimum cmake version
cmake_minimum_required (VERSION 2.8)

# set the version number
set (LIBROBOSIM_VERSION 0.0.70)
set (LIBROBOSIM_SO_VERSION 0)

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
	"${PROJECT_SOURCE_DIR}/include"
	"${PROJECT_BINARY_DIR}"
)

# add librs library
include_directories ("/home/kgucwa/projects/librs/include")
link_directories ("/home/kgucwa/projects/librs/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsSim debug rsSimd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsScene debug rsScened)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsCallback debug rsCallbackd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsXML debug rsXMLd)

# add ODE headers
if (MSVC)
	include_directories ("/home/kgucwa/projects/librs/deps/ode/include")
else (MSVC)
	include_directories ("/home/kgucwa/projects/librs/deps/ode/sys/include")
endif (MSVC)

# ODE options
add_definitions (-DdDOUBLE)
add_definitions (-DdNODEBUG)

# add OpenSceneGraph headers
include_directories ("/home/kgucwa/projects/librs/deps/osg/build/include")
include_directories ("/home/kgucwa/projects/librs/deps/osg/include")

# add tinyxml2 headers
include_directories ("/home/kgucwa/projects/librs/deps/tinyxml2")

# add source files to library
set (SRCS ${SRCS}
	src/robosim.cpp
	src/linkbot.cpp
)

# set output directory for shared objects
set (OUTPUT_LIBDIR "${LIBROBOSIM_BINARY_DIR}/lib")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_LIBDIR})
make_directory (${OUTPUT_LIBDIR})

# add the library
add_library (robosim SHARED ${SRCS})

# link library against dependencies
target_link_libraries (robosim ${EXTRA_LIBS})

# set version properties
set_target_properties (robosim PROPERTIES VERSION ${LIBROBOSIM_VERSION} SOVERSION ${LIBROBOSIM_SO_VERSION})

# set output directory for demos
set (OUTPUT_BINDIR "${LIBROBOSIM_BINARY_DIR}/bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BINDIR})
make_directory (${OUTPUT_BINDIR})

# link library against dependencies
target_link_libraries (robosim ${EXTRA_LIBS})

# build demos
add_subdirectory ("${LIBROBOSIM_SOURCE_DIR}/demo/")

