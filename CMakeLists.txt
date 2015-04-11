# project: libRoboSim
project (LIBROBOSIM)

# Set minimum cmake version
cmake_minimum_required (VERSION 2.8)

# set the version number
set (LIBROBOSIM_VERSION 0.0.70)
set (LIBROBOSIM_SO_VERSION 0)

# verbose ouptut
set (CMAKE_VERBOSE_MAKEFILE off)

# set the build postfix extension for debugging
if (CMAKE_BUILD_TYPE MATCHES "Debug")
	set (CMAKE_DEBUG_POSTFIX "d")
endif ()

# set gcc compiler version
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# supress developer warnings
cmake_policy (SET CMP0015 NEW)

# set developer warnings
if (WIN32)
	add_definitions (-D_CRT_SECURE_NO_WARNINGS)
else ()
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
endif ()

# include directory
include_directories (
	"${PROJECT_SOURCE_DIR}/include"
	"${PROJECT_BINARY_DIR}"
)

# link directory
link_directories ("${LIBROBOSIM_BINARY_DIR}/lib")

# add librs library
include_directories ("/home/kgucwa/projects/librs/include")
link_directories ("/home/kgucwa/projects/librs/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rs debug rsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsSim debug rsSimd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsScene debug rsScened)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsCallback debug rsCallbackd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsXML debug rsXMLd)

# add ODE headers
if (WIN32)
	include_directories ("/home/kgucwa/projects/librs/deps/ode/include")
else ()
	include_directories ("/home/kgucwa/projects/librs/deps/ode/sys/include")
endif ()

# ODE options
add_definitions (-DdDOUBLE)
add_definitions (-DdNODEBUG)

# add OpenSceneGraph headers
include_directories ("/home/kgucwa/projects/librs/deps/osg/build/include")
include_directories ("/home/kgucwa/projects/librs/deps/osg/include")

# add tinyxml2 headers
include_directories ("/home/kgucwa/projects/librs/deps/tinyxml2")

# options for individual robot compilation
set (ENABLE_LINKBOT ON)
set (ENABLE_MINDSTORMS ON)

# add source files to library
set (SRCS ${SRCS}
	src/robosim.cpp
	src/robot.cpp
)
if (ENABLE_LINKBOT)
	set (SRCS ${SRCS} src/linkbot.cpp)
endif (ENABLE_LINKBOT)
if (ENABLE_MINDSTORMS)
	set (SRCS ${SRCS} src/mindstorms.cpp)
endif (ENABLE_MINDSTORMS)

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

