# project: LibRoboSim
project (LibRoboSim)

# Set minimum cmake version
cmake_minimum_required (VERSION 2.8)

# set the version number
set (LibRoboSim_VERSION 0.0.70)
set (LibRoboSim_SO_VERSION 0)

# verbose output
set (CMAKE_VERBOSE_MAKEFILE off)

# default to release build
set (CMAKE_BUILD_TYPE Release)

# set the build postfix extension
set (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add build postfix type")
set (CMAKE_RELEASE_POSTFIX "" CACHE STRING "add build postfix type")
set (CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add build postfix type")
set (CMAKE_RELWITHDEBINFO_POSTFIX "rd" CACHE STRING "add build postfix type")
if (CMAKE_BUILD_TYPE MATCHES "Release")
	set (CMAKE_BUILD_POSTFIX "")
elseif (CMAKE_BUILD_TYPE MATCHES "Debug")
	set (CMAKE_BUILD_POSTFIX "d")
elseif (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
	set (CMAKE_BUILD_POSTFIX "s")
elseif (CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
	set (CMAKE_BUILD_POSTFIX "rd")
endif ()

# suppress developer warnings
cmake_policy (SET CMP0015 NEW)

# set developer warnings
if (WIN32)
	add_definitions (-D_CRT_SECURE_NO_WARNINGS)
else ()
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-write-strings")
endif ()

# include directory
include_directories (
	"${PROJECT_SOURCE_DIR}/include"
	"${PROJECT_BINARY_DIR}"
)

# link directory
link_directories ("${LibRoboSim_BINARY_DIR}/lib")

# add ODE headers
if (WIN32)
	include_directories ("C:/Users/kgucwa/projects/librs/deps/ode/include")
	link_directories ("C:/Users/kgucwa/projects/librs/deps/ode/lib/Release")
	set (EXTRA_LIBS ${EXTRA_LIBS} optimized ode debug oded)
else ()
	include_directories ("/home/kgucwa/projects/librs/deps/ode/sys/include")
endif ()

# ODE options
add_definitions (-DdDOUBLE)
add_definitions (-DdNODEBUG)

# add OpenSceneGraph headers
if (WIN32)
	include_directories ("C:/Users/kgucwa/projects/librs/deps/osg3.4.0/build/include")
	include_directories ("C:/Users/kgucwa/projects/librs/deps/osg3.4.0/include")
	link_directories ("C:/Users/kgucwa/projects/librs/deps/osg3.4.0/build/lib")
else ()
	include_directories ("/home/kgucwa/projects/librs/deps/osg3.4.0/build/include")
	include_directories ("/home/kgucwa/projects/librs/deps/osg3.4.0/include")
	link_directories ("/home/kgucwa/projects/librs/deps/osg3.4.0/build/lib")
endif()
set (EXTRA_LIBS ${EXTRA_LIBS} optimized OpenThreads debug OpenThreadsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osg debug osgd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgDB debug osgDBd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgFX debug osgFXd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgGA debug osgGAd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgShadow debug osgShadowd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgText debug osgTextd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgUtil debug osgUtild)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgViewer debug osgViewerd)

# add tinyxml2 headers
if (WIN32)
	include_directories ("C:/Users/kgucwa/projects/librs/deps/tinyxml2")
	link_directories ("C:/Users/kgucwa/projects/librs/deps/tinyxml2/build/lib")
else ()
	include_directories ("/home/kgucwa/projects/librs/deps/tinyxml2")
endif()

# add LibRS library
if (WIN32)
	include_directories ("C:/Users/kgucwa/projects/librs/include")
	include_directories ("C:/Users/kgucwa/projects/librs/build")
	link_directories ("C:/Users/kgucwa/projects/librs/build/lib")
else ()
	include_directories ("/home/kgucwa/projects/librs/include")
	include_directories ("/home/kgucwa/projects/librs/build")
	link_directories ("/home/kgucwa/projects/librs/build/lib")
endif ()
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rs debug rsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsCallback debug rsCallbackd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsScene debug rsScened)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsSim debug rsSimd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsXML debug rsXMLd)

# options for individual robot compilation
option (ENABLE_DOF "Enable dof compilation" OFF)
option (ENABLE_LINKBOT "Enable linkbot compilation" ON)
option (ENABLE_MINDSTORMS "Enable mindstorms compilation" ON)
option (ENABLE_MINIDOF "Enable minidof compilation" OFF)

# add source files to library
set (SRCS ${SRCS}
	src/robosim.cpp
	src/robot.cpp
	src/rgbhashtable.c
)
if (ENABLE_DOF)
	set (SRCS ${SRCS} src/dof.cpp)
endif (ENABLE_DOF)
if (ENABLE_LINKBOT)
	set (SRCS ${SRCS} src/linkbot.cpp)
endif (ENABLE_LINKBOT)
if (ENABLE_MINDSTORMS)
	set (SRCS ${SRCS} src/mindstorms.cpp)
endif (ENABLE_MINDSTORMS)
if (ENABLE_MINIDOF)
	set (SRCS ${SRCS} src/minidof.cpp)
endif (ENABLE_MINIDOF)

# set output directories
set (OUTPUT_LIBDIR "${LibRoboSim_BINARY_DIR}/lib")
set (OUTPUT_BINDIR "${LibRoboSim_BINARY_DIR}/bin")
foreach (CONF ${CMAKE_CONFIGURATION_TYPES})
	string (TOUPPER "${CONF}" CONF)
	set ("CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_LIBDIR}")
	set ("CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_BINDIR}")
	if (WIN32)
		set ("CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_BINDIR}")
	else ()
		set ("CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_LIBDIR}")
	endif ()
endforeach ()
set ("CMAKE_RUNTIME_OUTPUT_DIRECTORY" "${OUTPUT_BINDIR}")
set ("CMAKE_LIBRARY_OUTPUT_DIRECTORY" "${OUTPUT_LIBDIR}")
make_directory (${OUTPUT_LIBDIR})
make_directory (${OUTPUT_BINDIR})

# add the library
add_library (robosim SHARED ${SRCS})

# link library against dependencies
target_link_libraries (robosim ${EXTRA_LIBS})

# set version properties
set_target_properties (robosim PROPERTIES VERSION ${LibRoboSim_VERSION} SOVERSION ${LibRoboSim_SO_VERSION})

# link library against dependencies
target_link_libraries (robosim ${EXTRA_LIBS})

# build demos
add_subdirectory ("${LibRoboSim_SOURCE_DIR}/demo/")

