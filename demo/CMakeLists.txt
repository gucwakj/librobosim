# project: libRoboSim demos
project (LibRoboSim_Demos)

# add librobosim library
include_directories ("${LibRoboSim_SOURCE_DIR}/include")
link_directories ("${LibRoboSim_SOURCE_DIR}/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized robosim debug robosimd)

# add each subdirectory
if (ENABLE_DOF)
	add_subdirectory (dof)
endif (ENABLE_DOF)
if (ENABLE_LINKBOT)
	add_subdirectory (linkbot)
endif (ENABLE_LINKBOT)
if (ENABLE_MINDSTORMS)
	add_subdirectory (mindstorms)
endif (ENABLE_MINDSTORMS)
if (ENABLE_MINIDOF)
	add_subdirectory (minidof)
endif (ENABLE_MINIDOF)

