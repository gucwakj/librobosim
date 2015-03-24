# project: libRoboSim demos
project (LIBROBOSIM_DEMOS)

# add librobosim library
include_directories ("${LIBROBOSIM_SOURCE_DIR}/include")
link_directories ("${LIBROBOSIM_SOURCE_DIR}/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized robosim debug robosimd)

# add each subdirectory
if (ENABLE_LINKBOT)
	add_subdirectory (linkbot)
endif (ENABLE_LINKBOT)
if (ENABLE_MINDSTORMS)
	add_subdirectory (mindstorms)
endif (ENABLE_MINDSTORMS)

