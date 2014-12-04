# project: libRoboSim linkbot demos
project (LIBROBOSIM_LINKBOT_DEMOS)

# add source files to library
set (SRCS ${SRCS} robosim.cpp)

# add executable
add_executable (linkbottest test.cpp)

# link library against dependencies
target_link_libraries (linkbottest ${EXTRA_LIBS})

