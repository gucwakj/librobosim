# project: LibRoboSim linkbot demos
project (LibRoboSim_Linkbot_Demos)

# add executable
add_executable (linkbot_one one.cpp)
add_executable (linkbot_two two.cpp)
add_executable (linkbot_group group.cpp)

# link library against dependencies
target_link_libraries (linkbot_one ${EXTRA_LIBS})
target_link_libraries (linkbot_two ${EXTRA_LIBS})
target_link_libraries (linkbot_group ${EXTRA_LIBS})

