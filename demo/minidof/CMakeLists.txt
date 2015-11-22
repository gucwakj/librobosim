# project: LibRoboSim MiniDof demos
project (LibRoboSim_MiniDof_Demos)

# add executable
add_executable (minidof_one one.cpp)
add_executable (minidof_two two.cpp)
add_executable (minidof_group group.cpp)

# link library against dependencies
target_link_libraries (minidof_one ${EXTRA_LIBS})
target_link_libraries (minidof_two ${EXTRA_LIBS})
target_link_libraries (minidof_group ${EXTRA_LIBS})

