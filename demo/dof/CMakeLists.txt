# project: LibRoboSim Dof demos
project (LibRoboSim_Dof_Demos)

# add executable
add_executable (dof_one one.cpp)
add_executable (dof_two two.cpp)
add_executable (dof_group group.cpp)

# link library against dependencies
target_link_libraries (dof_one ${EXTRA_LIBS})
target_link_libraries (dof_two ${EXTRA_LIBS})
target_link_libraries (dof_group ${EXTRA_LIBS})

