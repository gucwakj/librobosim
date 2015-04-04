# project: libRoboSim mindstorms demos
project (LIBROBOSIM_MINDSTORMS_DEMOS)

# add executable
add_executable (mindstorms_one one.cpp)
add_executable (mindstorms_two two.cpp)
add_executable (mindstorms_group group.cpp)

# link library against dependencies
target_link_libraries (mindstorms_one ${EXTRA_LIBS})
target_link_libraries (mindstorms_two ${EXTRA_LIBS})
target_link_libraries (mindstorms_group ${EXTRA_LIBS})

