# project: libRoboSim linkbot demos
project (LIBROBOSIM_LINKBOT_DEMOS)

# add executable
add_executable (linkbot_one one.cpp)
add_executable (linkbot_two two.cpp)
add_executable (linkbot_dog dog.cpp)

# link library against dependencies
target_link_libraries (linkbot_one ${EXTRA_LIBS})
target_link_libraries (linkbot_two ${EXTRA_LIBS})
target_link_libraries (linkbot_dog ${EXTRA_LIBS})

