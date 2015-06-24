# project: libRoboSim linkbot demos
project (LIBROBOSIM_LINKBOT_DEMOS)

# add executable
add_executable (linkbot_one one.cpp)
add_executable (linkbot_two two.cpp)
add_executable (linkbot_dog dog.cpp)
add_executable (linkbot_group group.cpp)
add_executable (linkbot_salamander salamander.cpp)
add_executable (linkbot_sinusoid sinusoid.cpp)
add_executable (linkbot_snake snake.cpp)

# link library against dependencies
target_link_libraries (linkbot_one ${EXTRA_LIBS})
target_link_libraries (linkbot_two ${EXTRA_LIBS})
target_link_libraries (linkbot_dog ${EXTRA_LIBS})
target_link_libraries (linkbot_group ${EXTRA_LIBS})
target_link_libraries (linkbot_salamander ${EXTRA_LIBS})
target_link_libraries (linkbot_sinusoid ${EXTRA_LIBS})
target_link_libraries (linkbot_snake ${EXTRA_LIBS})

