add_executable(buzzsim_helper buzzsim_helper.cpp)
target_link_libraries(buzzsim_helper ${catkin_LIBRARIES})

add_executable(particle_filter particle_filter.cpp)
target_link_libraries(particle_filter ${catkin_LIBRARIES})

