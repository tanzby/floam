add_executable(test_factor test_factor.cpp)
target_link_libraries(test_factor ${PROJECT_NAME})

add_executable(test_align test_align.cpp point_cloud.h point_cloud.cpp visualization.h)
target_link_libraries(test_align ${PROJECT_NAME})