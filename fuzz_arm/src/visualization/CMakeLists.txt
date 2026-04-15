add_library(visualization_lib STATIC
    joint_angle_publisher.cpp
    voxel_wireframe_generator.cpp
)
target_include_directories(visualization_lib PUBLIC .)
target_link_libraries(visualization_lib PUBLIC common_lib status_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(visualization_lib Eigen3)
else()
    target_link_libraries(visualization_lib PUBLIC Eigen3::Eigen)
endif()
