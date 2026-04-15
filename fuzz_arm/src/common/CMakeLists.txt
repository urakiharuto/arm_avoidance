add_library(common_lib STATIC
    config_manager.cpp
    geometry_utils.cpp
)
target_include_directories(common_lib PUBLIC .)
if(BUILD_WITH_ROS2)
    ament_target_dependencies(common_lib Eigen3)
else()
    target_link_libraries(common_lib PUBLIC Eigen3::Eigen)
endif()
