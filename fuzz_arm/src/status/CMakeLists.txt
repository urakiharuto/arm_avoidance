add_library(status_lib STATIC
    manipulability.cpp
    dynamic_manipulability.cpp
)
target_include_directories(status_lib PUBLIC .)
target_link_libraries(status_lib PUBLIC common_lib kinematics_lib gng_lib)
if(BUILD_WITH_ROS2)
    ament_target_dependencies(status_lib Eigen3)
else()
    target_link_libraries(status_lib PUBLIC Eigen3::Eigen)
endif()
