add_library(control_lib STATIC
    trajectory_safety_manager.cpp
)
target_include_directories(control_lib PUBLIC .)
target_link_libraries(control_lib PUBLIC common_lib kinematics_lib gng_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(control_lib Eigen3)
else()
    target_link_libraries(control_lib PUBLIC Eigen3::Eigen)
endif()
