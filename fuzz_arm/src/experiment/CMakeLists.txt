add_library(experiment_lib STATIC
    experiment_config.cpp
    scenario_definition.cpp
)
target_include_directories(experiment_lib PUBLIC .)
target_link_libraries(experiment_lib PUBLIC common_lib kinematics_lib control_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(experiment_lib Eigen3)
else()
    target_link_libraries(experiment_lib PUBLIC Eigen3::Eigen)
endif()
