cmake_minimum_required(VERSION 3.10)
project(gng_planner)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. Try to find ROS 2 (ament_cmake) ---
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    message(STATUS "ROS 2 (Humble) environment detected.")
    add_definitions(-DUSE_ROS2)
    set(BUILD_WITH_ROS2 ON)
    
    # ROS 2 packages
    find_package(rclcpp REQUIRED)
    find_package(urdf REQUIRED)
    find_package(urdfdom REQUIRED)
    find_package(tinyxml2_vendor REQUIRED)
    find_package(fcl REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(ament_index_cpp REQUIRED)
else()
    message(STATUS "Standard CMake environment detected (No ROS 2).")
    set(BUILD_WITH_ROS2 OFF)
    
    # macOS/Homebrew paths
    if(APPLE)
        list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
        add_definitions(-DGL_SILENCE_DEPRECATION)
    endif()

    # Standard package search
    find_package(OpenCV REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(tinyxml2 REQUIRED)
    find_package(console_bridge REQUIRED)
    find_package(urdfdom REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(Threads REQUIRED)
    
    # Manual FCL search for standalone
    find_package(FCL QUIET)
endif()

# --- 2. Optional ODE / Drawstuff (Common for both) ---
option(USE_ODE "Use ODE for physical simulation" ON)
if(USE_ODE)
    find_path(ODE_INCLUDE_DIR NAMES ode/ode.h HINTS /usr/local/include /opt/homebrew/include)
    find_library(ODE_LIBRARY NAMES ode HINTS /usr/local/lib /opt/homebrew/lib)
    if(ODE_INCLUDE_DIR AND ODE_LIBRARY)
        message(STATUS "ODE found: ${ODE_LIBRARY}")
        add_definitions(-DUSE_ODE)
    else()
        message(WARNING "ODE not found!")
        set(USE_ODE OFF)
    endif()
endif()

option(USE_DRAWSTUFF "Use Drawstuff for visualization" ON)
if(USE_DRAWSTUFF)
    find_library(DRAWSTUFF_LIBRARY NAMES drawstuff HINTS /usr/local/lib /opt/homebrew/lib)
    if(DRAWSTUFF_LIBRARY)
        message(STATUS "Drawstuff found: ${DRAWSTUFF_LIBRARY}")
        add_definitions(-DUSE_DRAWSTUFF)
    else()
        message(WARNING "Drawstuff not found!")
        set(USE_DRAWSTUFF OFF)
    endif()
endif()

# ImGui setup (for online_integration)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glut.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
)

# Standard definitions
add_definitions(-DPROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

if(FCL_FOUND OR BUILD_WITH_ROS2)
    add_definitions(-DUSE_FCL)
    set(USE_FCL ON)
endif()

# Include directories
include_directories(src)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)

if(NOT BUILD_WITH_ROS2)
    include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${urdfdom_INCLUDE_DIRS}
        ${ODE_INCLUDE_DIR}
        ${FCL_INCLUDE_DIRS}
    )
endif()

# Subdirectories
add_subdirectory(src)

# --- 3. Executables ---

# 3.1 Offline Trainer (Main Target)
add_executable(offline_urdf_trainer src/offline_urdf_trainer.cpp)
target_link_libraries(offline_urdf_trainer PRIVATE 
    simulation_lib collision_lib gng_lib spatial_lib status_lib
)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(offline_urdf_trainer 
        rclcpp urdf urdfdom fcl Eigen3 OpenCV cv_bridge ament_index_cpp
    )
    install(TARGETS offline_urdf_trainer DESTINATION lib/${PROJECT_NAME})
else()
    target_link_libraries(offline_urdf_trainer PRIVATE 
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen
        ${urdfdom_LIBRARIES}
    )
    if(USE_FCL)
        target_link_libraries(offline_urdf_trainer PRIVATE ${FCL_LIBRARIES})
    endif()
endif()

# 3.2 Online Integration (Simulator)
if(USE_ODE AND USE_DRAWSTUFF)
    add_executable(online_integration ${IMGUI_SOURCES} src/simulation/online_integration_main.cpp)
    target_link_libraries(online_integration PRIVATE 
        simulation_full_lib 
        simulation_lib 
        hardware_lib 
        visualization_lib 
        control_lib 
        experiment_lib
        gng_lib
        status_lib
        spatial_lib
        collision_lib
        ${ODE_LIBRARY}
        ${DRAWSTUFF_LIBRARY}
    )
    if(NOT BUILD_WITH_ROS2)
        target_link_libraries(online_integration PRIVATE ${OpenCV_LIBRARIES} Eigen3::Eigen)
        if(APPLE)
            target_link_libraries(online_integration PRIVATE "-framework GLUT" "-framework OpenGL")
        else()
            target_link_libraries(online_integration PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
        endif()
    endif()
endif()

if(BUILD_WITH_ROS2)
    ament_package()
endif()
