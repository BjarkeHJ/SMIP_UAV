cmake_minimum_required(VERSION 3.8)
project(smip_uav)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("Building for arm64...")
    add_compile_options(-march=armv8.2-a+simd+crypto+fp16+rcpc+dotprod -mtune=cortex-a77)
    add_compile_options(-flto -ffast-math -funroll-loops)
    add_link_options(-flto)
else()
    message("Building for x86_64...")
    add_compile_options(-march=native)
    add_compile_options(-flto -ffast-math -funroll-loops)
    add_link_options(-flto)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SMIP_EXEC smip_uav_node)
set(TRAJ_EXEC trajectory_node)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
    include/smip_uav
    ${EIGEN3_INCLUDE_DIRS}
)

# Build Minimal PX4 Messages 


# ACTION: FlightPath
rosidl_generate_interfaces(${PROJECT_NAME}
    "action/FlightPath.action"
    DEPENDENCIES nav_msgs geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# SurfelMap + Planner
set(
    DEPS
    rclcpp
    tf2_ros
    tf2_eigen
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
)

add_executable(
    ${SMIP_EXEC}
    src/smip_exec.cpp
    src/ros_nodes/map_node.cpp
    src/ros_nodes/voxl_tf_node.cpp
    src/surfel_map/frame_builder.cpp
    src/surfel_map/frame_processor.cpp
    src/surfel_map/voxel_grid.cpp
    src/surfel_map/surfel_map.cpp
)
ament_target_dependencies(
  ${SMIP_EXEC}
  ${DEPS}
)
target_link_libraries(
    ${SMIP_EXEC}
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
)

# TrajectoryNode: Action Server for waypoint trajectory interpolation
add_executable(
    ${TRAJ_EXEC}
    src/ros_nodes/trajectory_node.cpp
)
ament_target_dependencies(
    ${TRAJ_EXEC}
    rclcpp
    rclcpp_action
    geometry_msgs
    nav_msgs
)
target_link_libraries(
    ${TRAJ_EXEC}
    Eigen3::Eigen
)

# ACTION: Link generated action interfaces to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${TRAJ_EXEC} ${cpp_typesupport_target})

install(
  TARGETS ${SMIP_EXEC} ${TRAJ_EXEC}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# OPTIONAL: Build test/
option(BUILD_TESTS "Build test executables" ON)
if(BUILD_TESTS)
    include(test/CMakeLists.txt)
endif()

ament_package()

