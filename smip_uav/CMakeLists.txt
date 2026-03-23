cmake_minimum_required(VERSION 3.8)
project(smip_uav)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SMIP_EXEC smip_uav_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(
    DEPS
    rclcpp
    tf2_ros
    tf2_eigen
)

include_directories(
    include/smip_uav
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
    ${SMIP_EXEC}
    src/smip_exec.cpp
    src/sensor_data_preprocess.cpp
    src/surfel_map.cpp
)

ament_target_dependencies(
  ${SMIP_EXEC}
  ${DEPS}
)

target_link_libraries(
    ${SMIP_EXEC}
    Eigen3::eigen
)

install(
  TARGETS ${SMIP_EXEC}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

