set(TEST_EXEC test_flight)

add_executable(
    ${TEST_EXEC}
    test/test_fligth.cpp
)

ament_target_dependencies(
    ${TEST_EXEC}
    rclcpp
    rclcpp_action
    nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${TEST_EXEC} ${cpp_typesupport_target_test} Eigen3::Eigen)

install(
    TARGETS ${TEST_EXEC}
    DESTINATION lib/${PROJECT_NAME}
)
