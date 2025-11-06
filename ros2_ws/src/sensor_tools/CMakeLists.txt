cmake_minimum_required(VERSION 3.8)
project(sensor_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


# If the flag is enabled, define the macro, Needed for Realsense
if(USE_LIFECYCLE_NODE)
    find_package(rclcpp_lifecycle REQUIRED)
    find_package(lifecycle_msgs REQUIRED)
    list(APPEND dependencies rclcpp_lifecycle lifecycle_msgs)
    list(APPEND targets 
        rclcpp_lifecycle::rclcpp_lifecycle
        lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp)
    add_definitions(-DUSE_LIFECYCLE_NODE)
    message("🚀 USE_LIFECYCLE_NODE is ENABLED")

    # Create a configuration file for the launch file
    file(WRITE "${CMAKE_BINARY_DIR}/global_settings.yaml"
        "use_lifecycle_node: true\n")
else()
    file(WRITE "${CMAKE_BINARY_DIR}/global_settings.yaml"
        "use_lifecycle_node: false\n")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
