cmake_minimum_required(VERSION 3.8)
project(test_callback_groups)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DoWork.action"
  DEPENDENCIES std_msgs
)

# Create executable
add_executable(do_work_server src/do_work_server.cpp)

# Link dependencies
ament_target_dependencies(do_work_server
  rclcpp
  rclcpp_action
  std_msgs
  diagnostic_updater
)

# Link generated interfaces
rosidl_target_interfaces(do_work_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  do_work_server
  DESTINATION lib/${PROJECT_NAME}
)

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

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()