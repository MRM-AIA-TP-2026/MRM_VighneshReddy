cmake_minimum_required(VERSION 3.8)
project(cashier_system)

# Add compiler options for GNU or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces from the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bill.msg"
  DEPENDENCIES std_msgs
)

# Include directories for generated message files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Add executables
add_executable(bill_publisher src/bill_publisher.cpp)
ament_target_dependencies(bill_publisher rclcpp std_msgs cashier_system__rosidl_generator_cpp)

add_executable(inventory_subscriber src/inventory_subscriber.cpp)
ament_target_dependencies(inventory_subscriber rclcpp std_msgs cashier_system__rosidl_generator_cpp)

add_executable(status_printer src/status_printer.cpp)
ament_target_dependencies(status_printer rclcpp std_msgs cashier_system__rosidl_generator_cpp)

# Run linter if BUILD_TESTING is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

