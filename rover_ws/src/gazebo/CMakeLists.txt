cmake_minimum_required(VERSION 3.8)
project(gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

# Install directories
install(
  DIRECTORY 
    launch
    model
    config    # If you have config files
    rviz     # If you have rviz config files
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts if you have any
install(
  PROGRAMS
    launch/map_to_odom.launch.py  # Add your Python scripts here
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
