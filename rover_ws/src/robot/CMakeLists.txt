cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io segmentation kdtree sample_consensus search)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(cartographer_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rtabmap_ros REQUIRED)
find_package(cartographer_ros REQUIRED)


include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# navigation_node
add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node 
  rclcpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs std_msgs
)

# pointcloud_preprocessor_node
add_executable(pointcloud_preprocessor_node src/pointcloud_preprocessor_node.cpp)
ament_target_dependencies(pointcloud_preprocessor_node 
  rclcpp sensor_msgs pcl_conversions pcl_ros std_msgs
)
target_link_libraries(pointcloud_preprocessor_node ${PCL_LIBRARIES})

# obstacle_avoider_node (pcloa)
add_executable(pcloa src/pcloa.cpp)
ament_target_dependencies(pcloa
  rclcpp sensor_msgs geometry_msgs std_msgs pcl_conversions pcl_ros
)
target_link_libraries(pcloa ${PCL_LIBRARIES})

# compass_node
add_executable(compass_node src/compass.cpp)
ament_target_dependencies(compass_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge tf2 tf2_geometry_msgs
)
target_link_libraries(compass_node ${OpenCV_LIBRARIES})

# Install Python scripts
install(
  PROGRAMS
    src/imu_receiver.py 
    src/gui.py
    src/imu_calibrate.py
    src/relay_node.py # Path to your Python script
  DESTINATION lib/${PROJECT_NAME}  # Install location for executables
)

# Install binaries
install(TARGETS
  navigation_node
  pointcloud_preprocessor_node
  pcloa
  compass_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other project files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

