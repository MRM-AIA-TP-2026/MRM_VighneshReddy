include "/opt/ros/humble/share/cartographer_ros/configuration_files/backpack_2d.lua"


MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_background_threads = 4
TRAJECTORY_BUILDER_2D.submaps_trajectory_builder = false

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.linear_programming_solver_options.use_ipopt = true

-- Sensor settings
TRAJECTORY_BUILDER_2D.laser_scan_topic = '/scan'  -- Modify with your LiDAR topic
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.horizontal_laser_min_angle = -3.14
TRAJECTORY_BUILDER_2D.horizontal_laser_max_angle = 3.14

