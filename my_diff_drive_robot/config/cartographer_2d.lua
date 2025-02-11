-- Minimal 2D cartographer configuration file for a single laser

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "lidar_link",  -- frame Cartographer uses as reference for SLAM
  published_frame = "lidar_link", -- frame that gets published by Cartographer
  odom_frame = "odom",           -- the name of the odometry frame (even if we don't have real odom, we keep it here)
  provide_odom_frame = true,     -- Cartographer will publish its own odometry
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,           -- number of separate LaserScan topics
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
}

-- 2D map builder settings
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D = TRAJECTORY_BUILDER.trajectory_builder_2d

-- Laser scan parameters
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 15.0
TRAJECTORY_BUILDER_2D.missing_data_raycasting = true
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- false if you don't have IMU
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

return options
