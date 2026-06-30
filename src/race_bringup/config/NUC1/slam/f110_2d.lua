include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",  --Change to "odom" for REP105 compliance (but worse performance)
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_nav_sat = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  publish_to_tf = true,
  use_landmarks = false,
  publish_tracked_pose = true,
  publish_frame_projected_to_2d = true,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Lidar scan filtering
TRAJECTORY_BUILDER_2D.max_range = 12.0 -- Restrict max range to ignore distant reflections and speed up matching
TRAJECTORY_BUILDER_2D.min_range = 0.1  -- Ignore reflections from the car's own chassis

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100

-- Occupancy grid tuning (makes walls stickier and harder for noise to clear out)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.60 -- Reduced from 0.75 to make walls thinner
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45 -- Back to original (0.45) to aggressively clean up noise

-- Stop erasing existing walls when the LiDAR shoots over short barriers
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.0

POSE_GRAPH.optimize_every_n_nodes = 20

-- Reduce constraint search density to lighten the pose graph on Jetson
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

-- Trust the scan matcher more (scale factor 1.5 instead of 0.2) to correct for wheel slip
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.5 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.5 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight

-- Use odometry in pose graph optimization to prevent drift/smearing
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5

return options
