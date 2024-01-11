
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom", --mitooni ino base_link bezari vali bayad badesj provide_odom_frame ro true kuni --> alabte na lozoman--> ye rah dg ine ke controlleret topic /odom ba frame odom mide vali az tf esh az odom->base_link use nakuni va inja benvisi base_link
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 2e-2, --5e-3 this is also either should be the hz of your /odom topic which is 50 (0.02 seconds)usually or /scan which is often 10hz but i don't know which --> this number actually defines the hz between map->odom or map->base_link tfs (depending on what you wrote on published_frame) --> but i guess my system is slow and it alwasy give me soemthing along the lines of 10,11 hz
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 7
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 5.0 
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)


TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e3
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e-2
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1e1

-- POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.5 -- default is 0.6 on the cartographer POSE_GRAPH config_file 


-- TRAJECTORY_BUILDER.pure_localization = true


-- output map to base_link for evaluation
-- options.provide_odom_frame = false
 POSE_GRAPH.optimization_problem.log_solver_summary = true

-- fast localization
-- MAP_BUILDER.num_background_threads = 12
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.5 * POSE_GRAPH.constraint_builder.sampling_ratio
-- POSE_GRAPH.global_sampling_ratio = 0.1 * POSE_GRAPH.global_sampling_ratio
-- POSE_GRAPH.max_num_final_iterations = 1



TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.global_constraint_search_after_n_seconds = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 6
MAP_BUILDER.num_background_threads = 8
POSE_GRAPH.global_sampling_ratio = 0.003


-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1.0
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1.0
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.0
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
-- https://answers.ros.org/question/347334/settings-for-pure-localization-for-cartographer_ros/
-- https://github.com/cartographer-project/cartographer_ros/issues/1582
return options
