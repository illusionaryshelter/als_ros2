from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():

    classifier_dir = get_package_share_directory(
        "als_ros")+"/classifiers/MAE/"

    mcl = Node(package='als_ros',
               executable='mcl',
               output='screen',
               namespace='',
               parameters=[{
                   'scan_name': '/scan',
                   'odom_name': '/odom',
                   'map_name': '/map',

                   'pose_name': '/mcl_pose',
                   'particles_name': '/mcl_particles',
                   'unknown_scan_name': '/unknown_scan',
                   'residual_errors_name': '/residual_errors',
                   'reliability_marker_name': '/reliability_marker_name',

                   'gl_sampled_poses_name': '/gl_sampled_poses',
                   'local_map_name': '/gl_local_map',
                   'sdf_keypoints_name': '/gl_sdf_keypoints',
                   'local_sdf_keypoints_name': '/gl_local_sdf_keypoints',

                   'laser_frame': 'laser',
                   'base_link_frame': 'base_link',
                   'map_frame': 'map',
                   'odom_frame': 'odom',

                   'broadcast_tf': True,
                   'use_odom_tf': True,
                   'initial_pose_x': 0.0,
                   'initial_pose_y': 0.0,
                   'initial_pose_yaw': 0.0,
                   'initial_noise_x': 0.02,
                   'initial_noise_y': 0.02,
                   'initial_noise_yaw': 0.01,
                   'measurement_model_type': 2,
                   'particle_num': 1000,
                   'use_augmented_mcl': False,
                   'alpha_slow': 0.001,
                   'alpha_fast': 0.99,
                   'add_random_particles_in_resampling': True,
                   'random_particles_rate': 0.1,
                   'use_omni_directional_model': False,
                   'reject_unknown_scan': False,
                   'publish_unknown_scan': True,
                   'unknown_scan_prob_threshold': 0.9,
                   'publish_residual_errors': True,
                   'scan_step': 5,
                   'z_hit': 0.9,
                   'z_short': 0.2,
                   'z_max': 0.05,
                   'z_rand': 0.05,
                   'var_hit': 0.08,
                   'lambda_short': 1.0,
                   'lambda_unknown': 0.01,
                   'known_class_prior': 0.5,
                   'resample_threshold_ess': 0.5,
                   'estimate_reliability': False,
                   'classifier_type': 0,
                   'mae_classifier_dir': classifier_dir,
                   'mae_max_residual_error': 0.5,
                   'mae_histogram_bin_width': 0.05,
                   'use_gl_pose_sampler': False,
                   'fuse_gl_pose_sampler_only_unreliable': False,
                   'gl_sampled_pose_time_th': 1.0,
                   'gmm_positional_variance': 0.01,
                   'gmm_angular_variance': 0.01,
                   'pred_dist_unif_rate': 0.05,
                   'use_mrf_failure_detector': False,
                   'write_pose': True,
                   'pose_log_file': '/tmp/als_ros_pose.txt',
                   'localization_hz': 10.0,
                   'transform_tolerance': 0.0,

                   'random_particles_noise': [0.05, 0.05, 0.1],
                   'odom_noise_ddm': [1.0, 0.5, 0.5, 1.5],
                   'odom_noise_odm': [4.0, 1.0, 1.0, 1.0, 4.0, 1.0, 1.0, 1.0, 8.0],
                   'rel_trans_ddm': [0.0, 0.0],
                   'rel_trans_odm': [0.0, 0.0, 0.0],
                   'resample_thresholds': [0.2, 0.2, 0.2, 0.02, -99999.0]
               }])

    ld = LaunchDescription()
    ld.add_action(mcl)

    return ld
