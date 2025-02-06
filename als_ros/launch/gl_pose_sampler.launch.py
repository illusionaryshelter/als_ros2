from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mae_classifier_learning = Node(package='als_ros',
                                   executable='gl_pose_sampler',
                                   output='screen',
                                   namespace='',
                                   parameters=[{
                                       'map_name': '/map',
                                       'scan_name': '/scan',
                                       'odom_name': '/odom',

                                       'poses_name': '/gl_sampled_poses',
                                       'local_map_name': '/gl_local_map',
                                       'sdf_keypoints_name': '/gl_sdf_keypoints',
                                       'local_sdf_keypoints_name': '/gl_local_sdf_keypoints',

                                       'map_frame': 'map',
                                       'odom_frame': 'odom',
                                       'base_link_frame': 'base_link',
                                       'laser_frame': 'laser',

                                       'key_sacns_num': 3,
                                       'key_scan_interval_dist': 0.2,
                                       'key_scan_interval_yaw': 5.0,
                                       'gradient_square_th': 0.01,
                                       'keypoints_min_dist_from_map': 0.5,
                                       'sdf_feature_window_size': 1.0,
                                       'average_sdf_delta_th': 0.01,
                                       'add_random_samples': True,
                                       'add_opposite_samples': True,
                                       'random_samples_num': 20,
                                       'positional_random_noise': 0.5,
                                       'angular_random_noise': 0.3,
                                       'matching_rate_th': 0.03
                                   }]
                                   )

    ld = LaunchDescription()
    ld.add_action(mae_classifier_learning)

    return ld
