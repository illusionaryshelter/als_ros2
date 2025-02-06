from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mrf_failure_detector = Node(package='als_ros',
                                executable='mrf_failure_detector',
                                output='screen',
                                namespace='',
                                parameters=[{
                                    'residual_errors_name': '/residual_errors',
                                    'failure_probability_name': '/failure_probability',
                                    'aligned_scan_mrf': '/aligned_scan_mrf',
                                    'unknown_scan_mrf': '/unknown_scan_mrf',
                                    'failure_probability_marker_name': '/failure_probability_marker_name',
                                    'marker_frame': 'base_link',

                                    'publish_failure_probability_marker': True,
                                    'publish_classified_scans': True,
                                    'normal_distribution_mean': 0.0,
                                    'normal_distribution_var': 0.01,
                                    'exponential_distribution_lambda': 2.0,
                                    'max_residual_error': 1.0,
                                    'residual_error_resolution': 0.05,
                                    'min_valid_residual_errors_num': 10,
                                    'max_residual_errors_num': 500,
                                    'max_lpb_computation_num': 1000,
                                    'sampling_num': 1000,
                                    'misalignment_ratio_threshold': 0.1,
                                    'unknown_ratio_threshold': 0.7,
                                    'failure_detection_hz': 5.0,
                                    'transition_probability_matrix': [0.8, 0.0, 0.2, 0.0, 0.8, 0.2, 0.333333, 0.333333, 0.333333]
                                }])

    ld = LaunchDescription()
    ld.add_action(mrf_failure_detector)

    return ld
