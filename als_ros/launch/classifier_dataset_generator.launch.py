from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mae_classifier_learning = Node(package='als_ros',
                                   executable='classifier_dataset_generator',
                                   output='screen',
                                   namespace='',
                                   parameters=[{
                                       'map_name': '/map',

                                       'generate_sample_num': 2000,

                                       'save_dir': '/tmp/classifier_dataset_test/',

                                       'obstacles_num': 20,

                                       'angle_min': -135.0,
                                       'angle_max': 135.0,
                                       'angle_increment': 0.25,
                                       'range_min': 0.02,
                                       'range_max': 30.0,

                                       'scan_angle_noise': 0.001,
                                       'scan_range_noise': 0.02,

                                       'valid_scan_rate_th': 0.5,

                                       'failure_positional_error_th': 0.2,
                                       'failure_angular_error_th': 2.0,
                                       'positional_error_max': 0.5,
                                       'angular_error_max': 5.0
                                   }]
                                   )


    ld = LaunchDescription()
    ld.add_action(mae_classifier_learning)

    return ld
