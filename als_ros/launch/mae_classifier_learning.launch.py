from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import yaml


def generate_launch_description():

    classifier_dir = get_package_share_directory(
        "als_ros")+"/classifiers/MAE/"
    
    mae_classifier_learning = Node(package='als_ros',
                    executable='mae_classifier_learning',
                    output='screen',
                    namespace='',
                    parameters=[{
                        'max_residual_error': 1.0,
                        'histogram_bin_width': 0.025,
                        'classifier_dir': classifier_dir
                    }]
                    )

    ld = LaunchDescription()
    ld.add_action(mae_classifier_learning)

    return ld
