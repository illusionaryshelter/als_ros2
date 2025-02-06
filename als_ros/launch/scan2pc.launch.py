from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    scan2pc = Node(package='als_ros',
                   executable='scan2pc',
                   output='screen',
                   namespace='',
                   parameters=[{
                       'scan_name': '/scan',
                       'scan_points_name': '/scan_points'
                   }]
                   )

    ld = LaunchDescription()
    ld.add_action(scan2pc)

    return ld
