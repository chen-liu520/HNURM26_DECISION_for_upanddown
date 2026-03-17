import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    decision_dir = get_package_share_directory('hnurm_small_decision')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(decision_dir, 'param', 'params.yaml'),
            description='default decision yaml'
        ),
        Node(
            package='hnurm_small_decision',
            executable='decision_node',
            output='screen',
            parameters=[params_file]
        )
        # DeclareLaunchArgument(
        #     'areas_params_file',
        #     default_value=os.path.join(decision_dir, 'params', 'areas.yaml'),
        #     description='special areas'
        # ),
        
    ])
