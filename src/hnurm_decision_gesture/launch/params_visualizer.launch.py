import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    decision_dir = get_package_share_directory('hnurm_decision_gesture')
    params_file = os.path.join(decision_dir, 'param', 'params.yaml')
    
    return LaunchDescription([
        Node(
            package='hnurm_decision_gesture',
            executable='params_visualizer',
            name='params_visualizer',
            output='screen',
            parameters=[params_file]  # 加载参数文件
        )
    ])
