from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('test_pkg'),
        'config',
        'config.rviz'
    )

    return LaunchDescription([
        # su_fkik 노드 실행
        Node(
            package='test_pkg',
            executable='su_fkik',
            output='screen'
        ),

        # su_rviz 노드 실행
        Node(
            package='test_pkg',
            executable='su_rviz',
            output='screen'
        ),

        # rviz2 노드 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        ),
    ])

