from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_algo_manager',
            executable='algo_manager_node',
            name='algo_manager_node',
            output='screen'
        ),
        # 注意：算法节点不在 launch 中启动
    ])
