from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('path_planner'),
        'config', 'params.yaml'
    )

    planner_node = Node(
        package='path_planner',
        executable='planner_node',
        name='path_planner',
        parameters=[config],
        output='screen'
    )

    mission_node = Node(
        package='path_planner',
        executable='mavros_mission',
        name='mavros_mission',
        output='screen'
    )

    return LaunchDescription([
        planner_node,
        mission_node,
    ])