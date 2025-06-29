from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Paths
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    custom_dwa_pkg = get_package_share_directory('custom_dwa_planner')

    # Launch file path
    gazebo_launch_file = os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
    rviz_config_file = os.path.join(custom_dwa_pkg, 'rviz', 'default.rviz')

    return LaunchDescription([
        # Gazebo simulation (world with TurtleBot3)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
        ),

        #RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # Custom DWA planner node
        Node(
            package='custom_dwa_planner',
            executable='dwa_planner_node',
            name='dwa_planner_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])
