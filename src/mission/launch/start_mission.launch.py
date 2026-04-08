import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Bring back the Launch Argument (Defaults to '1' if not passed in terminal)
    env_id_arg = DeclareLaunchArgument(
        'env_id', 
        default_value='1',
        description='Index of the static environment to load (1-10)'
    )

    # 2. Locate the paths for params.yaml and mission.rviz
    pkg_dir = get_package_share_directory('mission')
    
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'mission.rviz')

    # 3. Define the Fake SLAM Node
    fake_slam_node = Node(
        package='mission',
        executable='fake_slam',
        name='fake_slam',
        parameters=[
            config_file, 
            {'env_id': LaunchConfiguration('env_id')} 
        ]
    )

    # 4. Define the Robot Position Updater Node
    robot_position_updater_node = Node(
        package='mission',
        executable='robot_position_updater',
        name='robot_position_updater',
        parameters=[config_file]
    )

    # 5. Define the RViz2 Node 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    planner_node = Node(
        package='mission',
        executable='planner',
        name='planner',
        output='screen'
    )

    # 7. Define the Controller Node
    controller_node = Node(
        package='mission',
        executable='controller',
        name='controller',
        output='screen'
    )

    return LaunchDescription([
        env_id_arg,
        fake_slam_node,
        robot_position_updater_node,
        planner_node,
        controller_node,
        rviz_node 
    ])