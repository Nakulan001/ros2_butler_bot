from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('b_bot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'b_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    world_file = os.path.join(share_dir, 'worlds', 'cafe_world.sdf')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default= True)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time':use_sim_time},
        ]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py',
            ])
        ]),
        launch_arguments={
            'pause': 'false',
            'world':world_file
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'b_bot1',
            '-topic', 'robot_description',
            '-x','2.499',
            '-y','-0.008',
            '-z','0.0387',
            '-Y','-3.130'       
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])
