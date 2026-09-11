import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

import xacro

def generate_launch_description():
    pkg_name = 'robots'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_share, 'robot_designs', 'turtlebot.urdf.xacro')
    # robot_1 için
    doc1 = xacro.process_file(xacro_file, mappings={'prefix': 'robot_1_'})
    robot_desc_1 = doc1.toxml()
    # robot_2 için
    doc2 = xacro.process_file(xacro_file, mappings={'prefix': 'robot_2_'})
    robot_desc_2 = doc2.toxml()

    robot_1_group = GroupAction([
        PushRosNamespace('robot_1'),
        
        # robot_1'e ait State Publisher (TF ve robot_description yayınlar)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc_1, 'use_sim_time': True}]
        ),
        
        # Gazebo'ya spawn etme (X = -1.5 koordinatına)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'robot_1',
                '-robot_namespace', 'robot_1',
                '-x', '-1.5', '-y', '0.0', '-z', '0.1'
            ],
            output='screen'
        )
    ])

    robot_2_group = GroupAction([
    PushRosNamespace('robot_2'),
    
    # robot_2'nin State Publisher'ı
    Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_2, 'use_sim_time': True}]
    ),
    
    # Gazebo'ya spawn (X = 0, Y = 2.0 koordinatına)
    Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_2',
            '-robot_namespace', 'robot_2',
            '-x', '0.0', '-y', '2.0', '-z', '0.1'
        ],
        output='screen'
    )
    ])

    # Gazebo Dünyasını Başlatıcı (test_obstacles.world)
    world_file = os.path.join(pkg_share, 'worlds', 'test_obstacles.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([
        gazebo,
        robot_1_group,
        robot_2_group
    ])