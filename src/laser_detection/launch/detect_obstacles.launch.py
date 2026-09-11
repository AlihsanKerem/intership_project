from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    # Robot 1 için Engel Algılama Düğümü
    robot_1_detector = GroupAction([
        PushRosNamespace('robot_1'),
        Node(
            package='laser_detection',
            executable='obstacle_extractor_node',
            name='obstacle_extractor_node',
            output='screen',
            parameters=[{'target_frame': 'world', 'use_sim_time': True}]
        )
    ])

    # Robot 2 için Engel Algılama Düğümü
    robot_2_detector = GroupAction([
        PushRosNamespace('robot_2'),
        Node(
            package='laser_detection',
            executable='obstacle_extractor_node',
            name='obstacle_extractor_node',
            output='screen',
            parameters=[{'target_frame': 'world', 'use_sim_time': True}]
        )
    ])

    return LaunchDescription([
        robot_1_detector,
        robot_2_detector
    ])
