import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    localization_params = os.path.join(
        get_package_share_directory('localization'),
        'test_params.yaml'
    )

    simulate_launch_file = os.path.join(
        get_package_share_directory('racecar_simulator'),
        'launch',
        'simulate.launch.xml'
    )

    map_file = os.path.join(
        get_package_share_directory('localization'),
        'test_map',
        'test_map.yaml'
    )

    return LaunchDescription([
        Node(
            package='localization',
            executable='sensor_model_test',
            name='particle_filter',
            output='screen',
            parameters=[localization_params]
        ),
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(simulate_launch_file),
                    launch_arguments={'map': map_file}.items(),
                )
            ]
        )
    ])
