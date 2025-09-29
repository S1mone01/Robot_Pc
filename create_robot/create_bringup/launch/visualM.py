from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Dichiarazione argomenti configurabili da terminale
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usa /clock simulato se true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('create_bringup'),
            'config',
            'nav2_params_mapping.yaml'
        ]),
        description='Percorso al file dei parametri Nav2'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Avvia RViz2 se true'
    )

    # Parametri
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Percorsi
    slam_toolbox_dir = FindPackageShare('slam_toolbox').find('slam_toolbox')
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # SLAM Toolbox (avvio immediato)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Navigation (ritardato di 5 secondi)
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file
                }.items()
            )
        ]
    )

    # RViz2 (partenza ritardata di 1 secondo, solo se use_rviz è true)
    rviz2 = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                condition=IfCondition(use_rviz)
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_use_rviz,
        slam_launch,
        navigation_launch,
        rviz2
    ])

