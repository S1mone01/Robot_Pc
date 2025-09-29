from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Dichiarazione argomenti
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='map_palermo.yaml',
        description='Nome del file della mappa nella cartella "map" del package create_bringup'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usa /clock simulato se true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Avvia RViz2 se true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('create_bringup'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Percorso al file dei parametri Nav2'
    )

    # Configurazioni
    map_name = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')

    # Percorso completo della mappa
    map_yaml_file = PathJoinSubstitution([
        FindPackageShare('create_bringup'),
        'map',
        map_name
    ])

    # Percorsi Nav2
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # Navigation (ritardata 10s)
    navigation_launch = TimerAction(
        period=10.0,
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

    # RViz2 (ritardato 1s)
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
        declare_map,
        declare_use_sim_time,
        declare_use_rviz,
        declare_params_file,
        localization_launch,
        navigation_launch,
        rviz2,
    ])

