"""
Robot State Publisher (RSP) Launch File

Questo launch file configura e avvia il Robot State Publisher per il robot Create.
Pubblica la struttura cinematica e i frame di trasformazione basati sulla descrizione URDF del robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Genera la descrizione del launch per il Robot State Publisher.
    """

    # Configurazioni runtime
    use_sim_time = LaunchConfiguration('use_sim_time')          # Usa tempo simulato
    use_ros2_control = LaunchConfiguration('use_ros2_control')  # Includi ros2_control

    # Percorso generico al file XACRO del robot (niente più assoluto)
    xacro_file = PathJoinSubstitution([
        FindPackageShare('create_description'),
        'urdf',
        'create_2.urdf.xacro'
    ])

    # Generazione dinamica dell'URDF tramite xacro
    robot_description_config = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' use_sim_time:=', use_sim_time
    ])

    # Parametri del nodo Robot State Publisher
    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str),
        'use_sim_time': use_sim_time
    }

    # Nodo Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Assemble launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usa tempo simulato se true, tempo reale se false'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Includi ros2_control nel robot description'
        ),
        node_robot_state_publisher
    ])

