"""Lanza el mundo del parque solar en Gazebo Harmonic.

    ros2 launch solar_farm_gz solar_farm.launch.py
    ros2 launch solar_farm_gz solar_farm.launch.py headless:=true
    ros2 launch solar_farm_gz solar_farm.launch.py world:=solar_farm_1000
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node

from solar_farm_gz import gpu


def generate_launch_description():
    pkg = get_package_share_directory('solar_farm_gz')
    worlds = os.path.join(pkg, 'worlds')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    bridge = LaunchConfiguration('bridge')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='solar_farm',
                              description='world file stem inside worlds/'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='run the server with no GUI'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='start the ros_gz clock bridge'),

        # los URIs relativos de malla y textura en el SDF se resuelven contra esto
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', worlds),

        # Renderiza en la GPU discreta cuando la máquina tiene una; ver
        # solar_farm_gz.gpu para saber por qué esto no es incondicional.
        *[SetEnvironmentVariable(k, v)
          for k, v in gpu.offload_env().items()],

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [
                    PathJoinSubstitution([worlds, world]), '.sdf',
                    ' -r -v 2 ',
                    # -s es solo servidor; la interfaz gráfica es la mitad
                    # cara. Esto tiene que ser una substitution: `headless`
                    # es una LaunchConfiguration, así que compararla con
                    # una cadena en Python siempre daría False e ignoraría
                    # el argumento silenciosamente.
                    PythonExpression(
                        ["'-s' if '", headless, "'.lower() == 'true' else ''"]),
                ],
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='clock_bridge', output='screen',
            condition=IfCondition(bridge),
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
    ])
