"""Lanza el parque solar con el dron de inspección y ambas vistas de operador.

    ros2 launch solar_farm_gz inspection.launch.py
    ros2 launch solar_farm_gz inspection.launch.py world:=solar_farm_1000
    ros2 launch solar_farm_gz inspection.launch.py drone_x:=-6.0 drone_y:=-6.0

Esto levanta el mundo, genera (spawn) el dron, abre juntas la vista de
órbita libre y el panel de la cámara en nadir, y conecta la cámara a ROS 2.
No arranca el controlador de vuelo: ArduPilot SITL se ejecuta como su
propio proceso, para que se pueda reiniciar a mitad de sesión sin cerrar
el mundo.

    cd ~/ardupilot
    Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
        --model JSON --console --map

SITL llega a la aeronave por UDP 9002, coincidiendo con <fdm_port_in> en
models/x500_rgb/model.sdf.
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
    models = os.path.join(pkg, 'models')
    gui_config = os.path.join(pkg, 'gui', 'inspection.config')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    bridge = LaunchConfiguration('bridge')
    ap_gazebo = LaunchConfiguration('ardupilot_gazebo')

    # Los nombres de topic vienen de las definiciones de sensor en
    # models/x500_rgb/model.sdf. camera_info se publica junto a la imagen
    # en lugar de anidado bajo ella, por eso ambos se conectan por
    # separado más abajo.
    image_topic = '/x500_rgb/nadir'
    caminfo_topic = '/x500_rgb/camera_info'

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='solar_farm',
                              description='world file stem inside worlds/'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='run the server with no GUI'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='bridge camera and clock to ROS 2'),

        # Genera (spawn) despejado de la primera fila para que la
        # aeronave no quede dentro de una mesa al arrancar; la pose
        # inicial de cámara de la interfaz gráfica apunta a esta esquina.
        DeclareLaunchArgument('drone_x', default_value='-6.0'),
        DeclareLaunchArgument('drone_y', default_value='-6.0'),
        DeclareLaunchArgument('drone_z', default_value='0.13',
                              description='leg feet sit 0.13 m below the hub'),
        DeclareLaunchArgument('drone_yaw', default_value='0.0'),

        # ArduPilotPlugin no es un paquete de ROS, así que su directorio
        # de compilación tiene que nombrarse explícitamente. El valor por
        # defecto coincide con la ubicación de instalación documentada
        # aguas arriba; sobrescríbelo si el repositorio vive en otro sitio.
        DeclareLaunchArgument(
            'ardupilot_gazebo',
            default_value=os.path.join(os.path.expanduser('~'),
                                       'ardupilot_gazebo'),
            description='path to the ardupilot_gazebo checkout'),

        # Los URIs relativos de malla y textura en el SDF se resuelven contra estos.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               f'{worlds}:{models}'),

        # Empuja el renderizado hacia la GPU discreta donde hay una. Sin
        # esto, un portátil con gráficos conmutables renderiza
        # silenciosamente todo el parque en gráficos integrados. No hace
        # nada en máquinas sin tarjeta NVIDIA.
        *[SetEnvironmentVariable(k, v)
          for k, v in gpu.offload_env().items()],
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            PathJoinSubstitution([ap_gazebo, 'build'])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [
                    PathJoinSubstitution([worlds, world]), '.sdf',
                    ' -r -v 2 ',
                    # Ambas vistas de operador vienen de esta config: la
                    # escena 3D y el panel acoplado de la cámara en nadir.
                    '--gui-config ', gui_config, ' ',
                    PythonExpression(
                        ["'-s' if '", headless, "'.lower() == 'true' else ''"]),
                ],
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # El dron se genera (spawn) en lugar de incrustarse en el mundo,
        # para que generate_farm.py siga siendo un asunto exclusivo de la
        # Fase 1 y el fichero de mundo se mantenga exactamente igual a lo
        # que produjo el generador.
        Node(
            package='ros_gz_sim', executable='create', name='spawn_drone',
            output='screen',
            arguments=[
                '-world', world,
                '-file', os.path.join(models, 'x500_rgb', 'model.sdf'),
                '-name', 'x500_rgb',
                '-x', LaunchConfiguration('drone_x'),
                '-y', LaunchConfiguration('drone_y'),
                '-z', LaunchConfiguration('drone_z'),
                '-Y', LaunchConfiguration('drone_yaw'),
            ],
        ),

        # image_bridge en lugar de parameter_bridge para la imagen en sí:
        # gestiona correctamente el transporte para 1920x1080 a 30 fps.
        Node(
            package='ros_gz_image', executable='image_bridge',
            name='nadir_image_bridge', output='screen',
            condition=IfCondition(bridge),
            arguments=[image_topic],
        ),

        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='inspection_bridge', output='screen',
            condition=IfCondition(bridge),
            # Sin IMU aquí, a propósito. ArduPilotPlugin deriva el topic
            # del IMU a partir del nombre con ámbito (scoped name) del
            # sensor, así que el sensor no debe declarar un <topic>
            # personalizado; conectarlo significaría fijar en el código
            # esa ruta larga generada, y el controlador de vuelo ya es
            # dueño del IMU. La cámara es lo que consume el pipeline de
            # detección.
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                f'{caminfo_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
        ),
    ])
