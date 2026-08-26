"""Launch the solar farm with the inspection drone and both operator views.

    ros2 launch solar_farm_gz inspection.launch.py
    ros2 launch solar_farm_gz inspection.launch.py world:=solar_farm_1000
    ros2 launch solar_farm_gz inspection.launch.py drone_x:=-6.0 drone_y:=-6.0

This brings up the world, spawns the drone, opens the free-orbit view and the
nadir camera panel together, and bridges the camera into ROS 2. It does not
start the flight controller: ArduPilot SITL runs as its own process, so that it
can be restarted mid-session without tearing down the world.

    cd ~/ardupilot
    Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
        --model JSON --console --map

SITL reaches the aircraft over UDP 9002, matching <fdm_port_in> in
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

    # Topic names come from the sensor definitions in models/x500_rgb/model.sdf.
    # camera_info is published alongside the image rather than nested under it,
    # which is why the two are bridged separately below.
    image_topic = '/x500_rgb/nadir'
    caminfo_topic = '/x500_rgb/camera_info'

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='solar_farm',
                              description='world file stem inside worlds/'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='run the server with no GUI'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='bridge camera and clock to ROS 2'),

        # Spawn clear of the first row so the aircraft is not inside a table on
        # startup; the GUI's initial camera pose is aimed at this corner.
        DeclareLaunchArgument('drone_x', default_value='-6.0'),
        DeclareLaunchArgument('drone_y', default_value='-6.0'),
        DeclareLaunchArgument('drone_z', default_value='0.13',
                              description='leg feet sit 0.13 m below the hub'),
        DeclareLaunchArgument('drone_yaw', default_value='0.0'),

        # ArduPilotPlugin is not a ROS package, so its build directory has to be
        # named explicitly. The default matches the install location documented
        # upstream; override it if the repository lives elsewhere.
        DeclareLaunchArgument(
            'ardupilot_gazebo',
            default_value=os.path.join(os.path.expanduser('~'),
                                       'ardupilot_gazebo'),
            description='path to the ardupilot_gazebo checkout'),

        # Relative mesh and texture URIs in the SDF resolve against these.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               f'{worlds}:{models}'),

        # Push rendering onto the discrete GPU where there is one. Without
        # this a switchable-graphics laptop silently renders the whole farm on
        # integrated graphics. No-op on machines without an NVIDIA card.
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
                    # Both operator views come from this config: the 3D scene
                    # and the docked nadir camera panel.
                    '--gui-config ', gui_config, ' ',
                    PythonExpression(
                        ["'-s' if '", headless, "'.lower() == 'true' else ''"]),
                ],
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # The drone is spawned rather than baked into the world, so that
        # generate_farm.py stays a Phase 1 concern and the world file remains
        # exactly what the generator produced.
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

        # image_bridge rather than parameter_bridge for the image itself: it
        # handles the transport properly for 1920x1080 at 30 fps.
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
            # No IMU here on purpose. ArduPilotPlugin derives the IMU topic
            # from the sensor's scoped name, so the sensor must not declare a
            # custom <topic>; bridging it would mean hard-coding that long
            # generated path, and the flight controller already owns the IMU.
            # The camera is what the detection pipeline consumes.
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                f'{caminfo_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
        ),
    ])
