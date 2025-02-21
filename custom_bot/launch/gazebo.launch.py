import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('custom_bot')
    
    # Path to the custom world file
    # world_file_name = 'empty_world.world'
    # world_path = join(pkg_share, 'worlds', world_file_name)

    world_file = LaunchConfiguration("world_file", default = join(pkg_share,"worlds" , "sample.sdf"))  #simple_world.sdf works
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world_file], 
        'on_exit_shutdown': 'true'}.items(),
        #launch_arguments={'world': world_path}.items(),
    )

    # URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_file,
                  '-topic', 'robot_description',
                  '-name', 'diffbot',
                  '-z', '0.1'],
        output='screen'
    )

#     start_gazebo_ros_spawner_cmd = Node(
#     package='ros_gz_sim',
#     executable='create',
#     arguments=[
#         '-file', urdf_path,
#         '-x', x_pose,
#         '-y', y_pose,
#         '-z', '0.01'
#     ],
#     output='screen',
# )
    
    # Bridge
    bridge_params = os.path.join(
    get_package_share_directory('custom_bot'),
    'params',
    'gz_bridge.yaml'
    )

    start_gazebo_ros_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}',
    ],
    output='screen',
)

    return LaunchDescription([
        gazebo,
        start_gazebo_ros_bridge,
        robot_state_publisher,
        spawn_entity,
    ])
