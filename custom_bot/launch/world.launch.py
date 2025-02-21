#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    custom_bot_path = get_package_share_directory("custom_bot")
    world_file = LaunchConfiguration("world_file", default=join(custom_bot_path, "worlds", "warehouse_v1.sdf"))
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    # Launch Ignition Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items()
    )

    # Spawn the robot
    # spawn_robot_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(join(custom_bot_path, "launch", "gazebo.launch.py")),
    #     launch_arguments={
    #         # Pass any required arguments for spawning
    #     }.items()
    # )

    return LaunchDescription([
        # Set the resource path for worlds and models
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(custom_bot_path, "worlds")
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(custom_bot_path, "models")
        ),

        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),

        gz_sim,
        # spawn_robot_node
    ])
