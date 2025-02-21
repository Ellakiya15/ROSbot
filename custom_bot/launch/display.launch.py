#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Process XACRO
    xacro_path = os.path.join(get_package_share_directory('custom_bot'), 'urdf', 'robot.xacro') #robot-sim.xacro
    doc = get_xacro_to_doc(xacro_path,{})

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': doc.toxml()}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('custom_bot'), 'rviz', 'rsp.rviz')]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value=doc.toxml(),
            description='Robot description in URDF/XACRO format'
        ),
        # Nodes
        robot_state_publisher,
        rviz
    ])

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     pkg_share = get_package_share_directory('custom_bot')
    
#     # URDF file
#     urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
#     # RViz configuration file
#     rviz_config = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    
#     with open(urdf_file, 'r') as infp:
#         robot_desc = infp.read()

#     # Robot State Publisher
#     # robot_state_publisher = Node(
#     #     package='robot_state_publisher',
#     #     executable='robot_state_publisher',
#     #     name='robot_state_publisher',
#     #     output='screen',
#     #     parameters=[{'robot_description': robot_desc}]
#     # )

#     # Joint State Publisher GUI
#     joint_state_publisher_gui = Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         name='joint_state_publisher_gui'
#     )

#     # RViz
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', rviz_config],
#         output='screen'
#     )

#     return LaunchDescription([
#         # robot_state_publisher,
#         joint_state_publisher_gui,
#         rviz
#     ])