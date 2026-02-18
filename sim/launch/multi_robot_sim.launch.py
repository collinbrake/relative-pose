#!/usr/bin/env python3

import os
import re
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _expand_sdf_includes(template_path):
    with open(template_path, 'r', encoding='utf-8') as template_file:
        content = template_file.read()

    include_pattern = re.compile(r'<include\s+filename="([^"]+)"\s*/>')

    def replace_include(match):
        include_filename = match.group(1)
        include_path = os.path.join(os.path.dirname(template_path), include_filename)
        with open(include_path, 'r', encoding='utf-8') as include_file:
            return include_file.read()

    expanded = include_pattern.sub(replace_include, content)

    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
    temp_file.write(expanded)
    temp_file.flush()
    temp_file.close()
    return temp_file.name


def generate_launch_description():
    # Get package directories
    pkg_relative_pose_sim = get_package_share_directory('relative_pose_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Paths
    world_file = os.path.join(pkg_relative_pose_sim, 'worlds', 'flat_plane.world')
    leader_template = os.path.join(pkg_relative_pose_sim, 'models', 'leader_robot.sdf')
    follower_template = os.path.join(pkg_relative_pose_sim, 'models', 'follower_robot.sdf')
    leader_sdf = _expand_sdf_includes(leader_template)
    follower_sdf = _expand_sdf_includes(follower_template)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_controllers = LaunchConfiguration('enable_controllers', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_enable_controllers = DeclareLaunchArgument(
        'enable_controllers',
        default_value='true',
        description='Enable robot controllers'
    )
    
    # Start Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )
    
    # Start Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Spawn Leader Robot
    spawn_leader = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='leader',
        arguments=[
            '-entity', 'leader',
            '-file', leader_sdf,
            '-robot_namespace', 'leader',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0',
            '-timeout', '120'
        ],
        output='screen'
    )

    # Spawn Follower Robot after leader spawn exits
    spawn_follower = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='follower',
        arguments=[
            '-entity', 'follower',
            '-file', follower_sdf,
            '-robot_namespace', 'follower',
            '-x', '-5.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0',
            '-timeout', '120'
        ],
        output='screen'
    )
    
    # Leader Controller
    leader_controller = Node(
        package='relative_pose_sim',
        executable='leader_controller.py',
        namespace='leader',
        name='leader_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(enable_controllers)
    )
    
    # Follower Controller
    follower_controller = Node(
        package='relative_pose_sim',
        executable='follower_controller.py',
        namespace='follower',
        name='follower_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(enable_controllers)
    )
    
    # Data Recorder
    data_recorder = Node(
        package='relative_pose_sim',
        executable='data_recorder.py',
        name='data_recorder',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    spawn_follower_after_leader = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_leader,
            on_exit=[spawn_follower]
        )
    )

    start_nodes_after_follower = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_follower,
            on_exit=[leader_controller, follower_controller, data_recorder]
        )
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_controllers,
        gzserver,
        gzclient,
        spawn_leader,
        spawn_follower_after_leader,
        start_nodes_after_follower
    ])
