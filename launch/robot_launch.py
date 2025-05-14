#!/usr/bin/env python
import os
import launch
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, LogInfo
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController # Используем для TeslaDriver
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tesla')
    world = LaunchConfiguration('world')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time', default='true')
    # run_lane_follower = LaunchConfiguration('run_lane_follower', default='true') # Если хотите контролировать

    # URDF для TeslaDriver (минималистичный с плагином)
    robot_description_path_for_driver = os.path.join(package_dir, 'resource', 'tesla_webots.urdf')
    # URDF для robot_state_publisher (с линками для TF)
    tesla_tf_urdf_path = os.path.join(package_dir, 'resource', 'tesla_tf_description.urdf')

    declare_world_argument = DeclareLaunchArgument(
        'world', default_value='tesla_world.wbt', description='Choose world file'
    )
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )
    # declare_run_lane_follower_argument = DeclareLaunchArgument(
    #     'run_lane_follower', default_value='true', description='Run lane follower'
    # )

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True # Если нужен для чего-то (например, для будущей одометрии)
    )

    # Запускаем кастомный TeslaDriver через WebotsController
    tesla_driver_node = WebotsController(
        robot_name='vehicle', # Имя робота в Webots (DEF или name)
        parameters=[
            {'robot_description': robot_description_path_for_driver}
        ],
        respawn=True
    )

    # Запускаем robot_state_publisher с URDF для TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', tesla_tf_urdf_path]), value_type=str),
            'use_sim_time': use_sim_time_launch_arg
        }]
    )

    # Запускаем lane_follower (если он вам нужен на этом этапе)
    # lane_follower_node = Node(
    #     package='webots_ros2_tesla',
    #     executable='lane_follower',
    #     # condition=IfCondition(run_lane_follower) # Если используете условный запуск
    # )

    return LaunchDescription([
        declare_world_argument,
        declare_use_sim_time_argument,
        # declare_run_lane_follower_argument,
        webots,
        webots._supervisor if hasattr(webots, '_supervisor') else launch.actions.OpaqueFunction(lambda context: []), # Запуск супервайзера, если он есть
        tesla_driver_node, # Запускаем TeslaDriver
        robot_state_publisher_node,
        # lane_follower_node, # Если нужен
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])