import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_webots_ros2_tesla = get_package_share_directory('webots_ros2_tesla')
    tesla_simulation_launch_file = os.path.join(
        pkg_webots_ros2_tesla, 'launch', 'robot_launch.py'
    )

    robot_driver_node_name = 'vehicle'
    lidar_frame_id = 'lidar_base'
    base_frame_id = 'base_link' # Фрейм, в который будем преобразовывать скан
    actual_pointcloud_topic = f'/{robot_driver_node_name}/{lidar_frame_id}/point_cloud'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config', default=os.path.join(
        pkg_webots_ros2_tesla, 'rviz', 'tesla_debug.rviz' # Используем простой RViz конфиг
    ))

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Webots) clock'
    )
    declare_rviz_config_file_argument = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_webots_ros2_tesla, 'rviz', 'tesla_debug.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    webots_tesla_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tesla_simulation_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        # Если robot_launch.py имеет 'run_lane_follower', передайте 'false', чтобы робот стоял:
        # launch_arguments={'use_sim_time': use_sim_time, 'run_lane_follower': 'false'}.items()
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', actual_pointcloud_topic), ('scan', '/scan')],
        parameters=[{
            'target_frame': base_frame_id,
            'transform_tolerance': 0.1, # Увеличено для большей устойчивости
            'min_height': 0.85,
            'max_height': 1.0,
            'angle_min': -3.14159, 'angle_max': 3.14159,
            'angle_increment': 0.00613, # Для 1024 точек на 360 градусов
            'scan_time': 0.1, 'range_min': 1.5, 'range_max': 100.0,
            'use_inf': True, 'inf_epsilon': 1.0, 'use_sim_time': use_sim_time,
        }]
    )

    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_rviz_config_file_argument,
        webots_tesla_simulation,
        pointcloud_to_laserscan_node,
        start_rviz_node,
    ])