"""webots_ros2 package setup file."""
from setuptools import setup
import os
from glob import glob
# from glob import glob # glob сейчас не используется для config и rviz, если мы указываем файлы явно

package_name = 'webots_ros2_tesla'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

# Launch-файлы (устанавливаем все .py из директории launch)
data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py'))) # Используем glob здесь, т.к. у нас два launch-файла

# Config-файлы
# Устанавливаем ТОЛЬКО slam_toolbox_params.yaml
# Убедитесь, что этот файл существует: src/webots_ros2_tesla/config/slam_toolbox_params.yaml
data_files.append((os.path.join('share', package_name, 'config'), ['config/slam_toolbox_params.yaml']))

# RViz-файлы
# Устанавливаем ТОЛЬКО tesla_debug.rviz (или tesla_slam.rviz, если вы его используете)
# Убедитесь, что этот файл существует: src/webots_ros2_tesla/rviz/tesla_debug.rviz
data_files.append((os.path.join('share', package_name, 'rviz'), ['rviz/tesla_debug.rviz'])) # ИЛИ 'rviz/tesla_slam.rviz'


# Worlds
data_files.append((os.path.join('share', package_name, 'worlds'), [
    'worlds/tesla_world.wbt', 'worlds/.tesla_world.wbproj',
]))
# Resource файлы (URDF и другие)
data_files.append((os.path.join('share', package_name, 'resource'), [
    'resource/tesla_webots.urdf',
    'resource/tesla_tf_description.urdf'
]))
# Package.xml
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='2025.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Tesla'],
    description='Tesla ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follower = webots_ros2_tesla.lane_follower:main',
            # 'ground_truth_odometry = ...' ЗАКОММЕНТИРОВАН или УДАЛЕН для этого этапа
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)