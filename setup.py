import os
from glob import glob
from setuptools import setup

package_name = 'warehouse_amr_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxim Jakinin',
    maintainer_email='maxim.mjakinin@gmail.com',
    description='Autonomous Warehouse AMR with 2D-LiDAR SLAM, Costmaps, Nav2 and Mission Dispatcher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'warehouse_simulator_node = warehouse_amr_navigation.warehouse_simulator_node:main',
            'warehouse_mission_node = warehouse_amr_navigation.warehouse_mission_node:main',
            'amr_telemetry_dashboard = warehouse_amr_navigation.amr_telemetry_dashboard:main',
        ],
    },
)
