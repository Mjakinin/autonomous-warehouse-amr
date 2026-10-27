import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_amr_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # Arguments
    map_yaml_file = os.path.join(pkg_share, 'maps', 'warehouse_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'warehouse_amr.rviz')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to launch RViz2'
    )
    declare_auto_mission = DeclareLaunchArgument(
        'auto_mission', default_value='false', description='Whether to automatically dispatch orders'
    )
    declare_dynamic_obs = DeclareLaunchArgument(
        'dynamic_obstacle', default_value='true', description='Enable crossing dynamic obstacle'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    auto_mission = LaunchConfiguration('auto_mission')
    dynamic_obstacle = LaunchConfiguration('dynamic_obstacle')

    # 1. High-Fidelity Warehouse Simulation Node
    sim_node = Node(
        package='warehouse_amr_navigation',
        executable='warehouse_simulator_node',
        name='warehouse_simulator_node',
        output='screen',
        parameters=[{
            'init_x': 2.0,
            'init_y': 2.0,
            'init_theta': 0.0,
            'enable_dynamic_obstacle': dynamic_obstacle,
        }]
    )

    # 2. Genuine ROS 2 Nav2 Navigation & Localization Stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    # 3. Fleet Mission Dispatcher Node
    mission_node = Node(
        package='warehouse_amr_navigation',
        executable='warehouse_mission_node',
        name='warehouse_mission_dispatcher',
        output='screen',
        condition=IfCondition(auto_mission)
    )

    # 4. RViz2 Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_auto_mission,
        declare_dynamic_obs,
        sim_node,
        nav2_bringup,
        mission_node,
        rviz_node,
    ])
