
import os

from ament_index_python.packages import get_package_share_directory #type: ignore

from launch import LaunchDescription # type: ignore
from launch.actions import GroupAction # type: ignore
from launch_ros.actions import ComposableNodeContainer # type: ignore
from launch_ros.descriptions import ComposableNode # type: ignore
from launch_ros.actions import Node  # type: ignore


def generate_launch_description():

    lifecycle_nodes = ['map_server']
    map_yaml_path_dir = os.path.join(
        get_package_share_directory('isaac_ros_occupancy_grid_localizer'), 'maps', 'map-rob.yaml')

    occupancy_grid_localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        parameters=[map_yaml_path_dir, {
            'loc_result_frame': 'map',
            'map_yaml_path': map_yaml_path_dir,
        }])

    occupancy_grid_localizer_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='occupancy_grid_localizer_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            occupancy_grid_localizer_node,
        ],
        output='screen'
    )

    # Seting transform between lidar_frame and base_link
    # since Isaac Sim does not set this transform
    baselink_lidar_publisher = Node(
        package='tf2_ros', executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link',
                   'lidar_frame'])

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[{'yaml_filename': map_yaml_path_dir}]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription([occupancy_grid_localizer_container,
                            baselink_lidar_publisher])

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_nodes)

    return ld
