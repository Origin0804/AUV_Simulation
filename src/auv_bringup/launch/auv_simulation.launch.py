# Copyright (c) 2025 AUV Team
# MIT License

"""Launch file for the complete AUV simulation."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for AUV simulation."""
    return LaunchDescription([
        # Environment nodes
        Node(
            package='env_pkg',
            executable='drag_node',
            name='drag_node',
            output='screen',
            parameters=[{
                'drag_coefficient': 0.47,
                'fluid_density': 1025.0,
                'reference_area': 0.25,
                'angular_drag_coefficient': 0.1,
            }]
        ),
        Node(
            package='env_pkg',
            executable='props_node',
            name='props_node',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,
            }]
        ),

        # AUV nodes
        Node(
            package='AUV_pkg',
            executable='position_node',
            name='position_node',
            output='screen',
            parameters=[{
                'mass': 30.0,
                'inertia_x': 2.0,
                'inertia_y': 2.0,
                'inertia_z': 1.0,
                'update_rate': 100.0,
                'buoyancy': 294.3,
            }]
        ),
        Node(
            package='AUV_pkg',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[{
                'max_thrust': 50.0,
                'motor_distance': 0.2,
            }]
        ),
        Node(
            package='AUV_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'fov_horizontal': 1.22,
                'fov_vertical': 0.87,
                'max_range': 20.0,
                'min_range': 0.3,
                'publish_rate': 30.0,
            }]
        ),
        Node(
            package='AUV_pkg',
            executable='dvl_node',
            name='dvl_node',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,
            }]
        ),
        Node(
            package='AUV_pkg',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'publish_rate': 100.0,
            }]
        ),
        Node(
            package='AUV_pkg',
            executable='main_node',
            name='main_node',
            output='screen',
            parameters=[{
                'control_rate': 50.0,
            }]
        ),
    ])
