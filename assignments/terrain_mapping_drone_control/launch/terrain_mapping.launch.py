#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """Generate launch description for terrain mapping with camera bridge."""
    
    # Create bridge nodes for each topic
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )

    depth_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_camera_bridge',
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=[
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    pointcloud_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pointcloud_bridge',
        arguments=[
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloud',
        ],
        output='screen'
    )

    # Launch spiral trajectory controller
    controller_node = Node(
        package='terrain_mapping_drone_control',
        executable='spiral_trajectory',
        name='spiral_trajectory',
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        camera_bridge,
        depth_camera_bridge,
        camera_info_bridge,
        pointcloud_bridge,
        controller_node
    ]) 