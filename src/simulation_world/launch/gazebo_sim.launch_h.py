#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launches Gazebo Classic with the hospital world environment,
    sets the Gazebo model path to ~/models,
    and includes ROS-Gazebo bridge nodes.
    """

    # Get the package share directory
    pkg_share = get_package_share_directory('simulation_world')
    world_file = os.path.join(pkg_share, 'worlds', 'hospital_area.world')
    
    # Set GAZEBO_MODEL_PATH to append ~/models
    models_path = os.pathsep + os.path.join(os.path.expanduser('~'), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = models_path

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.environ['GAZEBO_MODEL_PATH']
    )

    # Launch Gazebo Classic
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )

    # Bridge for cmd_vel and camera (same topics but using Gazebo Classic)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/keyboard/keypress@std_msgs/msg/Int32@gz.msgs.Int32",
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/depth_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        bridge,
    ])
