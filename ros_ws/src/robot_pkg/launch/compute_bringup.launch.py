"""
compute_bringup.launch.py — Launch SLAM, Nav2, and YOLO on the laptop.

Runs on the laptop (i9-13900H) with the same ROS_DOMAIN_ID as the Pi.
Receives /camera/image_raw, /range, /imu/data, /odom over WiFi DDS.

Usage:
    docker build -t samurai /home/rs/Projects/Samurai
    docker run -it --net=host -v /home/rs/Projects/Samurai:/root/Samurai samurai bash
    # Inside container:
    cd /root/Samurai/ros_ws
    colcon build --symlink-install
    source install/setup.bash
    export ROS_DOMAIN_ID=42
    ros2 launch robot_pkg compute_bringup.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_pkg')
    config_dir = os.path.join(pkg_share, 'config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    # ── robot_localization (EKF: fuse wheel odom + IMU) ──────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'robot_localization.yaml')],
    )

    # ── Monocular depth → LaserScan (MiDaS + ultrasonic) ────
    depth_to_scan_node = Node(
        package='robot_pkg',
        executable='depth_to_scan_node',
        name='depth_to_scan',
        output='screen',
        parameters=[{
            'camera_hfov': 1.085,       # ~62° RPi CSI v2
            'max_range': 3.0,
            'num_beams': 180,
            'publish_rate': 10.0,
            'scale_smoothing': 0.8,
        }],
    )

    # ── SLAM Toolbox (online async) ──────────────────────────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'slam_toolbox_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ── Nav2 bringup ─────────────────────────────────────────
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py'),
        ),
        launch_arguments={
            'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # ── YOLO detection node ──────────────────────────────────
    yolo_node = Node(
        package='robot_pkg',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model': 'yolov8n.pt',
            'confidence': 0.45,
            'device': 'cpu',
        }],
    )

    # ── Web Dashboard ─────────────────────────────────────────
    dashboard_node = Node(
        package='robot_pkg',
        executable='dashboard_node',
        name='dashboard',
        output='screen',
        parameters=[{'port': 5000}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
        depth_to_scan_node,
        slam_toolbox_node,
        nav2_launch,
        yolo_node,
        dashboard_node,
    ])
