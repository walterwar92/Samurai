"""
robot_bringup.launch.py — Launch all onboard nodes on Raspberry Pi.

Runs: motor, ultrasonic, camera, laser, servo, imu, voice, fsm, mqtt_bridge.
SLAM / Nav2 / YOLO run on the laptop (compute_bringup.launch.py).
Both machines share ROS_DOMAIN_ID for transparent DDS communication.

Usage:
  export ROS_DOMAIN_ID=42
  ros2 launch robot_pkg robot_bringup.launch.py
"""

import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_local_ip() -> str:
    """Auto-detect local network IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def generate_launch_description():
    # Declare arguments
    broker_arg = DeclareLaunchArgument(
        'mqtt_broker', default_value=_get_local_ip(),
        description='MQTT broker IP address (auto-detected)')
    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='robot1',
        description='Unique robot identifier')
    vosk_model_arg = DeclareLaunchArgument(
        'vosk_model', default_value='/home/pi/vosk-model-ru',
        description='Path to Vosk Russian language model')

    # ── Sensor / actuator nodes ──────────────────────────────
    motor_node = Node(
        package='robot_pkg', executable='motor_node',
        name='motor_node', output='screen',
    )

    ultrasonic_node = Node(
        package='robot_pkg', executable='ultrasonic_node',
        name='ultrasonic_node', output='screen',
    )

    camera_node = Node(
        package='robot_pkg', executable='camera_node',
        name='camera_node', output='screen',
        parameters=[{
            'width': 640, 'height': 480, 'fps': 30,
        }],
    )

    laser_node = Node(
        package='robot_pkg', executable='laser_node',
        name='laser_node', output='screen',
    )

    servo_node = Node(
        package='robot_pkg', executable='servo_node',
        name='servo_node', output='screen',
    )

    imu_node = Node(
        package='robot_pkg', executable='imu_node',
        name='imu_node', output='screen',
    )

    # ── High-level nodes ─────────────────────────────────────
    voice_node = Node(
        package='robot_pkg', executable='voice_node',
        name='voice_node', output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('vosk_model'),
        }],
    )

    fsm_node = Node(
        package='robot_pkg', executable='fsm_node',
        name='fsm_node', output='screen',
    )

    mqtt_bridge_node = Node(
        package='robot_pkg', executable='mqtt_bridge_node',
        name='mqtt_bridge_node', output='screen',
        parameters=[{
            'broker': LaunchConfiguration('mqtt_broker'),
            'port': 1883,
            'robot_id': LaunchConfiguration('robot_id'),
        }],
    )

    # ── Monitoring nodes ───────────────────────────────────────
    battery_node = Node(
        package='robot_pkg', executable='battery_node',
        name='battery_node', output='screen',
    )

    temperature_node = Node(
        package='robot_pkg', executable='temperature_node',
        name='temperature_node', output='screen',
    )

    watchdog_node = Node(
        package='robot_pkg', executable='watchdog_node',
        name='watchdog_node', output='screen',
    )

    # ── Static transforms ────────────────────────────────────
    # base_link → ultrasonic_link (front-mounted)
    tf_ultrasonic = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.10', '0', '0.05', '0', '0', '0',
                   'base_link', 'ultrasonic_link'],
    )

    # base_link → camera_link (top-front)
    tf_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.08', '0', '0.12', '0', '0.26', '0',
                   'base_link', 'camera_link'],
    )

    # base_link → imu_link (centre of HAT board)
    tf_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0.03', '0', '0', '0',
                   'base_link', 'imu_link'],
    )

    # base_link → laser_link (front, aligned with camera)
    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.10', '0', '0.10', '0', '0', '0',
                   'base_link', 'laser_link'],
    )

    return LaunchDescription([
        broker_arg, robot_id_arg, vosk_model_arg,
        motor_node, ultrasonic_node, camera_node,
        laser_node, servo_node, imu_node,
        voice_node, fsm_node, mqtt_bridge_node,
        battery_node, temperature_node, watchdog_node,
        tf_ultrasonic, tf_camera, tf_imu, tf_laser,
    ])
