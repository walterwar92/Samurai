"""
robot_bringup.launch.py — [DEPRECATED] Launch all onboard nodes on Raspberry Pi.

╔══════════════════════════════════════════════════════════════════════╗
║  УСТАРЕЛ — Pi теперь использует чистый Python + MQTT (без ROS2)    ║
║  Новый способ: ./start_robot_mqtt.sh                               ║
║  Launcher:     python3 -m pi_nodes.robot_launcher                  ║
╚══════════════════════════════════════════════════════════════════════╝

Runs: motor, ultrasonic, camera, laser, servo, imu, voice, fsm, mqtt_bridge.
SLAM / Nav2 / YOLO run on the laptop (compute_bringup.launch.py).
Both machines share ROS_DOMAIN_ID for transparent DDS communication.

Usage (обычный режим — multicast, работает если роутер не блокирует):
  export ROS_DOMAIN_ID=42
  ros2 launch robot_pkg robot_bringup.launch.py

Usage (через мобильный хотспот — unicast, указать IP ноутбука):
  export ROS_DOMAIN_ID=42
  ros2 launch robot_pkg robot_bringup.launch.py peer_ip:=192.168.43.XXX
"""

import os
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _setup_fastdds_unicast(context, *args, **kwargs):
    """
    Если задан peer_ip — генерирует XML конфиг unicast DDS и устанавливает
    переменную окружения FASTRTPS_DEFAULT_PROFILES_FILE.
    Вызывается через OpaqueFunction перед стартом нод.
    """
    peer_ip = LaunchConfiguration('peer_ip').perform(context)
    if not peer_ip:
        print('[DDS] peer_ip не задан — используется стандартный multicast DDS')
        return []

    local_ip = _get_local_ip()
    config_path = '/tmp/fastdds_samurai_robot.xml'

    xml = f"""<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UnicastUDP</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_participant" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>UnicastUDP</transport_id>
            </userTransports>
            <builtin>
                <discovery_config>
                    <initialPeersList>
                        <locator>
                            <udpv4>
                                <address>{local_ip}</address>
                            </udpv4>
                        </locator>
                        <locator>
                            <udpv4>
                                <address>{peer_ip}</address>
                            </udpv4>
                        </locator>
                    </initialPeersList>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>"""

    with open(config_path, 'w') as f:
        f.write(xml)

    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = config_path
    print(f'[DDS Unicast] Pi IP: {local_ip} | Ноутбук IP: {peer_ip} | Конфиг: {config_path}')
    return []


def generate_launch_description():
    # ── Аргументы запуска ────────────────────────────────────
    broker_arg = DeclareLaunchArgument(
        'mqtt_broker', default_value=_get_local_ip(),
        description='MQTT broker IP address (auto-detected)')
    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='robot1',
        description='Unique robot identifier')
    peer_ip_arg = DeclareLaunchArgument(
        'peer_ip', default_value='',
        description=(
            'IP ноутбука (compute node) в сети хотспота для unicast DDS. '
            'Оставь пустым если оба устройства в одной LAN с multicast. '
            'Пример: peer_ip:=192.168.43.50'))

    # Настройка unicast DDS (выполняется до старта нод)
    unicast_setup = OpaqueFunction(function=_setup_fastdds_unicast)

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
    # voice_node убран: распознавание речи работает на телефоне (VoskRecognizer.kt)
    # и передаётся через MQTT → mqtt_bridge_node → /voice_command

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
        # Аргументы
        broker_arg, robot_id_arg, peer_ip_arg,
        # Unicast DDS (если задан peer_ip)
        unicast_setup,
        # Ноды
        motor_node, ultrasonic_node, camera_node,
        laser_node, servo_node, imu_node,
        fsm_node, mqtt_bridge_node,
        battery_node, temperature_node, watchdog_node,
        tf_ultrasonic, tf_camera, tf_imu, tf_laser,
    ])
