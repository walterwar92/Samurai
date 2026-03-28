"""
compute_bringup.launch.py — Launch SLAM, Nav2, and YOLO on the laptop.

Runs on the laptop (i9-13900H) with the same ROS_DOMAIN_ID as the Pi.
Receives /camera/image_raw, /range, /imu/data, /odom over WiFi DDS.

Usage (обычный режим — multicast):
    docker build -t samurai /home/rs/Projects/Samurai
    docker run -it --net=host -v /home/rs/Projects/Samurai:/root/Samurai samurai bash
    # Inside container:
    cd /root/Samurai/ros_ws
    colcon build --symlink-install
    source install/setup.bash
    export ROS_DOMAIN_ID=42
    ros2 launch robot_pkg compute_bringup.launch.py

Usage (через мобильный хотспот — unicast, указать IP Raspberry Pi):
    export ROS_DOMAIN_ID=42
    ros2 launch robot_pkg compute_bringup.launch.py peer_ip:=192.168.43.XXX
"""

import os
import socket
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
    config_path = '/tmp/fastdds_samurai_compute.xml'

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
    print(f'[DDS Unicast] Ноутбук IP: {local_ip} | Pi IP: {peer_ip} | Конфиг: {config_path}')
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_pkg')
    config_dir = os.path.join(pkg_share, 'config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    peer_ip_arg = DeclareLaunchArgument(
        'peer_ip', default_value='',
        description=(
            'IP Raspberry Pi в сети хотспота для unicast DDS. '
            'Оставь пустым если оба устройства в одной LAN с multicast. '
            'Пример: peer_ip:=192.168.43.100'))

    remote_yolo_arg = DeclareLaunchArgument(
        'remote_yolo', default_value='false',
        description=(
            'Если true — YOLO запускается на отдельном GPU-ноутбуке '
            '(yolo_detector_mqtt.py), локальный YOLO-нод не стартует. '
            'Детекции приходят через MQTT → mqtt_bridge_compute → ROS2.'))

    # Настройка unicast DDS (выполняется до старта нод)
    unicast_setup = OpaqueFunction(function=_setup_fastdds_unicast)

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
    # Skipped when remote_yolo:=true (YOLO runs on separate GPU laptop)
    yolo_node = Node(
        package='robot_pkg',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('remote_yolo')),
        parameters=[{
            'model': '/root/Samurai/yolo11n.pt',
            'confidence': 0.45,
            'device': 'cpu',
            'detect_all_classes': True,
        }],
    )

    # ── Web Dashboard (FastAPI — compute_node version) ─────────
    dashboard_node = ExecuteProcess(
        cmd=['python3', '/root/Samurai/compute_node/dashboard_node.py'],
        name='dashboard_node',
        output='screen',
    )

    # ── New compute nodes ──────────────────────────────────────
    patrol_node = Node(
        package='robot_pkg',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
    )

    map_manager_node = Node(
        package='robot_pkg',
        executable='map_manager_node',
        name='map_manager_node',
        output='screen',
    )

    follow_me_node = Node(
        package='robot_pkg',
        executable='follow_me_node',
        name='follow_me_node',
        output='screen',
    )

    path_recorder_node = Node(
        package='robot_pkg',
        executable='path_recorder_node',
        name='path_recorder_node',
        output='screen',
    )

    qr_detector_node = Node(
        package='robot_pkg',
        executable='qr_detector_node',
        name='qr_detector_node',
        output='screen',
    )

    gesture_node = Node(
        package='robot_pkg',
        executable='gesture_node',
        name='gesture_node',
        output='screen',
    )

    # ── MQTT ↔ ROS2 Bridge (connects to Pi via MQTT) ─────────
    # Pi runs pure Python + MQTT (no ROS2), so this bridge
    # translates MQTT sensor data → ROS2 topics and vice versa.
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker', default_value='raspberrypi.local',
        description='MQTT broker IP (Pi address). Default: raspberrypi.local (mDNS).')

    mqtt_bridge_compute_node = Node(
        package='robot_pkg',
        executable='mqtt_bridge_compute',
        name='mqtt_bridge_compute',
        output='screen',
        parameters=[{
            'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            'mqtt_port': 1883,
            'robot_id': 'robot1',
        }],
    )

    return LaunchDescription([
        # Аргументы
        use_sim_time_arg,
        peer_ip_arg,
        remote_yolo_arg,
        mqtt_broker_arg,
        # Unicast DDS (если задан peer_ip)
        unicast_setup,
        # Ноды
        ekf_node,
        depth_to_scan_node,
        slam_toolbox_node,
        nav2_launch,
        yolo_node,
        dashboard_node,
        patrol_node,
        map_manager_node,
        follow_me_node,
        path_recorder_node,
        qr_detector_node,
        gesture_node,
        mqtt_bridge_compute_node,
    ])
