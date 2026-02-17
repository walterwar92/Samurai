#!/usr/bin/env python3
"""
mqtt_bridge_node — Bridges ROS2 topics ↔ MQTT for multi-robot communication.

MQTT topics:
  samurai/robot1/voice_command  ← from Android app / second robot
  samurai/robot1/call_robot     ← call for help
  samurai/robot2/call_robot     → outgoing call to second robot
  samurai/robot1/status         → current robot state

ROS2 topics:
  /voice_command     (std_msgs/String) — bidirectional
  /call_second_robot (std_msgs/String) — trigger to call robot 2
  /robot_status      (std_msgs/String) — current FSM state
"""

import json
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt


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


MQTT_BROKER = _get_local_ip()
MQTT_PORT = 1883
ROBOT_ID = 'robot1'


class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        self.declare_parameter('broker', MQTT_BROKER)
        self.declare_parameter('port', MQTT_PORT)
        self.declare_parameter('robot_id', ROBOT_ID)

        broker = self.get_parameter('broker').value
        port = self.get_parameter('port').value
        self._robot_id = self.get_parameter('robot_id').value

        # MQTT client
        self._mqtt = mqtt.Client(client_id=f'samurai_{self._robot_id}')
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        self._mqtt.connect_async(broker, port)
        self._mqtt.loop_start()

        # ROS2 publishers (MQTT → ROS2)
        self._voice_pub = self.create_publisher(String, '/voice_command', 10)
        self._call_pub = self.create_publisher(String, '/incoming_call', 10)

        # ROS2 subscribers (ROS2 → MQTT)
        self.create_subscription(
            String, '/call_second_robot', self._call_robot_cb, 10)
        self.create_subscription(
            String, '/robot_status', self._status_cb, 10)

        self.get_logger().info(
            f'MQTT bridge started: {broker}:{port} id={self._robot_id}')

    def _on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f'MQTT connected (rc={rc})')
        # Subscribe to incoming topics
        client.subscribe(f'samurai/{self._robot_id}/voice_command')
        client.subscribe(f'samurai/{self._robot_id}/call_robot')

    def _on_message(self, client, userdata, mqtt_msg):
        topic = mqtt_msg.topic
        payload = mqtt_msg.payload.decode('utf-8', errors='replace')
        self.get_logger().info(f'MQTT recv [{topic}]: {payload}')

        if topic.endswith('/voice_command'):
            msg = String()
            msg.data = payload
            self._voice_pub.publish(msg)
        elif topic.endswith('/call_robot'):
            msg = String()
            msg.data = payload
            self._call_pub.publish(msg)

    def _call_robot_cb(self, msg: String):
        other_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
        payload = json.dumps({
            'from': self._robot_id,
            'action': 'call',
            'data': msg.data,
        })
        self._mqtt.publish(f'samurai/{other_id}/call_robot', payload)
        self.get_logger().info(f'Called {other_id}: {msg.data}')

    def _status_cb(self, msg: String):
        self._mqtt.publish(
            f'samurai/{self._robot_id}/status', msg.data)

    def destroy_node(self):
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
