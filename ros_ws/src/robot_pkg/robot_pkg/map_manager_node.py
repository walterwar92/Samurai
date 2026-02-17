#!/usr/bin/env python3
"""
map_manager_node — Save and load SLAM maps.

Saves OccupancyGrid as PGM + YAML (nav2_map_server compatible).
Storage: ~/samurai_maps/

Subscribes:
    /map                (OccupancyGrid)  — current SLAM map
    /map_manager/save   (String)         — filename to save as
    /map_manager/load   (String)         — filename to load
    /map_manager/list   (String)         — trigger to list maps

Publishes:
    /map_manager/map_list  (String)  — JSON list of saved map names
    /map_manager/status    (String)  — last operation result
"""

import json
import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

MAPS_DIR = os.path.expanduser('~/samurai_maps')


class MapManagerNode(Node):
    def __init__(self):
        super().__init__('map_manager_node')

        os.makedirs(MAPS_DIR, exist_ok=True)

        self._latest_map = None
        self._latest_map_info = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(String, '/map_manager/save', self._save_cb, 10)
        self.create_subscription(String, '/map_manager/load', self._load_cb, 10)
        self.create_subscription(String, '/map_manager/list', self._list_cb, 10)

        self._list_pub = self.create_publisher(String, '/map_manager/map_list', 10)
        self._status_pub = self.create_publisher(String, '/map_manager/status', 10)
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        self.get_logger().info(f'Map manager — storage: {MAPS_DIR}')

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _save_cb(self, msg: String):
        name = msg.data.strip()
        if not name:
            self._pub_status('error: empty name')
            return
        if self._latest_map is None:
            self._pub_status('error: no map available')
            return

        m = self._latest_map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        # Convert to PGM (nav2 format: 0=occupied → 0, 100=free → 254, -1=unknown → 205)
        data = np.array(m.data, dtype=np.int8).reshape((h, w))
        pgm = np.full((h, w), 205, dtype=np.uint8)  # unknown
        pgm[data == 0] = 254      # free
        pgm[data >= 1] = np.clip(255 - (data[data >= 1].astype(np.uint16) * 255 // 100), 0, 253).astype(np.uint8)
        pgm = np.flipud(pgm)  # PGM origin is top-left

        pgm_path = os.path.join(MAPS_DIR, f'{name}.pgm')
        yaml_path = os.path.join(MAPS_DIR, f'{name}.yaml')

        # Write PGM
        with open(pgm_path, 'wb') as f:
            header = f'P5\n{w} {h}\n255\n'.encode()
            f.write(header)
            f.write(pgm.tobytes())

        # Write YAML
        meta = {
            'image': f'{name}.pgm',
            'resolution': float(res),
            'origin': [float(ox), float(oy), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False)

        self.get_logger().info(f'Map saved: {name}')
        self._pub_status(f'saved: {name}')

    def _load_cb(self, msg: String):
        name = msg.data.strip()
        yaml_path = os.path.join(MAPS_DIR, f'{name}.yaml')
        pgm_path = os.path.join(MAPS_DIR, f'{name}.pgm')

        if not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
            self._pub_status(f'error: map "{name}" not found')
            return

        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        with open(pgm_path, 'rb') as f:
            # Parse PGM P5
            magic = f.readline().decode().strip()
            if magic != 'P5':
                self._pub_status('error: invalid PGM format')
                return
            # Skip comments
            line = f.readline().decode().strip()
            while line.startswith('#'):
                line = f.readline().decode().strip()
            w, h = map(int, line.split())
            _maxval = int(f.readline().decode().strip())
            pgm_data = np.frombuffer(f.read(), dtype=np.uint8).reshape((h, w))

        # Convert PGM → OccupancyGrid data
        pgm_data = np.flipud(pgm_data)
        grid_data = np.full(h * w, -1, dtype=np.int8)
        flat = pgm_data.flatten()
        grid_data[flat >= 240] = 0    # free
        grid_data[flat <= 20] = 100   # occupied
        # values between are scaled
        mid = (flat > 20) & (flat < 240)
        grid_data[mid] = (100 - (flat[mid].astype(np.int16) * 100 // 255)).astype(np.int8)

        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        occ_grid.header.frame_id = 'map'
        occ_grid.info.resolution = meta.get('resolution', 0.05)
        occ_grid.info.width = w
        occ_grid.info.height = h
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        occ_grid.info.origin.position.x = origin[0]
        occ_grid.info.origin.position.y = origin[1]
        occ_grid.data = grid_data.tolist()

        self._map_pub.publish(occ_grid)
        self.get_logger().info(f'Map loaded: {name} ({w}x{h})')
        self._pub_status(f'loaded: {name}')

    def _list_cb(self, msg: String):
        maps = []
        for f in os.listdir(MAPS_DIR):
            if f.endswith('.yaml'):
                maps.append(f[:-5])
        maps.sort()
        out = String()
        out.data = json.dumps(maps)
        self._list_pub.publish(out)

    def _pub_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
