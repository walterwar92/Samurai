#!/usr/bin/env python3
"""
geofence_map_publisher — Publishes a static OccupancyGrid representing
the arena perimeter (geofencing polygon for nav2 costmap).

The arena boundary is defined as a polygon. Everything outside is
lethal cost (254). This is subscribed by the geofence_layer in
global_costmap as a static layer on /geofence_map.

Usage:
  ros2 run robot_pkg geofence_map_publisher --ros-args \
    -p arena_polygon:="[[0,0],[3,0],[3,2],[0,2]]" \
    -p resolution:=0.05
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy


class GeofenceMapPublisher(Node):
    def __init__(self):
        super().__init__('geofence_map_publisher')

        # Arena polygon as JSON list of [x, y] points (metres)
        self.declare_parameter(
            'arena_polygon', '[[0,0],[3,0],[3,2],[0,2]]')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('padding', 0.5)  # extra space around polygon

        poly_str = self.get_parameter('arena_polygon').value
        self._resolution = self.get_parameter('resolution').value
        self._padding = self.get_parameter('padding').value
        self._polygon = json.loads(poly_str)

        # Use transient local so late subscribers get the map
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(OccupancyGrid, '/geofence_map', qos)

        self._publish_map()
        self.get_logger().info('Geofence map published')

    def _publish_map(self):
        poly = np.array(self._polygon, dtype=np.float64)
        x_min = poly[:, 0].min() - self._padding
        y_min = poly[:, 1].min() - self._padding
        x_max = poly[:, 0].max() + self._padding
        y_max = poly[:, 1].max() + self._padding

        width = int((x_max - x_min) / self._resolution)
        height = int((y_max - y_min) / self._resolution)

        # Create grid: 0=free inside polygon, 100=lethal outside
        grid = np.full((height, width), 100, dtype=np.int8)

        for iy in range(height):
            for ix in range(width):
                px = x_min + ix * self._resolution
                py = y_min + iy * self._resolution
                if self._point_in_polygon(px, py, poly):
                    grid[iy, ix] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self._resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = x_min
        msg.info.origin.position.y = y_min
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()

        self._pub.publish(msg)

    @staticmethod
    def _point_in_polygon(x: float, y: float, poly: np.ndarray) -> bool:
        """Ray-casting algorithm."""
        n = len(poly)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
