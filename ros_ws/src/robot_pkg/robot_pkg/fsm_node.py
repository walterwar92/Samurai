#!/usr/bin/env python3
"""
fsm_node — Finite State Machine for autonomous ball-hunting robot.

States:
  IDLE        — waiting for voice command
  SEARCHING   — navigating arena, scanning for balls via YOLO detections
  APPROACHING — moving toward detected ball
  GRABBING    — positioning claw, closing on ball
  BURNING     — activating laser to burn ball
  CALLING     — requesting second robot via MQTT
  RETURNING   — returning to home / safe zone

Subscribes:
  /voice_command        (String) — parsed voice commands
  /ball_detection       (String) — JSON from YOLO node: {colour, x, y, w, h, conf}
  /range                (Range)  — ultrasonic proximity
  /incoming_call        (String) — call from second robot
  /move_base/result     (String) — nav2 goal result (simplified)

Publishes:
  /cmd_vel              (Twist)  — motor commands
  /claw/command         (String) — claw open/close
  /laser/command        (Bool)   — laser on/off
  /call_second_robot    (String) — MQTT call trigger
  /robot_status         (String) — current state for monitoring
  /goal_pose            (PoseStamped) — nav2 goal
"""

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, PoseStamped


class State:
    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    TARGETING = 'TARGETING'
    APPROACHING = 'APPROACHING'
    GRABBING = 'GRABBING'
    BURNING = 'BURNING'
    CALLING = 'CALLING'
    RETURNING = 'RETURNING'


class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        self._state = State.IDLE
        self._target_colour = ''
        self._target_action = ''  # 'grab' or 'burn'
        self._ball_detection = None
        self._range_m = float('inf')
        self._approach_timeout = 0.0
        self._last_steer = 0.0
        self._lost_frames = 0
        self._search_accumulated = 0.0
        self._prev_theta = 0.0

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._claw_pub = self.create_publisher(String, '/claw/command', 10)
        self._laser_pub = self.create_publisher(Bool, '/laser/command', 10)
        self._call_pub = self.create_publisher(String, '/call_second_robot', 10)
        self._status_pub = self.create_publisher(String, '/robot_status', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribers
        self.create_subscription(String, '/voice_command', self._voice_cb, 10)
        self.create_subscription(String, '/ball_detection', self._ball_cb, 10)
        self.create_subscription(Range, '/range', self._range_cb, 10)
        self.create_subscription(String, '/incoming_call', self._call_recv_cb, 10)

        # FSM tick — 10 Hz
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('FSM node started — state=IDLE')

    # ── Voice command parser ─────────────────────────────────
    def _voice_cb(self, msg: String):
        text = msg.data.lower().strip()
        self.get_logger().info(f'Voice: "{text}"')

        if 'вызови' in text and 'машин' in text:
            self._transition(State.CALLING)
            return

        if 'получи' in text or 'возьми' in text or 'найди' in text:
            self._target_action = 'grab'
            self._target_colour = self._extract_colour(text)
            self.get_logger().info(
                f'Target: grab {self._target_colour} ball')
            self._transition(State.SEARCHING)
            return

        if 'сожги' in text or 'прожги' in text or 'лазер' in text:
            self._target_action = 'burn'
            self._target_colour = self._extract_colour(text)
            self._transition(State.SEARCHING)
            return

        if 'стоп' in text or 'остановись' in text:
            self._transition(State.IDLE)
            return

        if 'домой' in text or 'вернись' in text:
            self._transition(State.RETURNING)
            return

    def _extract_colour(self, text: str) -> str:
        colours = {
            'красн': 'red', 'синий': 'blue', 'синего': 'blue',
            'зелен': 'green', 'желт': 'yellow', 'бел': 'white',
            'черн': 'black', 'оранж': 'orange',
        }
        for rus, eng in colours.items():
            if rus in text:
                return eng
        return ''

    # ── Sensor callbacks ─────────────────────────────────────
    def _ball_cb(self, msg: String):
        try:
            self._ball_detection = json.loads(msg.data)
        except json.JSONDecodeError:
            self._ball_detection = None

    def _range_cb(self, msg: Range):
        self._range_m = msg.range

    def _call_recv_cb(self, msg: String):
        self.get_logger().info(f'Incoming call: {msg.data}')
        # Accept call — start searching
        try:
            data = json.loads(msg.data)
            self._target_colour = data.get('colour', '')
            self._target_action = data.get('action', 'grab')
        except (json.JSONDecodeError, AttributeError):
            pass
        self._transition(State.SEARCHING)

    # ── State machine ────────────────────────────────────────
    def _transition(self, new_state: str):
        old = self._state
        self._state = new_state
        self._approach_timeout = 0.0
        self._lost_frames = 0
        self.get_logger().info(f'FSM: {old} → {new_state}')

        # Entry actions
        if new_state == State.IDLE:
            self._stop_all()
        elif new_state == State.SEARCHING:
            self._search_accumulated = 0.0
            # _prev_theta will be set on first tick from odometry
            self._prev_theta = self._get_yaw()
        elif new_state == State.CALLING:
            self._do_call()

    def _get_yaw(self) -> float:
        """Get current yaw from last odometry/detection state."""
        # Simple estimate from accumulated commands (open-loop)
        return self._prev_theta

    def _tick(self):
        if self._state == State.IDLE:
            pass

        elif self._state == State.SEARCHING:
            self._do_search()

        elif self._state == State.TARGETING:
            self._do_targeting()

        elif self._state == State.APPROACHING:
            self._do_approach()

        elif self._state == State.GRABBING:
            self._do_grab()

        elif self._state == State.BURNING:
            self._do_burn()

        elif self._state == State.RETURNING:
            self._do_return()

    # ── Behaviours ───────────────────────────────────────────
    def _do_search(self):
        """Rotate clockwise scanning for ball. Full 360° = not found."""
        # Track rotation: 10 Hz tick × 0.4 rad/s ≈ 0.04 rad/tick
        self._search_accumulated += 0.4 * 0.1  # angular_speed × dt

        if self._search_accumulated > 2 * math.pi + 0.3:
            colour = self._target_colour or 'any'
            self.get_logger().warn(f'Ball not found ({colour}) — full rotation')
            self._transition(State.IDLE)
            return

        det = self._ball_detection
        if det and self._colour_matches(det):
            self.get_logger().info(
                f'Ball spotted: {det.get("colour")} — targeting')
            self._cmd_pub.publish(Twist())  # stop rotation
            self._transition(State.TARGETING)
            return

        # Clockwise rotation
        twist = Twist()
        twist.angular.z = -0.4  # clockwise
        self._cmd_pub.publish(twist)

    def _do_targeting(self):
        """Rotate in place to centre ball in camera, then approach."""
        self._approach_timeout += 0.1
        det = self._ball_detection

        if det is None or not self._colour_matches(det):
            self._lost_frames += 1
            twist = Twist()
            twist.angular.z = self._last_steer * 0.3
            self._cmd_pub.publish(twist)
            if self._lost_frames > 15:  # ~1.5 sec
                self.get_logger().warn('Lost ball during targeting')
                self._transition(State.SEARCHING)
            return

        self._lost_frames = 0
        img_cx = 320
        ball_cx = det.get('x', img_cx) + det.get('w', 0) / 2.0
        error_x = (ball_cx - img_cx) / img_cx

        if abs(error_x) < 0.15:
            # Ball centred → approach
            self._cmd_pub.publish(Twist())
            self.get_logger().info(
                f'Ball centred — approaching {det.get("colour")}')
            self._transition(State.APPROACHING)
            return

        # Turn in place to centre
        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.4 + target_angular * 0.6
        self._last_steer = angular

        twist = Twist()
        twist.angular.z = angular
        self._cmd_pub.publish(twist)

    def _do_approach(self):
        """Drive straight toward centred ball."""
        det = self._ball_detection
        self._approach_timeout += 0.1

        if self._approach_timeout > 30.0:
            self.get_logger().warn('Approach timeout — back to search')
            self._transition(State.SEARCHING)
            return

        if det is None or not self._colour_matches(det):
            self._lost_frames += 1
            twist = Twist()
            twist.linear.x = 0.02
            twist.angular.z = self._last_steer * 0.3
            self._cmd_pub.publish(twist)
            if self._lost_frames > 20:
                self.get_logger().warn('Ball lost — back to search')
                self._transition(State.SEARCHING)
            return

        self._lost_frames = 0

        img_cx = 320
        ball_cx = det.get('x', img_cx) + det.get('w', 0) / 2.0
        error_x = (ball_cx - img_cx) / img_cx
        abs_error = abs(error_x)

        # Ball drifted too far → back to targeting
        if abs_error > 0.5:
            self._cmd_pub.publish(Twist())
            self._transition(State.TARGETING)
            return

        # Small course correction + forward
        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.3 + target_angular * 0.7
        self._last_steer = angular

        twist = Twist()
        twist.angular.z = angular
        twist.linear.x = 0.12 if abs_error < 0.15 else 0.06

        # Close enough?
        if self._range_m < 0.10:
            self._cmd_pub.publish(Twist())
            if self._target_action == 'burn':
                self._transition(State.BURNING)
            else:
                self._transition(State.GRABBING)
            return

        self._cmd_pub.publish(twist)

    def _do_grab(self):
        """Open claw, creep forward, close claw."""
        # Step 1: open
        self._pub_claw('open')
        # Small delay handled via state timing
        self._approach_timeout += 0.1
        if self._approach_timeout < 1.0:
            # Creep forward
            twist = Twist()
            twist.linear.x = 0.05
            self._cmd_pub.publish(twist)
        elif self._approach_timeout < 2.0:
            self._cmd_pub.publish(Twist())  # stop
        elif self._approach_timeout < 3.0:
            self._pub_claw('close')
        else:
            self.get_logger().info('Ball grabbed!')
            self._transition(State.RETURNING)

    def _do_burn(self):
        """Activate laser for a controlled burn."""
        self._approach_timeout += 0.1
        if self._approach_timeout < 0.5:
            self._pub_laser(True)
        elif self._approach_timeout < 5.0:
            pass  # laser is on, hold position
        else:
            self._pub_laser(False)
            self.get_logger().info('Burn complete')
            self._transition(State.SEARCHING)

    def _do_call(self):
        """Send MQTT call to second robot, then return to idle."""
        call_data = json.dumps({
            'colour': self._target_colour,
            'action': self._target_action or 'grab',
        })
        msg = String()
        msg.data = call_data
        self._call_pub.publish(msg)
        self.get_logger().info('Called second robot')
        self._transition(State.IDLE)

    def _do_return(self):
        """Navigate to home position via nav2."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        self._transition(State.IDLE)

    # ── Helpers ──────────────────────────────────────────────
    def _colour_matches(self, det: dict) -> bool:
        if not self._target_colour:
            return True  # any ball
        return det.get('colour', '') == self._target_colour

    def _stop_all(self):
        self._cmd_pub.publish(Twist())
        self._pub_laser(False)

    def _pub_claw(self, cmd: str):
        m = String()
        m.data = cmd
        self._claw_pub.publish(m)

    def _pub_laser(self, on: bool):
        m = Bool()
        m.data = on
        self._laser_pub.publish(m)

    def _publish_status(self):
        m = String()
        m.data = json.dumps({
            'state': self._state,
            'target_colour': self._target_colour,
            'target_action': self._target_action,
            'range_m': round(self._range_m, 3),
        })
        self._status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
