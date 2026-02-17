#!/usr/bin/env python3
"""
fsm_node — Finite State Machine for autonomous ball-hunting robot.

States:
  IDLE         — waiting for voice command
  SEARCHING    — navigating arena, scanning for balls via YOLO detections
  TARGETING    — rotating to centre ball in camera
  APPROACHING  — moving toward detected ball
  GRABBING     — positioning claw, closing on ball
  BURNING      — activating laser to burn ball
  CALLING      — requesting second robot via MQTT
  RETURNING    — returning to home / safe zone
  PATROLLING   — autonomous waypoint patrol
  FOLLOWING    — follow-me mode (person tracking)
  PATH_REPLAY  — replaying a recorded path

Subscribes:
  /voice_command        (String) — parsed voice commands
  /ball_detection       (String) — JSON from YOLO node: {colour, x, y, w, h, conf}
  /range                (Range)  — ultrasonic proximity
  /incoming_call        (String) — call from second robot
  /gesture/command      (String) — hand gesture commands

Publishes:
  /cmd_vel              (Twist)  — motor commands
  /claw/command         (String) — claw open/close
  /laser/command        (Bool)   — laser on/off
  /call_second_robot    (String) — MQTT call trigger
  /robot_status         (String) — current state for monitoring
  /goal_pose            (PoseStamped) — nav2 goal
  /patrol/command       (String) — patrol control
  /follow_me/command    (String) — follow-me control
  /path_recorder/command (String) — path recorder control
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
    PATROLLING = 'PATROLLING'
    FOLLOWING = 'FOLLOWING'
    PATH_REPLAY = 'PATH_REPLAY'


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
        self._patrol_cmd_pub = self.create_publisher(String, '/patrol/command', 10)
        self._follow_cmd_pub = self.create_publisher(String, '/follow_me/command', 10)
        self._path_cmd_pub = self.create_publisher(String, '/path_recorder/command', 10)

        # Subscribers
        self.create_subscription(String, '/voice_command', self._voice_cb, 10)
        self.create_subscription(String, '/ball_detection', self._ball_cb, 10)
        self.create_subscription(Range, '/range', self._range_cb, 10)
        self.create_subscription(String, '/incoming_call', self._call_recv_cb, 10)
        self.create_subscription(String, '/gesture/command', self._gesture_cb, 10)

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

        if 'патрул' in text or 'обход' in text:
            self._transition(State.PATROLLING)
            return

        if 'следуй' in text or 'за мной' in text:
            self._transition(State.FOLLOWING)
            return

        if 'запиши путь' in text or 'запись' in text:
            self._pub_string(self._path_cmd_pub, 'record')
            self.get_logger().info('Path recording started')
            return

        if 'воспроизведи' in text or 'повтори путь' in text:
            self._transition(State.PATH_REPLAY)
            return

    # ── Gesture command handler ────────────────────────────────
    def _gesture_cb(self, msg: String):
        gesture = msg.data.strip()
        if not gesture:
            return

        if gesture == 'stop':
            self._transition(State.IDLE)
        elif gesture == 'forward' and self._state == State.IDLE:
            twist = Twist()
            twist.linear.x = 0.15
            self._cmd_pub.publish(twist)
        elif gesture == 'grab' and self._state == State.IDLE:
            self._target_action = 'grab'
            self._target_colour = ''
            self._transition(State.SEARCHING)
        elif gesture == 'follow':
            self._transition(State.FOLLOWING)
        elif gesture == 'point_left' and self._state == State.IDLE:
            twist = Twist()
            twist.angular.z = 0.5
            self._cmd_pub.publish(twist)
        elif gesture == 'point_right' and self._state == State.IDLE:
            twist = Twist()
            twist.angular.z = -0.5
            self._cmd_pub.publish(twist)

    def _extract_colour(self, text: str) -> str:
        text_norm = text.replace('ё', 'е')
        colours = {
            'красн': 'red', 'синий': 'blue', 'синего': 'blue',
            'зелен': 'green', 'желт': 'yellow', 'бел': 'white',
            'черн': 'black', 'оранж': 'orange',
        }
        for rus, eng in colours.items():
            if rus in text_norm:
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

        # Exit actions: stop delegated modes when leaving
        if old == State.PATROLLING:
            self._pub_string(self._patrol_cmd_pub, 'stop')
        if old == State.FOLLOWING:
            self._pub_string(self._follow_cmd_pub, 'stop')
        if old == State.PATH_REPLAY:
            self._pub_string(self._path_cmd_pub, 'stop')

        # Entry actions
        if new_state == State.IDLE:
            self._stop_all()
        elif new_state == State.SEARCHING:
            self._search_accumulated = 0.0
            self._prev_theta = self._get_yaw()
        elif new_state == State.CALLING:
            self._do_call()
        elif new_state == State.PATROLLING:
            self._pub_string(self._patrol_cmd_pub, 'start')
        elif new_state == State.FOLLOWING:
            self._pub_string(self._follow_cmd_pub, 'start')
        elif new_state == State.PATH_REPLAY:
            self._pub_string(self._path_cmd_pub, 'replay')

    def _get_yaw(self) -> float:
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
        # PATROLLING, FOLLOWING, PATH_REPLAY are handled by their respective nodes

    # ── Behaviours ───────────────────────────────────────────
    def _do_search(self):
        self._search_accumulated += 0.4 * 0.1

        if self._search_accumulated > 2 * math.pi + 0.3:
            colour = self._target_colour or 'any'
            self.get_logger().warn(f'Ball not found ({colour}) — full rotation')
            self._transition(State.IDLE)
            return

        det = self._ball_detection
        if det and self._colour_matches(det):
            self.get_logger().info(
                f'Ball spotted: {det.get("colour")} — targeting')
            self._cmd_pub.publish(Twist())
            self._transition(State.TARGETING)
            return

        twist = Twist()
        twist.angular.z = -0.4
        self._cmd_pub.publish(twist)

    def _do_targeting(self):
        self._approach_timeout += 0.1
        det = self._ball_detection

        if det is None or not self._colour_matches(det):
            self._lost_frames += 1
            twist = Twist()
            twist.angular.z = self._last_steer * 0.3
            self._cmd_pub.publish(twist)
            if self._lost_frames > 15:
                self.get_logger().warn('Lost ball during targeting')
                self._transition(State.SEARCHING)
            return

        self._lost_frames = 0
        img_cx = 320
        ball_cx = det.get('x', img_cx) + det.get('w', 0) / 2.0
        error_x = (ball_cx - img_cx) / img_cx

        if abs(error_x) < 0.15:
            self._cmd_pub.publish(Twist())
            self.get_logger().info(
                f'Ball centred — approaching {det.get("colour")}')
            self._transition(State.APPROACHING)
            return

        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.4 + target_angular * 0.6
        self._last_steer = angular

        twist = Twist()
        twist.angular.z = angular
        self._cmd_pub.publish(twist)

    def _do_approach(self):
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

        if abs_error > 0.5:
            self._cmd_pub.publish(Twist())
            self._transition(State.TARGETING)
            return

        target_angular = -error_x * 0.8
        angular = self._last_steer * 0.3 + target_angular * 0.7
        self._last_steer = angular

        twist = Twist()
        twist.angular.z = angular
        twist.linear.x = 0.12 if abs_error < 0.15 else 0.06

        if self._range_m < 0.10:
            self._cmd_pub.publish(Twist())
            if self._target_action == 'burn':
                self._transition(State.BURNING)
            else:
                self._transition(State.GRABBING)
            return

        self._cmd_pub.publish(twist)

    def _do_grab(self):
        self._pub_claw('open')
        self._approach_timeout += 0.1
        if self._approach_timeout < 1.0:
            twist = Twist()
            twist.linear.x = 0.05
            self._cmd_pub.publish(twist)
        elif self._approach_timeout < 2.0:
            self._cmd_pub.publish(Twist())
        elif self._approach_timeout < 3.0:
            self._pub_claw('close')
        else:
            self.get_logger().info('Ball grabbed!')
            self._transition(State.RETURNING)

    def _do_burn(self):
        self._approach_timeout += 0.1
        if self._approach_timeout < 0.5:
            self._pub_laser(True)
        elif self._approach_timeout < 5.0:
            pass
        else:
            self._pub_laser(False)
            self.get_logger().info('Burn complete')
            self._transition(State.SEARCHING)

    def _do_call(self):
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
            return True
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

    def _pub_string(self, pub, text: str):
        m = String()
        m.data = text
        pub.publish(m)

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
