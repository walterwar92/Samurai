#!/usr/bin/env python3
"""
gesture_node — Hand gesture recognition via MediaPipe.

Recognizes gestures: open_palm (stop), fist (grab), point_up (forward),
point_left/right (turn), thumbs_up (follow me).

Subscribes:
    /camera/image_raw  (Image)  — camera feed

Publishes:
    /gesture/command  (String)  — gesture name or "" if none
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    import mediapipe as mp
    _MP = True
except ImportError:
    _MP = False


def _classify_hand(landmarks) -> str:
    """Classify hand gesture from MediaPipe landmarks."""
    # Landmark indices: WRIST=0, THUMB_TIP=4, INDEX_TIP=8,
    # MIDDLE_TIP=12, RING_TIP=16, PINKY_TIP=20
    # *_MCP = 1,5,9,13,17   *_PIP = 2,6,10,14,18
    tips = [4, 8, 12, 16, 20]
    pips = [3, 6, 10, 14, 18]

    lm = landmarks.landmark

    # Finger extended check (tip above pip in y, lower y = higher)
    extended = []
    # Thumb: compare x distance from wrist
    thumb_ext = abs(lm[4].x - lm[0].x) > abs(lm[3].x - lm[0].x)
    extended.append(thumb_ext)
    for tip, pip in zip(tips[1:], pips[1:]):
        extended.append(lm[tip].y < lm[pip].y)

    num_extended = sum(extended)

    # All fingers extended → open palm (stop)
    if num_extended >= 4:
        return 'stop'

    # No fingers extended → fist (grab)
    if num_extended == 0:
        return 'grab'

    # Only index extended
    if extended[1] and num_extended == 1:
        # Check direction
        ix = lm[8].x
        if ix < 0.35:
            return 'point_left'
        elif ix > 0.65:
            return 'point_right'
        else:
            return 'forward'

    # Thumb up only
    if extended[0] and num_extended == 1:
        return 'follow'

    # Index + middle extended (peace sign)
    if extended[1] and extended[2] and num_extended == 2:
        return 'forward'

    return ''


class GestureNode(Node):
    def __init__(self):
        super().__init__('gesture_node')

        self._bridge = CvBridge()
        self._frame_count = 0
        self._skip_frames = 5  # process every 5th frame (~6 Hz)
        self._last_gesture = ''

        if _MP:
            self._hands = mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.6,
                min_tracking_confidence=0.5,
            )
            self.get_logger().info('MediaPipe Hands initialized')
        else:
            self._hands = None
            self.get_logger().warn('mediapipe not available — gesture detection disabled')

        self._pub = self.create_publisher(String, '/gesture/command', 10)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 5)

        self.get_logger().info('Gesture node started')

    def _image_cb(self, msg: Image):
        if self._hands is None:
            return

        self._frame_count += 1
        if self._frame_count % self._skip_frames != 0:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = self._hands.process(frame)

        gesture = ''
        if results.multi_hand_landmarks:
            gesture = _classify_hand(results.multi_hand_landmarks[0])

        # Only publish on change or periodic clear
        if gesture != self._last_gesture:
            self._last_gesture = gesture
            out = String()
            out.data = gesture
            self._pub.publish(out)
            if gesture:
                self.get_logger().info(f'Gesture: {gesture}')

    def destroy_node(self):
        if self._hands is not None:
            self._hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
