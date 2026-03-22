#!/usr/bin/env python3
"""
Hand Gesture Tracker Node

Opens the webcam, detects hand landmarks via MediaPipe Hands (2D only),
interprets gestures, and publishes target pose / gripper / emergency-stop
commands to ROS 2 topics.

Publishes the annotated camera frame on /gesture/camera_feed (sensor_msgs/Image)
so it can be viewed in RViz.

Coordinate mapping (camera 2D → Panda base frame):
  Camera X (left/right)  →  Robot Y (sideways)
  Camera Y (up/down)     →  Robot Z (height)
  Robot X (forward)      →  Fixed reach distance
"""

import math

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class GestureTracker(Node):
    """ROS 2 node that captures webcam frames, runs MediaPipe hand tracking,
    and publishes gesture-derived commands."""

    # MediaPipe landmark indices
    WRIST = 0
    THUMB_TIP = 4
    INDEX_TIP = 8
    MIDDLE_TIP = 12
    RING_TIP = 16
    PINKY_TIP = 20
    INDEX_MCP = 5
    MIDDLE_MCP = 9
    RING_MCP = 13
    PINKY_MCP = 17

    def __init__(self):
        super().__init__('gesture_tracker')

        # ---------- Parameters ----------
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('smoothing_factor', 0.4)
        self.declare_parameter('pinch_threshold', 0.07)
        self.declare_parameter('fist_threshold', 0.10)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('ws_y_min', -0.3)
        self.declare_parameter('ws_y_max', 0.3)
        self.declare_parameter('ws_z_min', 0.15)
        self.declare_parameter('ws_z_max', 0.65)
        self.declare_parameter('ws_x_fixed', 0.4)

        self.camera_id = self.get_parameter('camera_id').value
        self.smoothing = self.get_parameter('smoothing_factor').value
        self.pinch_thresh = self.get_parameter('pinch_threshold').value
        self.fist_thresh = self.get_parameter('fist_threshold').value
        publish_rate = self.get_parameter('publish_rate').value
        self.ws_y_min = self.get_parameter('ws_y_min').value
        self.ws_y_max = self.get_parameter('ws_y_max').value
        self.ws_z_min = self.get_parameter('ws_z_min').value
        self.ws_z_max = self.get_parameter('ws_z_max').value
        self.ws_x_fixed = self.get_parameter('ws_x_fixed').value

        # ---------- Publishers ----------
        self.pose_pub = self.create_publisher(Pose, '/gesture/target_pose', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gesture/gripper_command', 10)
        self.estop_pub = self.create_publisher(Bool, '/gesture/emergency_stop', 10)
        # Image publisher for RViz visualization
        self.image_pub = self.create_publisher(Image, '/gesture/camera_feed', 10)
        self.cv_bridge = CvBridge()

        # ---------- MediaPipe ----------
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )

        # ---------- OpenCV ----------
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open webcam (device {self.camera_id}). '
                'Check camera_id parameter or device connection.'
            )

        # ---------- State ----------
        self.smooth_x = 0.5
        self.smooth_y = 0.5
        self.gripper_closed = False
        self.emergency = False

        # Timer drives the main loop
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            'GestureTracker started. Publishing camera feed on /gesture/camera_feed. '
            'Add an Image display in RViz to view.')

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        frame = cv2.flip(frame, 1)  # mirror
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        gesture_text = 'No hand detected'
        status_color = (128, 128, 128)
        robot_x = self.ws_x_fixed
        robot_y = 0.0
        robot_z = 0.0
        pinch_dist = 0.0
        hand_detected = False

        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]
            hand_detected = True

            # Draw hand landmarks
            self.mp_draw.draw_landmarks(
                frame, hand, self.mp_hands.HAND_CONNECTIONS,
                self.mp_styles.get_default_hand_landmarks_style(),
                self.mp_styles.get_default_hand_connections_style(),
            )

            lm = hand.landmark

            # ---- 2D wrist position (normalised 0-1) ----
            raw_x = lm[self.WRIST].x
            raw_y = lm[self.WRIST].y

            # Exponential moving average
            self.smooth_x += self.smoothing * (raw_x - self.smooth_x)
            self.smooth_y += self.smoothing * (raw_y - self.smooth_y)

            # ---- Map to robot workspace ----
            # Camera X (left/right) → Robot Y (sideways)
            robot_y = self._map(1.0 - self.smooth_x, 0.0, 1.0,
                                self.ws_y_min, self.ws_y_max)
            # Camera Y (top/bottom) → Robot Z (height)
            robot_z = self._map(1.0 - self.smooth_y, 0.0, 1.0,
                                self.ws_z_min, self.ws_z_max)
            robot_x = self.ws_x_fixed

            # ---- Pinch detection (gripper) ----
            pinch_dist = self._dist2d(lm[self.THUMB_TIP], lm[self.INDEX_TIP])
            self.gripper_closed = pinch_dist < self.pinch_thresh

            # ---- Fist detection (emergency stop) ----
            self.emergency = self._is_fist(lm)

            # ---- Publish Pose ----
            pose = Pose()
            pose.position.x = robot_x
            pose.position.y = robot_y
            pose.position.z = robot_z
            pose.orientation.x = 1.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
            self.pose_pub.publish(pose)

            # ---- Publish gripper ----
            grip_msg = Bool()
            grip_msg.data = self.gripper_closed
            self.gripper_pub.publish(grip_msg)

            # ---- Publish emergency stop ----
            estop_msg = Bool()
            estop_msg.data = self.emergency
            self.estop_pub.publish(estop_msg)

            # ---- Draw target crosshair on wrist ----
            cx = int(self.smooth_x * w)
            cy = int(self.smooth_y * h)
            cv2.drawMarker(frame, (cx, cy), (0, 255, 255),
                           cv2.MARKER_CROSS, 30, 2)
            cv2.circle(frame, (cx, cy), 8, (0, 255, 255), 2)

            # ---- Draw pinch line ----
            tx_px = int(lm[self.THUMB_TIP].x * w)
            ty_px = int(lm[self.THUMB_TIP].y * h)
            ix_px = int(lm[self.INDEX_TIP].x * w)
            iy_px = int(lm[self.INDEX_TIP].y * h)
            pinch_color = (0, 0, 255) if self.gripper_closed else (0, 255, 0)
            cv2.line(frame, (tx_px, ty_px), (ix_px, iy_px), pinch_color, 2)
            mid_px = ((tx_px + ix_px) // 2, (ty_px + iy_px) // 2)
            cv2.putText(frame, f'{pinch_dist:.3f}', mid_px,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, pinch_color, 1)

            # ---- Status ----
            if self.emergency:
                gesture_text = 'FIST - EMERGENCY STOP'
                status_color = (0, 0, 255)
            elif self.gripper_closed:
                gesture_text = 'Pinch - Gripper CLOSED'
                status_color = (0, 165, 255)
            else:
                gesture_text = 'Open hand - Gripper OPEN'
                status_color = (0, 255, 0)

        # ---- Draw status bar ----
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 105), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        cv2.circle(frame, (20, 25), 8, status_color, -1)
        cv2.putText(frame, gesture_text, (35, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if hand_detected:
            cv2.putText(frame,
                        f'Robot  X:{robot_x:.2f}  Y:{robot_y:.2f}(side)  Z:{robot_z:.2f}(height)',
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame,
                        f'Gripper: {"CLOSED" if self.gripper_closed else "OPEN"}  |  Pinch: {pinch_dist:.3f}',
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, 'LEFT/RIGHT = sideways | UP/DOWN = height',
                        (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

        # ---- Publish annotated frame to ROS topic for RViz ----
        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish image: {e}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _dist2d(a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def _is_fist(self, lm):
        palm_x = (lm[self.WRIST].x + lm[self.INDEX_MCP].x +
                  lm[self.MIDDLE_MCP].x + lm[self.RING_MCP].x +
                  lm[self.PINKY_MCP].x) / 5.0
        palm_y = (lm[self.WRIST].y + lm[self.INDEX_MCP].y +
                  lm[self.MIDDLE_MCP].y + lm[self.RING_MCP].y +
                  lm[self.PINKY_MCP].y) / 5.0

        class _P:
            pass

        palm = _P()
        palm.x = palm_x
        palm.y = palm_y

        tips = [self.THUMB_TIP, self.INDEX_TIP, self.MIDDLE_TIP,
                self.RING_TIP, self.PINKY_TIP]
        for t in tips:
            if self._dist2d(lm[t], palm) > self.fist_thresh:
                return False
        return True

    @staticmethod
    def _map(value, in_min, in_max, out_min, out_max):
        value = max(in_min, min(in_max, value))
        return out_min + (value - in_min) / (in_max - in_min) * (out_max - out_min)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
