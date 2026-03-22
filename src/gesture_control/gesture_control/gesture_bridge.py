#!/usr/bin/env python3
"""
Gesture-to-MoveIt2-Servo Bridge Node

Uses TF2 to look up the current end-effector position so that velocity
commands are proportional to the position error (target - current),
giving true position-tracking behavior.

Coordinate convention (Panda base frame panda_link0):
  X = forward (fixed from gesture — no depth from 2D camera)
  Y = sideways (mapped from camera left/right)
  Z = height  (mapped from camera up/down)
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, TwistStamped
from std_msgs.msg import Bool
from control_msgs.action import GripperCommand as GripperCommandAction
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class GestureServoBridge(Node):

    def __init__(self):
        super().__init__('gesture_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('servo_publish_rate', 30.0)
        self.declare_parameter('ws_y_min', -0.3)
        self.declare_parameter('ws_y_max', 0.3)
        self.declare_parameter('ws_z_min', 0.15)
        self.declare_parameter('ws_z_max', 0.65)
        self.declare_parameter('ws_x_fixed', 0.4)
        self.declare_parameter('planning_frame', 'panda_link0')
        self.declare_parameter('ee_frame', 'panda_link8')
        self.declare_parameter('velocity_gain', 3.0)
        self.declare_parameter('deadzone', 0.01)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('position_tolerance', 0.005)
        self.declare_parameter('gripper_open_pos', 0.04)
        self.declare_parameter('gripper_close_pos', 0.0)

        self.servo_rate = self.get_parameter('servo_publish_rate').value
        self.ws_y_min = self.get_parameter('ws_y_min').value
        self.ws_y_max = self.get_parameter('ws_y_max').value
        self.ws_z_min = self.get_parameter('ws_z_min').value
        self.ws_z_max = self.get_parameter('ws_z_max').value
        self.ws_x_fixed = self.get_parameter('ws_x_fixed').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.velocity_gain = self.get_parameter('velocity_gain').value
        self.deadzone = self.get_parameter('deadzone').value
        self.max_speed = self.get_parameter('max_linear_speed').value
        self.pos_tolerance = self.get_parameter('position_tolerance').value
        self.gripper_open = self.get_parameter('gripper_open_pos').value
        self.gripper_close = self.get_parameter('gripper_close_pos').value

        # ---------- Callback groups ----------
        self.cb_group = ReentrantCallbackGroup()

        # ---------- TF2 for current EE position ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- State ----------
        self.target_x = self.ws_x_fixed
        self.target_y = 0.0  # center sideways
        self.target_z = (self.ws_z_min + self.ws_z_max) / 2.0
        self.emergency_active = False
        self._prev_gripper_state = None
        self._lock = threading.Lock()
        self._servo_started = False
        self._got_first_pose = False

        # ---------- Subscribers ----------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Pose, '/gesture/target_pose',
            self._pose_cb, qos, callback_group=self.cb_group)
        self.create_subscription(
            Bool, '/gesture/gripper_command',
            self._gripper_cb, qos, callback_group=self.cb_group)
        self.create_subscription(
            Bool, '/gesture/emergency_stop',
            self._estop_cb, qos, callback_group=self.cb_group)

        # ---------- Servo twist publisher ----------
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10,
        )

        # ---------- Servo services ----------
        self._start_servo_client = self.create_client(
            Trigger, '/servo_node/start_servo',
            callback_group=self.cb_group)
        self._stop_servo_client = self.create_client(
            Trigger, '/servo_node/stop_servo',
            callback_group=self.cb_group)

        # ---------- Gripper action ----------
        self._gripper_action = ActionClient(
            self, GripperCommandAction,
            '/panda_hand_controller/gripper_cmd',
            callback_group=self.cb_group,
        )

        # ---------- Timer ----------
        period = 1.0 / self.servo_rate
        self.timer = self.create_timer(
            period, self._servo_tick, callback_group=self.cb_group)

        self.create_timer(
            3.0, self._try_start_servo, callback_group=self.cb_group)

        self.get_logger().info(
            f'GestureServoBridge ready — gain={self.velocity_gain}, '
            f'max_speed={self.max_speed} m/s, rate={self.servo_rate} Hz')

    # ==================================================================
    # Start MoveIt Servo
    # ==================================================================
    def _try_start_servo(self, event=None):
        if self._servo_started:
            return
        if not self._start_servo_client.service_is_ready():
            self.get_logger().info(
                'Waiting for /servo_node/start_servo service...')
            return
        req = Trigger.Request()
        future = self._start_servo_client.call_async(req)
        future.add_done_callback(self._start_servo_cb)

    def _start_servo_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self._servo_started = True
                self.get_logger().info('MoveIt Servo started successfully.')
            else:
                self.get_logger().warn(f'Servo start failed: {resp.message}')
        except Exception as e:
            self.get_logger().error(f'Servo start call failed: {e}')

    # ==================================================================
    # Subscriber callbacks
    # ==================================================================
    def _pose_cb(self, msg: Pose):
        with self._lock:
            # X is fixed forward reach
            self.target_x = self.ws_x_fixed
            # Y is sideways (from gesture tracker camera X mapping)
            self.target_y = self._clamp(
                msg.position.y, self.ws_y_min, self.ws_y_max)
            # Z is height (from gesture tracker camera Y mapping)
            self.target_z = self._clamp(
                msg.position.z, self.ws_z_min, self.ws_z_max)
            self._got_first_pose = True

    def _gripper_cb(self, msg: Bool):
        close = msg.data
        if close != self._prev_gripper_state:
            self._prev_gripper_state = close
            self._send_gripper_command(close)

    def _estop_cb(self, msg: Bool):
        if msg.data and not self.emergency_active:
            self.get_logger().warn('EMERGENCY STOP activated!')
            self.emergency_active = True
            self._publish_zero_twist()
        elif not msg.data and self.emergency_active:
            self.get_logger().info('Emergency stop released.')
            self.emergency_active = False

    # ==================================================================
    # Servo tick — position-tracking via TF feedback
    # ==================================================================
    def _servo_tick(self):
        if self.emergency_active or not self._servo_started:
            return
        if not self._got_first_pose:
            return

        # --- Look up current EE position ---
        try:
            t = self.tf_buffer.lookup_transform(
                self.planning_frame, self.ee_frame,
                rclpy.time.Time())
        except Exception:
            return

        current_x = t.transform.translation.x
        current_y = t.transform.translation.y
        current_z = t.transform.translation.z

        with self._lock:
            tx, ty, tz = self.target_x, self.target_y, self.target_z

        # --- Position error → velocity ---
        ex = tx - current_x
        ey = ty - current_y
        ez = tz - current_z

        vx = self.velocity_gain * ex
        vy = self.velocity_gain * ey
        vz = self.velocity_gain * ez

        # --- Deadzone ---
        if abs(ex) < self.pos_tolerance:
            vx = 0.0
        if abs(ey) < self.pos_tolerance:
            vy = 0.0
        if abs(ez) < self.pos_tolerance:
            vz = 0.0

        # --- Clamp ---
        vx = self._clamp(vx, -self.max_speed, self.max_speed)
        vy = self._clamp(vy, -self.max_speed, self.max_speed)
        vz = self._clamp(vz, -self.max_speed, self.max_speed)

        # --- Publish ---
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.planning_frame
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.twist_pub.publish(twist)

    def _publish_zero_twist(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.planning_frame
        self.twist_pub.publish(twist)

    # ==================================================================
    # Gripper
    # ==================================================================
    def _send_gripper_command(self, close: bool):
        if not self._gripper_action.server_is_ready():
            self.get_logger().warn('Gripper action server not ready.')
            return
        goal = GripperCommandAction.Goal()
        goal.command.position = self.gripper_close if close else self.gripper_open
        goal.command.max_effort = 50.0
        self._gripper_action.send_goal_async(goal)
        self.get_logger().info(
            f'Gripper {"CLOSE" if close else "OPEN"} command sent.')

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))


def main(args=None):
    rclpy.init(args=args)
    node = GestureServoBridge()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
