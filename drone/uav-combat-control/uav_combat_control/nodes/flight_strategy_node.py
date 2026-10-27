#!/usr/bin/env python3
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool


def yaw_from_quaternion(orientation):
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(sin_yaw, cos_yaw)


class LapRaceNode(Node):
    def __init__(self):
        super().__init__(
            'flight_strategy_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.low_altitude = float(
            self.declare_parameter('altitude', 1.35).value)
        self.high_altitude = float(
            self.declare_parameter('lap_high_altitude', 2.3).value)
        self.radius = float(self.declare_parameter('radius', 1.3).value)
        self.speed = float(
            self.declare_parameter('horizontal_velocity', 1.0).value)
        self.takeoff_hold = float(
            self.declare_parameter('takeoff_hold_sec', 0.5).value)
        self.minimum_altitude = float(
            self.declare_parameter('minimum_altitude', 1.3).value)
        self.maximum_altitude = float(
            self.declare_parameter('maximum_altitude', 2.5).value)
        self.field_half_x = float(
            self.declare_parameter('field_half_x', 4.5).value)
        self.field_half_y = float(
            self.declare_parameter('field_half_y', 3.0).value)
        self.field_margin = float(
            self.declare_parameter('field_margin', 0.6).value)
        self.enemy_timeout = float(
            self.declare_parameter('enemy_pose_timeout', 0.6).value)
        self.command_retry = 0.25
        self.feedback_timeout = 3.0

        namespace = self.get_namespace().strip('/')
        self.team_id = 2 if namespace.endswith('2') else 1
        self.phase = math.pi if self.team_id == 1 else 0.0
        self.angular_speed = -self.speed / self.radius

        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', px4_qos)
        self.command_publisher = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', px4_qos)
        self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.status_callback,
            px4_qos,
        )
        self.create_subscription(
            Pose, 'local_pose', self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(
            Pose, 'enemy_pose', self.enemy_callback, qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/game_start', self.start_callback, 10)

        self.started = False
        self.status = None
        self.armed = False
        self.offboard = False
        self.pose = None
        self.posed_at = None
        self.status_at = None
        self.enemy = None
        self.enemy_at = None
        self.takeoff_yaw = 0.0
        self.behavior_start = None
        self.heartbeat_since = None
        self.last_command_at = None
        self.active = False
        self.timer = self.create_timer(0.05, self.tick, clock=self.wall_clock)
        self.get_logger().info(
            f'LapRaceNode started for team {self.team_id}.')

    def status_callback(self, message):
        self.status = message
        self.status_at = time.monotonic()
        self.armed = (
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def pose_callback(self, message):
        self.pose = message
        self.posed_at = time.monotonic()
        if self.behavior_start is None:
            self.takeoff_yaw = yaw_from_quaternion(message.orientation)

    def enemy_callback(self, message):
        self.enemy = message
        self.enemy_at = time.monotonic()

    def start_callback(self, message):
        if message.data and not self.started:
            self.started = True
            self.behavior_start = None
            self.active = False
            self.get_logger().info('Game started.')
        elif not message.data and self.started:
            self.started = False
            self.active = False
            self.behavior_start = None
            self.get_logger().info('Game stopped.')

    def tick(self):
        self.publish_offboard_mode()
        now = time.monotonic()
        if not self.fresh(self.posed_at, now) or self.pose is None:
            self.publish_setpoint(None)
            return
        if not self.started:
            self.publish_setpoint(self.hover_setpoint())
            return
        if (not self.fresh(self.status_at, now)
                or self.heartbeat_since is None
                or now - self.heartbeat_since < 1.0):
            self.publish_setpoint(self.hover_setpoint())
            return
        command = None
        if not self.offboard:
            command = (VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif not self.armed and self.status.pre_flight_checks_pass:
            command = (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        if command is not None:
            if (self.last_command_at is None
                    or now - self.last_command_at >= self.command_retry):
                self.publish_command(*command)
                self.last_command_at = now
                self.get_logger().info(f'Startup request: command={command[0]}')
            self.publish_setpoint(self.hover_setpoint())
            return
        if not self.offboard or not self.armed:
            self.publish_setpoint(self.hover_setpoint())
            return
        if self.behavior_start is None:
            self.behavior_start = now
        if not self.active:
            self.get_logger().info('Lap race active.')
            self.active = True
        elapsed = now - self.behavior_start
        if elapsed < self.takeoff_hold:
            self.publish_setpoint(self.hover_setpoint())
            return
        self.publish_setpoint(self.race_setpoint(elapsed))

    def hover_setpoint(self):
        position = self.pose.position
        return self.new_setpoint(
            position.x,
            position.y,
            self.low_altitude,
            self.takeoff_yaw,
        )

    def race_setpoint(self, elapsed):
        enemy_altitude = None
        now = time.monotonic()
        if self.enemy is not None and self.fresh(self.enemy_at, now):
            enemy_altitude = self.enemy.position.z
        if enemy_altitude is not None and enemy_altitude <= 1.8:
            altitude = self.high_altitude
        else:
            altitude = self.low_altitude
        theta = self.phase + self.angular_speed * elapsed
        x = self.radius * math.cos(theta)
        y = self.radius * math.sin(theta)
        limit_x = self.field_half_x - self.field_margin
        limit_y = self.field_half_y - self.field_margin
        x = max(-limit_x, min(limit_x, x))
        y = max(-limit_y, min(limit_y, y))
        altitude = max(self.minimum_altitude, min(self.maximum_altitude, altitude))
        return self.new_setpoint(x, y, altitude, theta)

    def new_setpoint(self, x, y, z, yaw):
        message = TrajectorySetpoint()
        message.position = [float(x), float(-y), float(-z)]
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = float(-yaw)
        message.yawspeed = math.nan
        message.timestamp = self.timestamp()
        return message

    def publish_setpoint(self, message):
        if message is None:
            message = TrajectorySetpoint()
            message.position = [math.nan, math.nan, math.nan]
            message.velocity = [math.nan, math.nan, math.nan]
            message.acceleration = [math.nan, math.nan, math.nan]
            message.jerk = [math.nan, math.nan, math.nan]
            message.yaw = math.nan
            message.yawspeed = math.nan
            message.timestamp = self.timestamp()
        self.setpoint_publisher.publish(message)

    def publish_offboard_mode(self):
        if self.heartbeat_since is None:
            self.heartbeat_since = time.monotonic()
        message = OffboardControlMode()
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.timestamp = self.timestamp()
        self.offboard_publisher.publish(message)

    def publish_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.command = command
        message.param1 = param1
        message.param2 = param2
        message.target_system = 0
        message.target_component = 0
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        message.timestamp = self.timestamp()
        self.command_publisher.publish(message)

    def timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def fresh(self, stamp, now):
        return stamp is not None and 0.0 <= now - stamp <= self.feedback_timeout


def main(args=None):
    rclpy.init(args=args)
    node = LapRaceNode()
    qr_process = None
    try:
        qr_process = subprocess.Popen([
            sys.executable,
            '-m',
            'uav_combat_control.nodes.qr_scanner_node',
            '--ros-args',
            '-r',
            f'__ns:={node.get_namespace()}',
        ])
    except Exception as error:
        node.get_logger().error(f'Unable to start QR scanner: {error}')
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if qr_process is not None:
            qr_process.terminate()
            try:
                qr_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                qr_process.kill()


if __name__ == '__main__':
    main()
