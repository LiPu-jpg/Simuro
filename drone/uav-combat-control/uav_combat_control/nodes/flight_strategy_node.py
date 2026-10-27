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

from uav_combat_control.guidance import (
    CenterGuard,
    DuelPolicy,
    OrbitPath,
    SafetyEnvelope,
)


def quaternion_heading(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z),
    )


class DuelRaceNode(Node):
    def __init__(self):
        super().__init__(
            'flight_strategy_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        base_altitude = float(
            self.declare_parameter('base_altitude', 1.55).value)
        takeoff_altitude = float(
            self.declare_parameter('takeoff_altitude', 1.75).value)
        takeoff_threshold = float(
            self.declare_parameter('takeoff_threshold', 1.4).value)
        radius = float(self.declare_parameter('orbit_radius', 1.4).value)
        speed = float(self.declare_parameter('orbit_speed', 1.0).value)
        field_half_x = float(
            self.declare_parameter('field_half_x', 4.5).value)
        field_half_y = float(
            self.declare_parameter('field_half_y', 3.0).value)
        field_margin = float(
            self.declare_parameter('field_margin', 0.6).value)
        floor_altitude = float(
            self.declare_parameter('flight_floor', 1.3).value)
        recovery_altitude = float(
            self.declare_parameter('recovery_altitude', 1.75).value)
        center_clearance = float(
            self.declare_parameter('center_clearance', 1.4).value)
        center_escape = float(
            self.declare_parameter('center_escape_radius', 2.0).value)
        enemy_clearance = float(
            self.declare_parameter('enemy_clearance', 0.85).value)
        enemy_vertical_range = float(
            self.declare_parameter('enemy_vertical_range', 0.45).value)
        engage_distance = float(
            self.declare_parameter('engage_distance', 2.5).value)
        defense_vertical = float(
            self.declare_parameter('defense_vertical', 0.6).value)
        enemy_timeout = float(
            self.declare_parameter('enemy_pose_timeout', 0.6).value)

        if not (floor_altitude < base_altitude < takeoff_altitude):
            raise ValueError('altitude ordering is invalid')

        namespace = self.get_namespace().strip('/')
        team_id = 2 if namespace.endswith('2') else 1
        self.base_altitude = base_altitude
        self.takeoff_altitude = takeoff_altitude
        self.takeoff_threshold = takeoff_threshold
        self.enemy_timeout = enemy_timeout
        self.phase = 'ground'
        self.game_started = False
        self.armed = False
        self.offboard = False
        self.status = None
        self.status_at = None
        self.pose = None
        self.pose_at = None
        self.enemy = None
        self.enemy_at = None
        self.heading = 0.0
        self.heartbeat_at = None
        self.command_at = None
        self.race_started = None
        self.race_announced = False
        self.game_start_at = None
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.orbit = OrbitPath(
            team_id=team_id,
            radius=radius,
            speed=speed,
            field_half_x=field_half_x,
            field_half_y=field_half_y,
            margin=field_margin,
        )
        self.safety = SafetyEnvelope(
            field_half_x=field_half_x,
            field_half_y=field_half_y,
            field_margin=field_margin,
            flight_floor=floor_altitude,
            recovery_altitude=recovery_altitude,
            center_clearance=center_clearance,
            enemy_clearance=enemy_clearance,
            enemy_vertical_range=enemy_vertical_range,
        )
        self.center_guard = CenterGuard(
            trigger_radius=center_clearance,
            escape_radius=center_escape,
        )
        self.duel = DuelPolicy(
            engage_distance=engage_distance,
            vertical_threshold=defense_vertical,
            climb_altitude=base_altitude,
            maximum_climb=base_altitude,
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', qos)
        self.create_subscription(
            VehicleStatus, 'fmu/out/vehicle_status_v1', self.on_status, qos)
        self.create_subscription(
            Pose, 'local_pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(
            Pose, 'enemy_pose', self.on_enemy, qos_profile_sensor_data)
        self.create_subscription(Bool, '/game_start', self.on_start, 10)
        self.timer = self.create_timer(0.05, self.on_timer, clock=self.wall_clock)
        self.get_logger().info(f'DuelRaceNode ready for team {team_id}.')

    def on_status(self, message):
        self.status = message
        self.status_at = time.monotonic()
        self.armed = (
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def on_pose(self, message):
        self.pose = message
        self.pose_at = time.monotonic()
        if self.race_started is None:
            self.heading = quaternion_heading(message.orientation)

    def on_enemy(self, message):
        self.enemy = message
        self.enemy_at = time.monotonic()

    def on_start(self, message):
        if message.data and not self.game_started:
            self.game_started = True
            self.phase = 'ground'
            self.race_started = None
            self.race_announced = False
            self.game_start_at = time.monotonic()
            self.get_logger().info('Game started.')
        elif not message.data and self.game_started:
            self.game_started = False
            self.phase = 'ground'
            self.race_started = None
            self.race_announced = False
            self.game_start_at = None
            self.get_logger().info('Game stopped.')

    def on_timer(self):
        self.send_heartbeat()
        now = time.monotonic()
        if not self.fresh(self.pose_at, now) or self.pose is None:
            self.publish(None)
            return
        if not self.game_started:
            self.publish(self.hold_target(self.base_altitude))
            return
        if (not self.fresh(self.status_at, now)
                or self.heartbeat_at is None
                or now - self.heartbeat_at < 1.0):
            self.publish(self.hold_target(self.base_altitude))
            return
        if self.phase == 'ground':
            if self.issue_start(now):
                return
        if self.phase == 'climb':
            self.climb()
            return
        self.race(now)

    def issue_start(self, now):
        command = None
        if not self.offboard:
            command = (VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif not self.armed and self.status.pre_flight_checks_pass:
            command = (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        if command is not None:
            if self.command_at is None or now - self.command_at >= 0.25:
                self.send_command(*command)
                self.command_at = now
                self.get_logger().info(f'Startup request: {command[0]}')
            self.publish(self.hold_target(self.base_altitude))
            return True
        if self.offboard and self.armed:
            self.phase = 'climb'
            return False
        self.publish(self.hold_target(self.base_altitude))
        return True

    def climb(self):
        position = self.pose.position
        self.publish(self.make_setpoint(
            position.x, position.y, self.takeoff_altitude, self.heading))
        if position.z >= self.takeoff_threshold:
            self.phase = 'race'
            self.race_started = time.monotonic()
            if not self.race_announced:
                self.get_logger().info('Duel race active.')
                self.race_announced = True

    def race(self, now):
        if self.race_started is None:
            self.race_started = now
        current = self.pose.position
        enemy = self.current_enemy(now)
        own = (current.x, current.y, current.z)
        recovery = self.center_guard.escape_target(current.x, current.y)
        if recovery is not None:
            self.publish(self.make_setpoint(
                recovery[0], recovery[1], self.safety.recovery_altitude,
                self.heading))
            return
        x, y, orbit_yaw = self.orbit.point(now - self.race_started)
        x, y = self.safety.avoid_enemy((x, y), own, enemy)
        x, y = self.safety.clamp(x, y)
        target_altitude = self.safety.altitude_recovery(
            current.z, self.base_altitude)
        yaw = self.duel.yaw(own, enemy)
        if yaw is None:
            yaw = orbit_yaw
        self.publish(self.make_setpoint(x, y, target_altitude, yaw))

    def current_enemy(self, now):
        if self.enemy is None or not self.fresh(self.enemy_at, now):
            return None
        position = self.enemy.position
        return position.x, position.y, position.z

    def hold_target(self, altitude):
        position = self.pose.position
        return self.make_setpoint(
            position.x, position.y, altitude, self.heading)

    def make_setpoint(self, x, y, z, yaw):
        message = TrajectorySetpoint()
        message.position = [float(x), float(-y), float(-z)]
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = float(-yaw)
        message.yawspeed = math.nan
        message.timestamp = self.stamp()
        return message

    def publish(self, message):
        if message is None:
            message = TrajectorySetpoint()
            message.position = [math.nan, math.nan, math.nan]
            message.velocity = [math.nan, math.nan, math.nan]
            message.acceleration = [math.nan, math.nan, math.nan]
            message.jerk = [math.nan, math.nan, math.nan]
            message.yaw = math.nan
            message.yawspeed = math.nan
            message.timestamp = self.stamp()
        self.setpoint_pub.publish(message)

    def send_heartbeat(self):
        if self.heartbeat_at is None:
            self.heartbeat_at = time.monotonic()
        message = OffboardControlMode()
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.timestamp = self.stamp()
        self.offboard_pub.publish(message)

    def send_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.command = command
        message.param1 = param1
        message.param2 = param2
        message.target_system = 0
        message.target_component = 0
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        message.timestamp = self.stamp()
        self.command_pub.publish(message)

    def stamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def fresh(self, stamp, now):
        return stamp is not None and 0.0 <= now - stamp <= 3.0


def main(args=None):
    rclpy.init(args=args)
    node = DuelRaceNode()
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
