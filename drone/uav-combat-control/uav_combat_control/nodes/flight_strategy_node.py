#!/usr/bin/env python3

# 启动流程（本地与线上通用）
# 1. 准备：双方策略与飞控等开赛条件连续就绪 10 个现实秒。
# 2. 开赛：本地点击“开始比赛”，线上自动触发；裁判统一发布 /game_start=true。
# 3. 起飞：确认 OFFBOARD → 预检通过后请求 ARM → 确认解锁 → 爬升。
# 4. 比赛：达到目标高度后，执行 compute_strategy_setpoint() 中的行为。
#
# 时间：起飞期限从开赛通知起算，赛前等待不计入，重试也不会延长期限。
#       准备/重试用现实时间，回放用仿真时间；准备画面不一定正好 10 秒。
#
# 修改：未完全理解流程时保留启动代码，优先修改 compute_strategy_setpoint() 和 behaviors/。
#       完全理解后可自改，但须验证本地、线上及蓝/黄双方；自改造成不起飞、延迟解锁等，自行负责。
#       AI 修改须保留开赛约束、状态确认、心跳和限频重试，不能用调大 depth 替代状态确认。
#       平台与通信故障仍按运行证据判断。

import math
import subprocess
import sys

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
from std_msgs.msg import Bool, String

from uav_combat_control.behaviors import (
    OrbitBehavior,
    ScanBehavior,
    orbit_duration_for_laps,
)
from uav_combat_control.behaviors.lap_race import LapRaceBehavior
from uav_combat_control.behaviors.common import (
    FlightTarget,
    enu_to_ned_position,
    enu_yaw_to_ned_yaw,
    get_yaw_from_quaternion,
    team_id_from_namespace,
)
from uav_combat_control.behaviors.tracking import EnemyTracker


class FlightStrategyNode(Node):
    def __init__(self):
        super().__init__(
            'flight_strategy_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.strategy_mode = str(
            self.declare_parameter('strategy_mode', 'lap_race').value)
        self.target_altitude = float(
            self.declare_parameter('altitude', 1.35).value)
        self.horizontal_speed = float(
            self.declare_parameter('horizontal_velocity', 1.0).value)
        self.circle_radius = float(self.declare_parameter('radius', 1.3).value)
        self.orientation_mode = int(
            self.declare_parameter('orientation_mode', 1).value)
        self.orbit_laps = float(
            self.declare_parameter('orbit_laps', 0.25).value)
        self.takeoff_hold_sec = float(
            self.declare_parameter('takeoff_hold_sec', 0.5).value)
        self.scan_role_duration_sec = float(
            self.declare_parameter('scan_role_duration_sec', 45.0).value)
        self.scan_follow_distance = float(
            self.declare_parameter('scan_follow_distance', 1.0).value)
        self.scan_approach_velocity = float(
            self.declare_parameter('scan_approach_velocity', 1.0).value)
        self.minimum_opponent_clearance = float(
            self.declare_parameter(
                'scan_minimum_opponent_clearance', 0.85).value)
        self.opponent_slowdown_distance = float(
            self.declare_parameter(
                'scan_opponent_slowdown_distance', 1.6).value)
        self.obstacle_clearance_radius = float(
            self.declare_parameter(
                'center_obstacle_clearance_radius', 0.8).value)
        self.obstacle_detour_offset = float(
            self.declare_parameter(
                'center_obstacle_detour_offset', 1.1).value)
        self.scan_waypoint_tolerance = float(
            self.declare_parameter('scan_waypoint_tolerance', 0.15).value)
        self.enemy_prediction_horizon = float(
            self.declare_parameter('enemy_prediction_horizon', 0.5).value)
        self.enemy_pose_timeout = float(
            self.declare_parameter('enemy_pose_timeout', 0.6).value)
        self.enemy_velocity_smoothing = float(
            self.declare_parameter('enemy_velocity_smoothing', 0.45).value)
        self.enemy_max_speed = float(
            self.declare_parameter('enemy_max_speed', 2.0).value)
        self.field_half_x = float(
            self.declare_parameter('field_half_x', 4.5).value)
        self.field_half_y = float(
            self.declare_parameter('field_half_y', 3.0).value)
        self.field_margin = float(
            self.declare_parameter('field_margin', 0.6).value)
        self.post_submit_hold_sec = float(
            self.declare_parameter('post_submit_hold_sec', 10.0).value)
        self.scan_timeout_sec = float(
            self.declare_parameter('scan_timeout_sec', 20.0).value)
        self.minimum_altitude = float(
            self.declare_parameter('minimum_altitude', 1.3).value)
        self.maximum_altitude = float(
            self.declare_parameter('maximum_altitude', 2.5).value)
        self.lap_high_altitude = float(
            self.declare_parameter('lap_high_altitude', 2.3).value)
        self.scan_route_refresh_sec = float(
            self.declare_parameter('scan_route_refresh_sec', 0.8).value)
        self.scan_route_refresh_distance = float(
            self.declare_parameter('scan_route_refresh_distance', 0.35).value)
        self.collision_avoid_distance = float(
            self.declare_parameter('collision_avoid_distance', 0.55).value)
        self.validate_parameters()
        self.orbit_duration_sec = orbit_duration_for_laps(
            circle_radius=self.circle_radius,
            horizontal_speed=self.horizontal_speed,
            orbit_laps=self.orbit_laps,
        )
        self.team_id = team_id_from_namespace(self.get_namespace())
        self.orbit_behavior = OrbitBehavior(
            team_id=self.team_id,
            target_altitude=self.target_altitude,
            circle_radius=self.circle_radius,
            horizontal_speed=self.horizontal_speed,
            orientation_mode=self.orientation_mode,
        )
        self.lap_race_behavior = LapRaceBehavior(
            team_id=self.team_id,
            circle_radius=self.circle_radius,
            horizontal_speed=self.horizontal_speed,
            low_altitude=self.target_altitude,
            high_altitude=self.lap_high_altitude,
        )
        self.scan_behavior = ScanBehavior(
            team_id=self.team_id,
            target_altitude=self.target_altitude,
            role_duration_sec=self.scan_role_duration_sec,
            follow_distance=self.scan_follow_distance,
            approach_speed=self.scan_approach_velocity,
            obstacle_clearance_radius=self.obstacle_clearance_radius,
            obstacle_detour_offset=self.obstacle_detour_offset,
            waypoint_tolerance=self.scan_waypoint_tolerance,
            minimum_opponent_clearance=self.minimum_opponent_clearance,
            opponent_slowdown_distance=self.opponent_slowdown_distance,
            field_half_x=self.field_half_x,
            field_half_y=self.field_half_y,
            field_margin=self.field_margin,
            route_refresh_sec=self.scan_route_refresh_sec,
            route_refresh_distance=self.scan_route_refresh_distance,
        )
        self.enemy_tracker = EnemyTracker(
            smoothing=self.enemy_velocity_smoothing,
            max_speed=self.enemy_max_speed,
            timeout=self.enemy_pose_timeout,
            horizon=self.enemy_prediction_horizon,
            field_half_x=self.field_half_x,
            field_half_y=self.field_half_y,
            field_margin=self.field_margin,
        )

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
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.status_callback,
            px4_qos,
        )
        self.local_pose_subscription = self.create_subscription(
            Pose,
            'local_pose',
            self.local_pose_callback,
            qos_profile_sensor_data,
        )
        self.enemy_pose_subscription = self.create_subscription(
            Pose,
            'enemy_pose',
            self.enemy_pose_callback,
            qos_profile_sensor_data,
        )
        self.game_start_subscription = self.create_subscription(
            Bool, '/game_start', self.game_start_callback, 10)
        self.submit_subscription = self.create_subscription(
            String, 'qr_submit', self.submit_callback, 10)

        self.game_started = False
        self.game_finished = False
        self.armed = False
        self.offboard = False
        self.status = None
        self.status_received_at = None
        self.pose_received_at = None
        self.current_pose = None
        self.enemy_pose = None
        self.game_start_time = None
        self.behavior_start_time = None
        self.takeoff_reference = None
        self.current_scanner_team = None
        self.current_scan_phase = None
        self.heartbeat_started_at = None
        self.last_command_at = None
        self.command_retry_sec = 0.25
        self.feedback_timeout_sec = 3.0
        self.active_announced = False
        self.last_submit_at = None
        self.last_submit_payload = None
        self.last_phase_log_at = None
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.05,
            self.timer_callback,
            clock=self.wall_clock,
        )
        self.get_logger().info(
            f'FlightStrategyNode started for team {self.team_id} '
            f'in {self.strategy_mode} mode.')

    def validate_parameters(self):
        if self.strategy_mode not in (
                'scan_only', 'orbit_then_scan', 'pursuit', 'lap_race'):
            raise ValueError(
                'strategy_mode must be scan_only, orbit_then_scan, '
                'pursuit, or lap_race')
        positive_values = {
            'altitude': self.target_altitude,
            'horizontal_velocity': self.horizontal_speed,
            'radius': self.circle_radius,
            'orbit_laps': self.orbit_laps,
            'scan_role_duration_sec': self.scan_role_duration_sec,
            'scan_follow_distance': self.scan_follow_distance,
            'scan_approach_velocity': self.scan_approach_velocity,
            'scan_minimum_opponent_clearance': (
                self.minimum_opponent_clearance),
            'scan_opponent_slowdown_distance': (
                self.opponent_slowdown_distance),
            'center_obstacle_clearance_radius': (
                self.obstacle_clearance_radius),
            'center_obstacle_detour_offset': self.obstacle_detour_offset,
            'scan_waypoint_tolerance': self.scan_waypoint_tolerance,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.obstacle_detour_offset <= self.obstacle_clearance_radius:
            raise ValueError(
                'center_obstacle_detour_offset must exceed clearance radius')
        if (
            self.opponent_slowdown_distance
            <= self.minimum_opponent_clearance
        ):
            raise ValueError(
                'scan_opponent_slowdown_distance must exceed '
                'scan_minimum_opponent_clearance')
        if self.takeoff_hold_sec < 0.0:
            raise ValueError('takeoff_hold_sec must be non-negative')
        if not 0.0 <= self.enemy_velocity_smoothing <= 1.0:
            raise ValueError('enemy velocity smoothing must be in [0, 1]')
        if self.enemy_pose_timeout <= 0.0:
            raise ValueError('enemy pose timeout must be positive')
        if self.enemy_max_speed <= 0.0:
            raise ValueError('enemy max speed must be positive')
        if self.enemy_prediction_horizon < 0.0:
            raise ValueError('enemy prediction horizon must be non-negative')
        if self.field_margin < 0.0:
            raise ValueError('field margin must be non-negative')
        if (self.field_half_x <= self.field_margin
                or self.field_half_y <= self.field_margin):
            raise ValueError('field margin exceeds field bounds')
        if self.minimum_altitude <= 0.0:
            raise ValueError('minimum altitude must be positive')
        if self.maximum_altitude <= self.minimum_altitude:
            raise ValueError('maximum altitude must exceed minimum altitude')
        if self.scan_route_refresh_sec <= 0.0:
            raise ValueError('scan route refresh period must be positive')
        if self.scan_route_refresh_distance < 0.0:
            raise ValueError('scan route refresh distance must be non-negative')
        if self.collision_avoid_distance <= 0.0:
            raise ValueError('collision avoid distance must be positive')
        if self.post_submit_hold_sec < 0.0:
            raise ValueError('post submit hold must be non-negative')
        if self.scan_timeout_sec <= 0.0:
            raise ValueError('scan timeout must be positive')

    def status_callback(self, message):
        # 重复或倒退的飞控时间戳，不能刷新反馈有效期。
        if self.status is not None and message.timestamp <= self.status.timestamp:
            return
        self.status = message
        self.status_received_at = self.steady_seconds()
        self.armed = (
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def local_pose_callback(self, message):
        self.current_pose = message
        self.pose_received_at = self.steady_seconds()

    def enemy_pose_callback(self, message):
        self.enemy_pose = message
        self.enemy_tracker.update(message, self.steady_seconds())

    def game_start_callback(self, message):
        # 只订阅 /game_start：true 开赛，开赛后的 false 结束；策略不得自行发布。
        # 重复 true 不重置流程；比赛结束后，重新加载策略再参赛。
        if self.game_finished:
            return
        if message.data and not self.game_started:
            self.get_logger().info('Game started.')
            self.game_start_time = self.get_clock().now()
            self.last_submit_at = None
            self.last_submit_payload = None
            self.last_phase_log_at = None
            self.reset_behaviors()
        elif not message.data and self.game_started:
            self.get_logger().info('Game stopped.')
            self.active_announced = False
            self.game_finished = True
            self.reset_behaviors()
        self.game_started = message.data

    def submit_callback(self, message):
        self.last_submit_at = self.steady_seconds()
        self.last_submit_payload = message.data
        elapsed = 0.0
        if self.game_start_time is not None:
            elapsed = (
                self.get_clock().now() - self.game_start_time
            ).nanoseconds / 1e9
        self.get_logger().info(
            f'QR submitted: {message.data} elapsed={elapsed:.2f}s')

    def reset_behaviors(self):
        self.scan_behavior.reset()
        self.enemy_tracker.reset()
        self.current_scanner_team = None
        self.current_scan_phase = None

    def timer_callback(self):
        # 心跳/目标持续发送；OFFBOARD/ARM 请求必须等开赛通知。
        self.publish_offboard_mode()
        now = self.steady_seconds()
        if not self.feedback_is_fresh(self.pose_received_at, now):
            self.publish_setpoint(None)
            return
        if not self.game_started:
            self.publish_setpoint(self.get_hover_setpoint())
            return
        if self.takeoff_reference is None:
            position = self.current_pose.position
            orientation = self.current_pose.orientation
            values = (position.x, position.y, position.z,
                      orientation.x, orientation.y, orientation.z, orientation.w)
            if not all(math.isfinite(value) for value in values):
                self.publish_setpoint(None)
                return
            self.takeoff_reference = (
                position.x, position.y,
                get_yaw_from_quaternion(*values[3:]),
            )
        if (not self.feedback_is_fresh(self.status_received_at, now)
                or self.heartbeat_started_at is None
                or now - self.heartbeat_started_at < 1.0):
            # 状态须在有效期内、心跳须预热至少 1 个现实秒；不足时只维持控制流。
            self.publish_setpoint(self.get_hover_setpoint())
            return
        command = None
        if not self.offboard:
            command = (VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif not self.armed and self.status.pre_flight_checks_pass:
            command = (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        if command is not None:
            # 所有启动命令间隔至少 0.25 个现实秒，切换阶段也不得连发。
            # 只重试未确认的动作，不因次数耗尽而放弃；收到有效确认后推进下一步。
            if (self.last_command_at is None
                    or now - self.last_command_at >= self.command_retry_sec):
                self.publish_command(*command)
                self.last_command_at = now
                self.get_logger().info(f'Startup request: command={command[0]}')
            self.publish_setpoint(self.get_hover_setpoint())
            return
        if not self.offboard or not self.armed:
            self.publish_setpoint(self.get_hover_setpoint())
            return
        if self.behavior_start_time is None:
            self.behavior_start_time = self.get_clock().now()
        if not self.active_announced:
            self.get_logger().info(
                'Strategy active; climbing and pursuing.')
            self.active_announced = True

        self.publish_setpoint(self.compute_strategy_setpoint())

    def compute_strategy_setpoint(self):
        # 优先在此处修改比赛行为；启动流程按文件头约定处理。
        if self.current_pose is None or self.behavior_start_time is None:
            return None

        elapsed_sec = (
            (self.get_clock().now() - self.behavior_start_time).nanoseconds
            / 1_000_000_000.0
        )
        if elapsed_sec < self.takeoff_hold_sec:
            return self.get_hover_setpoint()

        flight_elapsed_sec = elapsed_sec - self.takeoff_hold_sec
        now_wall = self.steady_seconds()
        enemy_fresh = self.enemy_tracker.is_fresh(now_wall)

        if (
            self.last_submit_at is not None
            and now_wall - self.last_submit_at < self.post_submit_hold_sec
        ):
            return self.get_safe_hold_setpoint()

        if self.strategy_mode == 'lap_race':
            enemy_altitude = None
            if enemy_fresh:
                predicted = self.enemy_tracker.predicted(now_wall)
                if predicted is not None:
                    enemy_altitude = predicted[0][2]
            return self.target_to_setpoint(
                self.lap_race_behavior(flight_elapsed_sec, enemy_altitude))

        if (
            self.last_submit_at is None
            and flight_elapsed_sec >= self.scan_timeout_sec
        ):
            return self.target_to_setpoint(
                self.orbit_behavior(flight_elapsed_sec))

        if self.strategy_mode in ('scan_only', 'pursuit'):
            if self.strategy_mode == 'pursuit' and not enemy_fresh:
                return self.target_to_setpoint(
                    self.orbit_behavior(flight_elapsed_sec))
            return self.compute_scan_setpoint(flight_elapsed_sec)
        if enemy_fresh:
            return self.compute_scan_setpoint(flight_elapsed_sec)
        if flight_elapsed_sec < self.orbit_duration_sec:
            return self.target_to_setpoint(
                self.orbit_behavior(flight_elapsed_sec))
        return self.compute_scan_setpoint(
            flight_elapsed_sec - self.orbit_duration_sec)

    def compute_scan_setpoint(self, scan_elapsed_sec):
        current = self.current_pose.position
        enemy_position = None
        enemy_orientation = None
        predicted = self.enemy_tracker.predicted(self.steady_seconds())
        enemy_velocity = None
        if predicted is not None:
            enemy_position, enemy_orientation = predicted
            enemy_velocity = self.enemy_tracker.velocity

        output = self.scan_behavior(
            scan_elapsed_sec=scan_elapsed_sec,
            current_position=(current.x, current.y, current.z),
            enemy_position=enemy_position,
            enemy_orientation=enemy_orientation,
            enemy_velocity=enemy_velocity,
        )
        if output.scanner_team != self.current_scanner_team:
            self.current_scanner_team = output.scanner_team
            self.get_logger().info(
                'Scan role active: '
                f'team {output.scanner_team} is the scanner.')
        if output.phase != self.current_scan_phase:
            self.current_scan_phase = output.phase
            now_wall = self.steady_seconds()
            if (
                self.last_phase_log_at is None
                or now_wall - self.last_phase_log_at >= 0.5
            ):
                self.get_logger().info(f'Scan phase: {output.phase}.')
                self.last_phase_log_at = now_wall
        if output.target is None:
            return self.get_hover_setpoint()
        return self.target_to_setpoint(output.target)

    def clamp_flight_target(self, target):
        x_enu = target.position_ned[1]
        y_enu = target.position_ned[0]
        z_enu = -target.position_ned[2]
        x_enu = max(
            -self.field_half_x + self.field_margin,
            min(self.field_half_x - self.field_margin, x_enu),
        )
        y_enu = max(
            -self.field_half_y + self.field_margin,
            min(self.field_half_y - self.field_margin, y_enu),
        )
        z_enu = max(
            self.minimum_altitude,
            min(self.maximum_altitude, z_enu),
        )
        return FlightTarget(
            position_ned=enu_to_ned_position(x_enu, y_enu, z_enu),
            yaw_ned=target.yaw_ned,
            velocity_ned=target.velocity_ned,
        )

    def target_to_setpoint(self, target):
        target = self.clamp_flight_target(target)
        now = self.steady_seconds()
        if (
            self.current_pose is not None
            and self.enemy_tracker.is_fresh(now)
            and self.enemy_tracker.position is not None
        ):
            current = self.current_pose.position
            enemy = self.enemy_tracker.position
            delta_x = current.x - enemy[0]
            delta_y = current.y - enemy[1]
            distance = math.hypot(delta_x, delta_y)
            if 1e-6 < distance < self.collision_avoid_distance:
                away_x = delta_x / distance
                away_y = delta_y / distance
                escape_x = current.x + away_x * 0.45
                escape_y = current.y + away_y * 0.45
                escape_x = max(
                    -self.field_half_x + self.field_margin,
                    min(self.field_half_x - self.field_margin, escape_x),
                )
                escape_y = max(
                    -self.field_half_y + self.field_margin,
                    min(self.field_half_y - self.field_margin, escape_y),
                )
                velocity_ned = (
                    away_y * self.horizontal_speed,
                    away_x * self.horizontal_speed,
                    0.0,
                )
                return self.new_setpoint(
                    enu_to_ned_position(
                        escape_x,
                        escape_y,
                        self.target_altitude,
                    ),
                    target.yaw_ned,
                    velocity_ned,
                )
        return self.new_setpoint(
            target.position_ned,
            target.yaw_ned,
            target.velocity_ned,
        )

    def get_hover_setpoint(self):
        if self.current_pose is None:
            return None
        position = self.current_pose.position
        orientation = self.current_pose.orientation
        x, y = position.x, position.y
        yaw_enu = get_yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)
        if self.takeoff_reference is not None:
            x, y, yaw_enu = self.takeoff_reference
        position_ned = enu_to_ned_position(
            x,
            y,
            self.target_altitude,
        )
        return self.new_setpoint(
            position_ned,
            enu_yaw_to_ned_yaw(yaw_enu),
        )

    def get_safe_hold_setpoint(self):
        if self.current_pose is None:
            return None
        position = self.current_pose.position
        target_x = position.x
        target_y = position.y
        now_wall = self.steady_seconds()
        if (
            self.enemy_tracker.is_fresh(now_wall)
            and self.enemy_tracker.position is not None
        ):
            enemy = self.enemy_tracker.position
            delta_x = target_x - enemy[0]
            delta_y = target_y - enemy[1]
            distance = math.hypot(delta_x, delta_y)
            safe_distance = max(
                self.scan_follow_distance,
                self.minimum_opponent_clearance + 0.3,
            )
            if 1e-6 < distance < safe_distance:
                target_x = enemy[0] + delta_x / distance * safe_distance
                target_y = enemy[1] + delta_y / distance * safe_distance
        target_x = max(
            -self.field_half_x + self.field_margin,
            min(self.field_half_x - self.field_margin, target_x),
        )
        target_y = max(
            -self.field_half_y + self.field_margin,
            min(self.field_half_y - self.field_margin, target_y),
        )
        orientation = self.current_pose.orientation
        yaw_enu = get_yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        return self.new_setpoint(
            enu_to_ned_position(target_x, target_y, self.target_altitude),
            enu_yaw_to_ned_yaw(yaw_enu),
        )

    def get_current_hover_setpoint(self):
        if self.current_pose is None:
            return None
        position = self.current_pose.position
        orientation = self.current_pose.orientation
        target_x = max(
            -self.field_half_x + self.field_margin,
            min(self.field_half_x - self.field_margin, position.x),
        )
        target_y = max(
            -self.field_half_y + self.field_margin,
            min(self.field_half_y - self.field_margin, position.y),
        )
        yaw_enu = get_yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        return self.new_setpoint(
            enu_to_ned_position(target_x, target_y, self.target_altitude),
            enu_yaw_to_ned_yaw(yaw_enu),
        )

    def new_setpoint(self, position=None, yaw=math.nan, velocity=None):
        message = TrajectorySetpoint()
        message.position = (
            [math.nan, math.nan, math.nan]
            if position is None
            else [float(value) for value in position]
        )
        message.velocity = (
            [math.nan, math.nan, math.nan]
            if velocity is None
            else [float(value) for value in velocity]
        )
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = float(yaw)
        message.yawspeed = math.nan
        message.timestamp = self.timestamp()
        return message

    def publish_offboard_mode(self):
        if self.heartbeat_started_at is None:
            self.heartbeat_started_at = self.steady_seconds()
        message = OffboardControlMode()
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.timestamp = self.timestamp()
        self.offboard_publisher.publish(message)

    def publish_setpoint(self, message):
        if message is None:
            message = self.new_setpoint()
        self.setpoint_publisher.publish(message)

    def publish_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.command = command
        message.param1 = param1
        message.param2 = param2
        # 0/0 为 broadcast，只控制本方通道：蓝 /px4_1，黄 /px4_2，不会同时控制双方。
        # 阵营编号 1/2 不是 PX4 system_id；地址不匹配会被忽略，已确认过此类失败。
        # 保留这两个值；自行改错造成蓝方或黄方不起飞，由修改者负责。
        message.target_system = 0
        message.target_component = 0
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        message.timestamp = self.timestamp()
        self.command_publisher.publish(message)

    def timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def steady_seconds(self):
        return self.wall_clock.now().nanoseconds / 1_000_000_000.0

    def feedback_is_fresh(self, received_at, now):
        return (received_at is not None
                and 0.0 <= now - received_at <= self.feedback_timeout_sec)


def main(args=None):
    rclpy.init(args=args)
    flight_node = FlightStrategyNode()
    qr_process = None
    try:
        namespace = flight_node.get_namespace()
        qr_process = subprocess.Popen([
            sys.executable,
            '-m',
            'uav_combat_control.nodes.qr_scanner_node',
            '--ros-args',
            '-r',
            f'__ns:={namespace}',
        ])
    except Exception as error:
        flight_node.get_logger().error(
            f'Unable to start QR scanner process: {error}')

    try:
        rclpy.spin(flight_node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        flight_node.destroy_node()
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
