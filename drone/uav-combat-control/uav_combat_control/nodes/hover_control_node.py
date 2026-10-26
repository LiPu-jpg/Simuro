#!/usr/bin/env python3

import math

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class HoverControlNode(Node):
    def __init__(self):
        super().__init__('hover_control_node')
        self.hover_height = self.declare_parameter('hover_height', 1.3).value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', qos)
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos)
        self.command_publisher = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', qos)
        self.status_subscription = self.create_subscription(
            VehicleStatus, 'fmu/out/vehicle_status_v1', self.status_callback, qos)
        self.local_position_subscription = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos,
        )
        self.game_start_subscription = self.create_subscription(
            Bool, '/game_start', self.game_start_callback, 10)
        self.status = None
        self.local_position = None
        self.takeoff_position = None
        self.takeoff_yaw = None
        self.game_started = False
        self.setpoint_count = 0
        self.retry_count = 0
        self.active_announced = False
        self.timer = self.create_timer(0.1, self.timer_callback)

    def status_callback(self, message):
        self.status = message

    def local_position_callback(self, message):
        self.local_position = message
        if self.game_started and self.takeoff_position is None:
            self.capture_takeoff_reference()

    def game_start_callback(self, message):
        if message.data and not self.game_started:
            self.get_logger().info('Game started')
            self.capture_takeoff_reference()
        self.game_started = message.data

    def local_position_is_valid(self):
        if self.local_position is None:
            return False
        return (
            self.local_position.xy_valid
            and self.local_position.z_valid
            and self.local_position.heading_good_for_control
            and all(math.isfinite(value) for value in (
                self.local_position.x,
                self.local_position.y,
                self.local_position.z,
                self.local_position.heading,
            ))
        )

    def capture_takeoff_reference(self):
        if not self.local_position_is_valid():
            return
        self.takeoff_position = (
            float(self.local_position.x),
            float(self.local_position.y),
            float(self.local_position.z),
        )
        self.takeoff_yaw = float(self.local_position.heading)

    def timer_callback(self):
        self.publish_offboard_mode()
        self.publish_setpoint()

        if self.setpoint_count < 10:
            self.setpoint_count += 1
        if not self.game_started or self.setpoint_count < 10:
            return
        if self.takeoff_position is None:
            self.capture_takeoff_reference()
            return
        if self.status is None:
            return

        offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if offboard and armed:
            if not self.active_announced:
                self.get_logger().info(
                    f'Offboard active; target height {self.hover_height:.2f} m'
                )
                self.active_announced = True
            self.retry_count = 0
            return

        self.active_announced = False
        self.retry_count += 1
        if self.retry_count < 10:
            return
        self.retry_count = 0
        if not offboard:
            self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if not armed:
            self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def publish_offboard_mode(self):
        message = OffboardControlMode()
        message.position = True
        message.timestamp = self.timestamp()
        self.offboard_publisher.publish(message)

    def publish_setpoint(self):
        message = TrajectorySetpoint()
        if self.takeoff_position is not None:
            x, y, z = self.takeoff_position
            message.position = [x, y, z - float(self.hover_height)]
            message.yaw = self.takeoff_yaw
        elif self.local_position_is_valid():
            message.position = [
                float(self.local_position.x),
                float(self.local_position.y),
                float(self.local_position.z),
            ]
            message.yaw = float(self.local_position.heading)
        else:
            message.position = [math.nan, math.nan, math.nan]
            message.velocity = [math.nan, math.nan, math.nan]
            message.acceleration = [math.nan, math.nan, math.nan]
            message.jerk = [math.nan, math.nan, math.nan]
            message.yaw = math.nan
            message.yawspeed = math.nan
        message.timestamp = self.timestamp()
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


def main(args=None):
    rclpy.init(args=args)
    node = HoverControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
