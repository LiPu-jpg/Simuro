#!/usr/bin/env python3

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import ZBarSymbol, decode as decode_barcodes
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from uav_combat_control.qr_scanner import QrSubmissionGate, decode_qr_payloads


class QrScannerNode(Node):
    def __init__(self):
        super().__init__(
            'qr_scanner_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.bridge = CvBridge()
        self.submission_gate = QrSubmissionGate()
        self.submit_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_submit_ns = None
        self.min_submit_interval_ns = 40_000_000
        self.submission_publisher = self.create_publisher(
            String,
            'qr_submit',
            10,
        )
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.game_start_subscription = self.create_subscription(
            Bool,
            '/game_start',
            self.game_start_callback,
            10,
        )
        self.get_logger().info('QrScannerNode started.')

    def game_start_callback(self, message):
        if message.data and not self.submission_gate.active:
            self.last_submit_ns = None
        self.submission_gate.set_game_active(message.data)

    def image_callback(self, message):
        if not self.submission_gate.active:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='mono8',
            )
        except CvBridgeError as error:
            self.get_logger().warning(
                f'Unable to convert camera frame: {error}')
            return

        now_ns = self.submit_clock.now().nanoseconds
        if (
            self.last_submit_ns is not None
            and now_ns - self.last_submit_ns < self.min_submit_interval_ns
        ):
            return

        for payload in decode_qr_payloads(
            frame,
            decode_barcodes,
            ZBarSymbol.QRCODE,
        ):
            accepted_payload = self.submission_gate.accept(payload)
            if accepted_payload is None:
                continue

            submission = String()
            submission.data = accepted_payload
            self.submission_publisher.publish(submission)
            self.last_submit_ns = now_ns
            self.get_logger().info('Submitted a decoded opponent QR payload.')


def main(args=None):
    rclpy.init(args=args)
    node = QrScannerNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
