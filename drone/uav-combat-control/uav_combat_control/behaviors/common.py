import math
from dataclasses import dataclass
from typing import Optional, Tuple


PI = math.pi
PI_2 = PI / 2.0
Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class FlightTarget:
    position_ned: Vector3
    yaw_ned: float
    velocity_ned: Optional[Vector3] = None


def get_yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (x * y + w * z)
    cosy_cosp = w * w + x * x - y * y - z * z
    return math.atan2(siny_cosp, cosy_cosp)


def enu_to_ned_position(x_enu, y_enu, z_enu):
    return y_enu, x_enu, -z_enu


def enu_yaw_to_ned_yaw(yaw_enu):
    return normalize_angle(PI_2 - yaw_enu)


def normalize_angle(angle):
    while angle > PI:
        angle -= 2.0 * PI
    while angle < -PI:
        angle += 2.0 * PI
    return angle


def clockwise_tangent_yaw(x, y):
    return normalize_angle(math.atan2(y, x) - PI_2)


def team_id_from_namespace(namespace):
    normalized = namespace.strip('/')
    if normalized == 'px4_1':
        return 1
    if normalized == 'px4_2':
        return 2
    raise ValueError(
        "controller namespace must be '/px4_1' or '/px4_2', "
        f"got {namespace!r}"
    )
