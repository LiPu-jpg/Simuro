import math

from uav_combat_control.behaviors.common import (
    get_yaw_from_quaternion,
    normalize_angle,
)


def clamp(value, low, high):
    return max(low, min(high, value))


class EnemyTracker:
    def __init__(
        self,
        smoothing=0.45,
        max_speed=2.0,
        max_yaw_rate=4.0,
        timeout=0.6,
        horizon=0.45,
        field_half_x=4.5,
        field_half_y=3.0,
        field_margin=0.3,
    ):
        if not 0.0 <= smoothing <= 1.0:
            raise ValueError('smoothing must be in [0, 1]')
        if max_speed <= 0.0:
            raise ValueError('max speed must be positive')
        if max_yaw_rate <= 0.0:
            raise ValueError('max yaw rate must be positive')
        if timeout <= 0.0:
            raise ValueError('timeout must be positive')
        if horizon < 0.0:
            raise ValueError('horizon must be non-negative')
        if field_margin < 0.0:
            raise ValueError('field margin must be non-negative')
        if field_half_x <= field_margin or field_half_y <= field_margin:
            raise ValueError('field margin exceeds field bounds')

        self.smoothing = smoothing
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.timeout = timeout
        self.horizon = horizon
        self.field_half_x = field_half_x
        self.field_half_y = field_half_y
        self.field_margin = field_margin
        self.reset()

    def reset(self):
        self.received_at = None
        self.last_time = None
        self.position = None
        self.yaw = None
        self.velocity = (0.0, 0.0)
        self.yaw_rate = 0.0

    def update(self, pose, now):
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        yaw = get_yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            return

        if self.last_time is not None and self.position is not None:
            delta_time = now - self.last_time
            if 1e-4 < delta_time < 0.5:
                velocity_x = (x - self.position[0]) / delta_time
                velocity_y = (y - self.position[1]) / delta_time
                speed = math.hypot(velocity_x, velocity_y)
                if speed > self.max_speed:
                    scale = self.max_speed / speed
                    velocity_x *= scale
                    velocity_y *= scale
                yaw_rate = normalize_angle(yaw - self.yaw) / delta_time
                yaw_rate = clamp(yaw_rate, -self.max_yaw_rate,
                                 self.max_yaw_rate)
                alpha = self.smoothing
                self.velocity = (
                    (1.0 - alpha) * self.velocity[0] + alpha * velocity_x,
                    (1.0 - alpha) * self.velocity[1] + alpha * velocity_y,
                )
                self.yaw_rate = (
                    (1.0 - alpha) * self.yaw_rate + alpha * yaw_rate)

        self.position = (x, y, z)
        self.yaw = yaw
        self.last_time = now
        self.received_at = now

    def is_fresh(self, now):
        return (
            self.received_at is not None
            and 0.0 <= now - self.received_at <= self.timeout
        )

    def predicted(self, now):
        if not self.is_fresh(now) or self.position is None:
            return None

        horizon = self.horizon
        target_x = self.position[0] + self.velocity[0] * horizon
        target_y = self.position[1] + self.velocity[1] * horizon
        target_yaw = normalize_angle(self.yaw + self.yaw_rate * horizon)

        target_x = clamp(
            target_x,
            -self.field_half_x + self.field_margin,
            self.field_half_x - self.field_margin,
        )
        target_y = clamp(
            target_y,
            -self.field_half_y + self.field_margin,
            self.field_half_y - self.field_margin,
        )
        half_yaw = target_yaw * 0.5
        orientation = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
        return (target_x, target_y, self.position[2]), orientation
