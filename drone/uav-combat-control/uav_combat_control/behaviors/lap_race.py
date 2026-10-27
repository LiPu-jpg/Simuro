import math

from uav_combat_control.behaviors.common import (
    FlightTarget,
    PI,
    enu_to_ned_position,
    enu_yaw_to_ned_yaw,
    normalize_angle,
)


class LapRaceBehavior:
    def __init__(
        self,
        team_id,
        circle_radius,
        horizontal_speed,
        low_altitude,
        high_altitude,
    ):
        if team_id not in (1, 2):
            raise ValueError('team id must be 1 or 2')
        if circle_radius <= 0.0:
            raise ValueError('circle radius must be positive')
        if horizontal_speed <= 0.0:
            raise ValueError('horizontal speed must be positive')
        if low_altitude <= 0.0 or high_altitude <= low_altitude:
            raise ValueError('lap race altitudes are invalid')

        self.phase_offset = PI if team_id == 1 else 0.0
        self.circle_radius = circle_radius
        self.horizontal_speed = horizontal_speed
        self.low_altitude = low_altitude
        self.high_altitude = high_altitude
        self.angular_speed = -horizontal_speed / circle_radius

    def __call__(self, elapsed_sec, enemy_altitude):
        theta = normalize_angle(
            self.phase_offset + self.angular_speed * elapsed_sec)
        radius = self.circle_radius
        if enemy_altitude is not None and enemy_altitude <= 1.8:
            altitude = self.high_altitude
        else:
            altitude = self.low_altitude
        return FlightTarget(
            position_ned=enu_to_ned_position(
                radius * math.cos(theta),
                radius * math.sin(theta),
                altitude,
            ),
            yaw_ned=enu_yaw_to_ned_yaw(theta),
        )
