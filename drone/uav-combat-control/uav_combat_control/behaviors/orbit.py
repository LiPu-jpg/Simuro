import math

from uav_combat_control.behaviors.common import (
    FlightTarget,
    PI,
    PI_2,
    enu_to_ned_position,
    enu_yaw_to_ned_yaw,
    normalize_angle,
)


def synchronized_orbit_theta(team_id, elapsed_sec, angular_speed):
    if team_id not in (1, 2):
        raise ValueError('team id must be 1 or 2')
    if elapsed_sec < 0.0:
        raise ValueError('orbit elapsed time must be non-negative')
    phase_offset = PI if team_id == 1 else 0.0
    return normalize_angle(phase_offset + angular_speed * elapsed_sec)


def orbit_duration_for_laps(circle_radius, horizontal_speed, orbit_laps):
    if circle_radius <= 0.0:
        raise ValueError('circle radius must be positive')
    if horizontal_speed <= 0.0:
        raise ValueError('horizontal speed must be positive')
    if orbit_laps <= 0.0:
        raise ValueError('orbit laps must be positive')
    return math.tau * circle_radius * orbit_laps / horizontal_speed


class OrbitBehavior:
    def __init__(
        self,
        team_id,
        target_altitude,
        circle_radius,
        horizontal_speed,
        orientation_mode,
    ):
        if target_altitude <= 0.0:
            raise ValueError('target altitude must be positive')
        if circle_radius <= 0.0:
            raise ValueError('circle radius must be positive')
        if horizontal_speed <= 0.0:
            raise ValueError('horizontal speed must be positive')
        if orientation_mode not in (0, 1, 2):
            raise ValueError('orientation mode must be 0, 1, or 2')

        self.team_id = team_id
        self.target_altitude = target_altitude
        self.circle_radius = circle_radius
        self.horizontal_speed = horizontal_speed
        self.orientation_mode = orientation_mode

    def __call__(self, elapsed_sec):
        angular_speed = -self.horizontal_speed / self.circle_radius
        theta = synchronized_orbit_theta(
            self.team_id,
            elapsed_sec,
            angular_speed,
        )
        target_x = self.circle_radius * math.cos(theta)
        target_y = self.circle_radius * math.sin(theta)

        if self.orientation_mode == 0:
            yaw_enu = normalize_angle(theta + PI)
        elif self.orientation_mode == 1:
            yaw_enu = theta
        else:
            yaw_enu = normalize_angle(theta - PI_2)

        return FlightTarget(
            position_ned=enu_to_ned_position(
                target_x,
                target_y,
                self.target_altitude,
            ),
            yaw_ned=enu_yaw_to_ned_yaw(yaw_enu),
        )
