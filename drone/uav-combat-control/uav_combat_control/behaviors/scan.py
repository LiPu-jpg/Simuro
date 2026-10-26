import math
from dataclasses import dataclass
from typing import Optional, Tuple

from uav_combat_control.behaviors.common import (
    FlightTarget,
    clockwise_tangent_yaw,
    enu_to_ned_position,
    enu_yaw_to_ned_yaw,
    get_yaw_from_quaternion,
    normalize_angle,
)


Point2 = Tuple[float, float]


@dataclass(frozen=True)
class ScanOutput:
    target: Optional[FlightTarget]
    scanner_team: int
    phase: str


def active_scanner_team(scan_elapsed_sec, role_duration_sec):
    if scan_elapsed_sec < 0.0:
        raise ValueError('scan elapsed time must be non-negative')
    if role_duration_sec <= 0.0:
        raise ValueError('scan role duration must be positive')
    role_index = int(scan_elapsed_sec // role_duration_sec)
    return 1 if role_index % 2 == 0 else 2


def compute_rear_scan_target(
    enemy_position,
    enemy_orientation,
    target_altitude,
    follow_distance,
):
    if follow_distance <= 0.0:
        raise ValueError('follow distance must be positive')

    enemy_yaw = get_yaw_from_quaternion(*enemy_orientation)
    target_x = enemy_position[0] - follow_distance * math.cos(enemy_yaw)
    target_y = enemy_position[1] - follow_distance * math.sin(enemy_yaw)
    return FlightTarget(
        position_ned=enu_to_ned_position(
            target_x,
            target_y,
            target_altitude,
        ),
        yaw_ned=enu_yaw_to_ned_yaw(enemy_yaw),
    )


def compute_planar_approach_velocity(
    current_position,
    target_position_ned,
    maximum_speed,
    stop_distance=0.15,
):
    if maximum_speed <= 0.0:
        raise ValueError('maximum speed must be positive')
    if stop_distance < 0.0:
        raise ValueError('stop distance must be non-negative')

    current_x, current_y, _ = current_position
    target_x = target_position_ned[1]
    target_y = target_position_ned[0]
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    distance = math.hypot(delta_x, delta_y)
    if distance <= stop_distance:
        return 0.0, 0.0, 0.0

    approach_speed = min(maximum_speed, 2.0 * (distance - stop_distance))
    velocity_x_enu = approach_speed * delta_x / distance
    velocity_y_enu = approach_speed * delta_y / distance
    return velocity_y_enu, velocity_x_enu, 0.0


def limit_velocity_for_opponent_clearance(
    velocity_ned,
    current_position,
    opponent_position,
    minimum_clearance,
    slowdown_distance,
):
    if minimum_clearance <= 0.0:
        raise ValueError('minimum opponent clearance must be positive')
    if slowdown_distance <= minimum_clearance:
        raise ValueError(
            'opponent slowdown distance must exceed minimum clearance')

    velocity_x_enu = velocity_ned[1]
    velocity_y_enu = velocity_ned[0]
    opponent_delta_x = opponent_position[0] - current_position[0]
    opponent_delta_y = opponent_position[1] - current_position[1]
    opponent_distance = math.hypot(opponent_delta_x, opponent_delta_y)
    speed = math.hypot(velocity_x_enu, velocity_y_enu)
    if speed == 0.0 or opponent_distance == 0.0:
        return velocity_ned

    moving_toward_opponent = (
        velocity_x_enu * opponent_delta_x
        + velocity_y_enu * opponent_delta_y
    ) > 0.0
    if not moving_toward_opponent or opponent_distance >= slowdown_distance:
        return velocity_ned

    speed_scale = max(
        0.0,
        (opponent_distance - minimum_clearance)
        / (slowdown_distance - minimum_clearance),
    )
    return tuple(component * speed_scale for component in velocity_ned)


def distance_from_center_to_segment(start, end):
    start_x, start_y = start
    end_x, end_y = end
    delta_x = end_x - start_x
    delta_y = end_y - start_y
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared == 0.0:
        return math.hypot(start_x, start_y)

    projection = -(
        start_x * delta_x + start_y * delta_y
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest_x = start_x + projection * delta_x
    closest_y = start_y + projection * delta_y
    return math.hypot(closest_x, closest_y)


def plan_center_avoidance_route(
    start,
    goal,
    clearance_radius,
    detour_offset,
    preferred_side,
):
    if clearance_radius <= 0.0:
        raise ValueError('clearance radius must be positive')
    if detour_offset <= clearance_radius:
        raise ValueError('detour offset must exceed clearance radius')
    if preferred_side not in (-1, 1):
        raise ValueError('preferred side must be -1 or 1')
    if distance_from_center_to_segment(start, goal) > clearance_radius:
        return ()

    direction_x = goal[0] - start[0]
    direction_y = goal[1] - start[1]
    distance = math.hypot(direction_x, direction_y)
    if distance == 0.0:
        return ()
    direction_x /= distance
    direction_y /= distance
    perpendicular_x = -direction_y
    perpendicular_y = direction_x

    def candidate(side):
        offset_x = side * perpendicular_x * detour_offset
        offset_y = side * perpendicular_y * detour_offset
        return (
            (
                -direction_x * clearance_radius + offset_x,
                -direction_y * clearance_radius + offset_y,
            ),
            (
                direction_x * clearance_radius + offset_x,
                direction_y * clearance_radius + offset_y,
            ),
        )

    def route_length(route):
        points = (start, *route, goal)
        return sum(
            math.hypot(
                points[index + 1][0] - points[index][0],
                points[index + 1][1] - points[index][1],
            )
            for index in range(len(points) - 1)
        )

    preferred_route = candidate(preferred_side)
    alternate_route = candidate(-preferred_side)
    if route_length(alternate_route) + 1e-9 < route_length(preferred_route):
        return alternate_route
    return preferred_route


def plan_point_avoidance_route(
    start,
    goal,
    obstacle,
    clearance_radius,
    detour_offset,
    preferred_side,
):
    relative_route = plan_center_avoidance_route(
        start=(start[0] - obstacle[0], start[1] - obstacle[1]),
        goal=(goal[0] - obstacle[0], goal[1] - obstacle[1]),
        clearance_radius=clearance_radius,
        detour_offset=detour_offset,
        preferred_side=preferred_side,
    )
    return tuple(
        (point[0] + obstacle[0], point[1] + obstacle[1])
        for point in relative_route
    )


class ScanBehavior:
    def __init__(
        self,
        team_id,
        target_altitude,
        role_duration_sec,
        follow_distance,
        approach_speed,
        obstacle_clearance_radius,
        obstacle_detour_offset,
        waypoint_tolerance,
        minimum_opponent_clearance,
        opponent_slowdown_distance,
        field_half_x=4.5,
        field_half_y=3.0,
        field_margin=0.3,
        route_refresh_sec=0.8,
        route_refresh_distance=0.35,
    ):
        if team_id not in (1, 2):
            raise ValueError('team id must be 1 or 2')
        if target_altitude <= 0.0:
            raise ValueError('target altitude must be positive')
        if role_duration_sec <= 0.0:
            raise ValueError('scan role duration must be positive')
        if follow_distance <= 0.0:
            raise ValueError('follow distance must be positive')
        if approach_speed <= 0.0:
            raise ValueError('approach speed must be positive')
        if waypoint_tolerance <= 0.0:
            raise ValueError('waypoint tolerance must be positive')
        if minimum_opponent_clearance <= 0.0:
            raise ValueError('minimum opponent clearance must be positive')
        if opponent_slowdown_distance <= minimum_opponent_clearance:
            raise ValueError(
                'opponent slowdown distance must exceed minimum clearance')
        if field_margin < 0.0:
            raise ValueError('field margin must be non-negative')
        if field_half_x <= field_margin or field_half_y <= field_margin:
            raise ValueError('field margin exceeds field bounds')
        if route_refresh_sec <= 0.0:
            raise ValueError('route refresh period must be positive')
        if route_refresh_distance < 0.0:
            raise ValueError('route refresh distance must be non-negative')

        self.team_id = team_id
        self.target_altitude = target_altitude
        self.role_duration_sec = role_duration_sec
        self.follow_distance = follow_distance
        self.approach_speed = approach_speed
        self.obstacle_clearance_radius = obstacle_clearance_radius
        self.obstacle_detour_offset = obstacle_detour_offset
        self.waypoint_tolerance = waypoint_tolerance
        self.minimum_opponent_clearance = minimum_opponent_clearance
        self.opponent_slowdown_distance = opponent_slowdown_distance
        self.field_half_x = field_half_x
        self.field_half_y = field_half_y
        self.field_margin = field_margin
        self.route_refresh_sec = route_refresh_sec
        self.route_refresh_distance = route_refresh_distance
        self.reset()

    def reset(self):
        self._scanner_team = self.team_id
        self._hold_target = None
        self._route = None
        self._route_index = 0
        self._route_target = None
        self._route_time = None

    def _clamp_enu(self, x, y):
        x = max(
            -self.field_half_x + self.field_margin,
            min(self.field_half_x - self.field_margin, x),
        )
        y = max(
            -self.field_half_y + self.field_margin,
            min(self.field_half_y - self.field_margin, y),
        )
        return x, y

    def _avoid_center(self, x, y):
        radius = math.hypot(x, y)
        if radius < self.obstacle_clearance_radius:
            scale = (
                self.obstacle_clearance_radius + 0.03
            ) / max(radius, 1e-6)
            x *= scale
            y *= scale
        return self._clamp_enu(x, y)

    def _target_moved(self, target):
        if self._route_target is None:
            return True
        return math.hypot(
            target[0] - self._route_target[0],
            target[1] - self._route_target[1],
        ) > self.route_refresh_distance

    def _wall_velocity(self, current_position, velocity_ned):
        velocity_x_enu = velocity_ned[1]
        velocity_y_enu = velocity_ned[0]
        margin = max(self.field_margin, 0.05)
        gain = 1.2

        if current_position[0] > self.field_half_x - margin:
            velocity_x_enu -= gain * (
                current_position[0] - (self.field_half_x - margin)
            ) / margin
        elif current_position[0] < -self.field_half_x + margin:
            velocity_x_enu += gain * (
                (-self.field_half_x + margin) - current_position[0]
            ) / margin

        if current_position[1] > self.field_half_y - margin:
            velocity_y_enu -= gain * (
                current_position[1] - (self.field_half_y - margin)
            ) / margin
        elif current_position[1] < -self.field_half_y + margin:
            velocity_y_enu += gain * (
                (-self.field_half_y + margin) - current_position[1]
            ) / margin

        speed = math.hypot(velocity_x_enu, velocity_y_enu)
        if speed > self.approach_speed:
            scale = self.approach_speed / speed
            velocity_x_enu *= scale
            velocity_y_enu *= scale
        return velocity_y_enu, velocity_x_enu, velocity_ned[2]

    def _build_route(self, current_xy, goal_xy, enemy_xy, preferred_side):
        center_route = plan_center_avoidance_route(
            start=current_xy,
            goal=goal_xy,
            clearance_radius=self.obstacle_clearance_radius,
            detour_offset=self.obstacle_detour_offset,
            preferred_side=preferred_side,
        )
        opponent_route_start = (
            center_route[-1] if center_route else current_xy)
        opponent_route = plan_point_avoidance_route(
            start=opponent_route_start,
            goal=goal_xy,
            obstacle=enemy_xy,
            clearance_radius=self.minimum_opponent_clearance,
            detour_offset=self.opponent_slowdown_distance,
            preferred_side=preferred_side,
        )
        route = []
        for point in (*center_route, *opponent_route):
            route.append(self._avoid_center(*point))
        return tuple(route)

    def __call__(
        self,
        scan_elapsed_sec,
        current_position,
        enemy_position=None,
        enemy_orientation=None,
        enemy_velocity=None,
    ):
        self._scanner_team = self.team_id
        if enemy_position is None or enemy_orientation is None:
            self._route = None
            self._route_index = 0
            self._route_target = None
            self._route_time = None
            return ScanOutput(
                target=None,
                scanner_team=self.team_id,
                phase='waiting_enemy_pose',
            )
        return self._scanner_target(
            scan_elapsed_sec,
            current_position,
            enemy_position,
            enemy_orientation,
            enemy_velocity,
        )

    def _scanner_target(
        self,
        scan_elapsed_sec,
        current_position,
        enemy_position,
        enemy_orientation,
        enemy_velocity=None,
    ):
        current_x = current_position[0]
        current_y = current_position[1]
        enemy_x = enemy_position[0]
        enemy_y = enemy_position[1]
        enemy_yaw = get_yaw_from_quaternion(*enemy_orientation)
        if enemy_velocity is None:
            enemy_velocity_x = 0.0
            enemy_velocity_y = 0.0
        else:
            enemy_velocity_x = enemy_velocity[0]
            enemy_velocity_y = enemy_velocity[1]

        rear_bearing = normalize_angle(enemy_yaw + math.pi)
        desired_x = enemy_x + self.follow_distance * math.cos(rear_bearing)
        desired_y = enemy_y + self.follow_distance * math.sin(rear_bearing)
        desired_x, desired_y = self._clamp_enu(desired_x, desired_y)
        desired_x, desired_y = self._avoid_center(desired_x, desired_y)

        relative_x = current_x - enemy_x
        relative_y = current_y - enemy_y
        distance = math.hypot(relative_x, relative_y)
        angle_us = math.atan2(relative_y, relative_x)
        delta = normalize_angle(rear_bearing - angle_us)
        orbit_radius = max(
            self.follow_distance,
            self.minimum_opponent_clearance + 0.2,
        )

        segment_clearance = distance_from_center_to_segment(
            (relative_x, relative_y),
            (desired_x - enemy_x, desired_y - enemy_y),
        )
        need_orbit = (
            distance < orbit_radius
            or (
                distance < self.opponent_slowdown_distance
                and abs(delta) > 0.6
            )
            or segment_clearance < self.minimum_opponent_clearance
        )

        velocity_x = 0.0
        velocity_y = 0.0
        phase = 'rear_approach'

        if need_orbit and abs(delta) > 0.35:
            if delta > 0.0:
                tangent_x = -relative_y
                tangent_y = relative_x
            else:
                tangent_x = relative_y
                tangent_y = -relative_x
            tangent_length = math.hypot(tangent_x, tangent_y)
            if tangent_length > 1e-6:
                tangent_x /= tangent_length
                tangent_y /= tangent_length

            radial_x = relative_x / max(distance, 1e-6)
            radial_y = relative_y / max(distance, 1e-6)
            radial_error = orbit_radius - distance
            radial_speed = max(
                -self.approach_speed,
                min(self.approach_speed, 1.2 * radial_error),
            )
            if distance < self.minimum_opponent_clearance:
                radial_speed = self.approach_speed

            velocity_x = (
                tangent_x * self.approach_speed
                + radial_x * radial_speed
            )
            velocity_y = (
                tangent_y * self.approach_speed
                + radial_y * radial_speed
            )
            phase = 'orbit'
        else:
            delta_x = desired_x - current_x
            delta_y = desired_y - current_y
            distance_to_goal = math.hypot(delta_x, delta_y)
            if distance_to_goal > 1e-6:
                speed = min(
                    self.approach_speed,
                    max(0.15, 2.0 * distance_to_goal),
                )
                velocity_x = delta_x / distance_to_goal * speed
                velocity_y = delta_y / distance_to_goal * speed

        velocity_x += enemy_velocity_x * 0.6
        velocity_y += enemy_velocity_y * 0.6

        center_distance = math.hypot(current_x, current_y)
        center_influence = self.obstacle_clearance_radius + 0.6
        if center_distance < center_influence:
            outward_x = current_x / max(center_distance, 1e-6)
            outward_y = current_y / max(center_distance, 1e-6)
            strength = (
                center_influence - center_distance
            ) / center_influence
            velocity_x += outward_x * strength * self.approach_speed * 1.2
            velocity_y += outward_y * strength * self.approach_speed * 1.2
            inward_dot = (
                velocity_x * (-outward_x) + velocity_y * (-outward_y)
            )
            if inward_dot > 0.0:
                velocity_x += outward_x * inward_dot
                velocity_y += outward_y * inward_dot
            if center_distance < self.obstacle_clearance_radius:
                gradient = (
                    self.obstacle_clearance_radius - center_distance
                ) / self.obstacle_clearance_radius
                velocity_x += (
                    outward_x * gradient * self.approach_speed * 2.0
                )
                velocity_y += (
                    outward_y * gradient * self.approach_speed * 2.0
                )
            phase = 'center_avoid'

        wall_gain = 2.0
        near_wall = False
        margin = self.field_margin
        if current_x > self.field_half_x - margin:
            velocity_x -= (
                wall_gain
                * (current_x - (self.field_half_x - margin))
                / margin
                * self.approach_speed
            )
            near_wall = True
        elif current_x < -self.field_half_x + margin:
            velocity_x += (
                wall_gain
                * ((-self.field_half_x + margin) - current_x)
                / margin
                * self.approach_speed
            )
            near_wall = True
        if current_y > self.field_half_y - margin:
            velocity_y -= (
                wall_gain
                * (current_y - (self.field_half_y - margin))
                / margin
                * self.approach_speed
            )
            near_wall = True
        elif current_y < -self.field_half_y + margin:
            velocity_y += (
                wall_gain
                * ((-self.field_half_y + margin) - current_y)
                / margin
                * self.approach_speed
            )
            near_wall = True

        if near_wall and phase == 'rear_approach':
            phase = 'wall_avoid'

        speed = math.hypot(velocity_x, velocity_y)
        maximum_speed = self.approach_speed * 1.1
        if speed > maximum_speed:
            scale = maximum_speed / speed
            velocity_x *= scale
            velocity_y *= scale

        if distance < self.minimum_opponent_clearance:
            away_x = relative_x / max(distance, 1e-6)
            away_y = relative_y / max(distance, 1e-6)
            velocity_x = away_x * self.approach_speed
            velocity_y = away_y * self.approach_speed
            phase = 'collision_avoid'

        lookahead = 0.6
        target_x = current_x + velocity_x * lookahead
        target_y = current_y + velocity_y * lookahead
        target_x, target_y = self._avoid_center(target_x, target_y)
        target_x, target_y = self._clamp_enu(target_x, target_y)

        if (
            abs(delta) < 0.35
            and distance < orbit_radius + 0.5
            and not need_orbit
        ):
            target_x = desired_x
            target_y = desired_y

        if (
            abs(delta) < 0.7
            and distance < self.follow_distance * 0.95
        ):
            radial_x = relative_x / max(distance, 1e-6)
            radial_y = relative_y / max(distance, 1e-6)
            stand_x = enemy_x + radial_x * self.follow_distance
            stand_y = enemy_y + radial_y * self.follow_distance
            stand_x, stand_y = self._avoid_center(stand_x, stand_y)
            stand_x, stand_y = self._clamp_enu(stand_x, stand_y)
            target_x = stand_x
            target_y = stand_y
            velocity_x = radial_x * self.approach_speed * 0.6
            velocity_y = radial_y * self.approach_speed * 0.6
            phase = 'standoff'

        if abs(delta) < 0.6:
            yaw_enu = enemy_yaw
        else:
            yaw_enu = math.atan2(
                enemy_y - current_y,
                enemy_x - current_x,
            )

        return ScanOutput(
            target=FlightTarget(
                position_ned=enu_to_ned_position(
                    target_x,
                    target_y,
                    self.target_altitude,
                ),
                yaw_ned=enu_yaw_to_ned_yaw(yaw_enu),
                velocity_ned=(velocity_y, velocity_x, 0.0),
            ),
            scanner_team=self.team_id,
            phase=phase,
        )
