import math


class RaceSafety:
    def __init__(
        self,
        field_half_x,
        field_half_y,
        field_margin,
        minimum_flight_altitude,
        recovery_altitude,
        enemy_clearance,
        enemy_vertical_range,
        center_clearance,
    ):
        self.limit_x = field_half_x - field_margin
        self.limit_y = field_half_y - field_margin
        self.minimum_flight_altitude = minimum_flight_altitude
        self.recovery_altitude = recovery_altitude
        self.enemy_clearance = enemy_clearance
        self.enemy_vertical_range = enemy_vertical_range
        self.center_clearance = center_clearance

    def center_recovery(self, x, y):
        radius = math.hypot(x, y)
        if radius >= self.center_clearance:
            return None
        if radius < 1e-6:
            return self.center_clearance, 0.0
        scale = self.center_clearance / radius
        return x * scale, y * scale

    def guard_altitude(self, current_z, target_z):
        if current_z < self.minimum_flight_altitude:
            return max(target_z, self.recovery_altitude)
        return target_z

    def avoid_enemy(self, x, y, current, enemy):
        if enemy is None or current is None:
            return x, y
        dx = x - enemy[0]
        dy = y - enemy[1]
        distance = math.hypot(dx, dy)
        vertical = abs(current[2] - enemy[2])
        if vertical > self.enemy_vertical_range:
            return x, y
        if distance >= self.enemy_clearance:
            return x, y
        if distance < 1e-6:
            dx, dy, distance = 1.0, 0.0, 1.0
        push = self.enemy_clearance - distance
        return x + dx / distance * push, y + dy / distance * push

    def clamp(self, x, y):
        return (
            max(-self.limit_x, min(self.limit_x, x)),
            max(-self.limit_y, min(self.limit_y, y)),
        )
