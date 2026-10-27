import math


class SafetyEnvelope:
    def __init__(
        self,
        field_half_x,
        field_half_y,
        field_margin,
        flight_floor,
        recovery_altitude,
        center_clearance,
        enemy_clearance,
        enemy_vertical_range,
    ):
        self.limit_x = field_half_x - field_margin
        self.limit_y = field_half_y - field_margin
        self.flight_floor = flight_floor
        self.recovery_altitude = recovery_altitude
        self.center_clearance = center_clearance
        self.enemy_clearance = enemy_clearance
        self.enemy_vertical_range = enemy_vertical_range

    def center_recovery(self, x, y):
        radius = math.hypot(x, y)
        if radius >= self.center_clearance:
            return None
        if radius < 1e-6:
            return self.center_clearance, 0.0
        scale = self.center_clearance / radius
        return x * scale, y * scale

    def altitude_recovery(self, current_z, target_z):
        if current_z < self.flight_floor:
            return max(target_z, self.recovery_altitude)
        return target_z

    def avoid_enemy(self, target, own, enemy):
        if enemy is None or own is None:
            return target
        dx = target[0] - enemy[0]
        dy = target[1] - enemy[1]
        distance = math.hypot(dx, dy)
        vertical = abs(own[2] - enemy[2])
        if vertical > self.enemy_vertical_range:
            return target
        if distance >= self.enemy_clearance:
            return target
        if distance < 1e-6:
            dx, dy, distance = 1.0, 0.0, 1.0
        push = self.enemy_clearance - distance
        return (
            target[0] + dx / distance * push,
            target[1] + dy / distance * push,
        )

    def clamp(self, x, y):
        return (
            max(-self.limit_x, min(self.limit_x, x)),
            max(-self.limit_y, min(self.limit_y, y)),
        )
