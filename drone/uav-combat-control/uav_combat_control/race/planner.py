import math


class OrbitPlanner:
    def __init__(
        self,
        team_id,
        radius,
        speed,
        altitude,
        field_half_x,
        field_half_y,
        field_margin,
    ):
        if team_id not in (1, 2):
            raise ValueError('team id must be 1 or 2')
        if radius <= 0.0 or speed <= 0.0 or altitude <= 0.0:
            raise ValueError('planner values must be positive')
        self.phase = math.pi if team_id == 1 else 0.0
        self.radius = radius
        self.speed = speed
        self.altitude = altitude
        self.limit_x = field_half_x - field_margin
        self.limit_y = field_half_y - field_margin
        self.omega = -speed / radius

    def target(self, elapsed_sec):
        theta = self.phase + self.omega * elapsed_sec
        x = self.radius * math.cos(theta)
        y = self.radius * math.sin(theta)
        x = max(-self.limit_x, min(self.limit_x, x))
        y = max(-self.limit_y, min(self.limit_y, y))
        return x, y, self.altitude, theta
