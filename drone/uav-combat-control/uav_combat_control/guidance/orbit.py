import math


class OrbitPath:
    def __init__(self, team_id, radius, speed, field_half_x, field_half_y, margin):
        if team_id not in (1, 2):
            raise ValueError('team id must be 1 or 2')
        if radius <= 0.0 or speed <= 0.0:
            raise ValueError('orbit values must be positive')
        self.phase = math.pi if team_id == 1 else 0.0
        self.radius = radius
        self.speed = speed
        self.omega = -speed / radius
        self.limit_x = field_half_x - margin
        self.limit_y = field_half_y - margin

    def point(self, elapsed_sec):
        theta = self.phase + self.omega * elapsed_sec
        x = self.radius * math.cos(theta)
        y = self.radius * math.sin(theta)
        x = max(-self.limit_x, min(self.limit_x, x))
        y = max(-self.limit_y, min(self.limit_y, y))
        return x, y, theta
