import math


class CenterGuard:
    def __init__(self, trigger_radius, escape_radius):
        if trigger_radius <= 0.0 or escape_radius <= trigger_radius:
            raise ValueError('center guard radii are invalid')
        self.trigger_radius = trigger_radius
        self.escape_radius = escape_radius

    def escape_target(self, x, y):
        radius = math.hypot(x, y)
        if radius >= self.trigger_radius:
            return None
        if radius < 1e-6:
            return self.escape_radius, 0.0
        scale = self.escape_radius / radius
        return x * scale, y * scale
