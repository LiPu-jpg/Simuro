import math


class RearGuard:
    def __init__(self, engage_distance, ceiling_altitude, vertical_margin=0.08):
        self.engage_distance = engage_distance
        self.ceiling_altitude = ceiling_altitude
        self.vertical_margin = vertical_margin

    def yaw(self, own, enemy):
        if own is None or enemy is None:
            return None
        horizontal = math.hypot(own[0] - enemy[0], own[1] - enemy[1])
        if horizontal > self.engage_distance:
            return None
        return math.atan2(enemy[1] - own[1], enemy[0] - own[0])

    def altitude(self, own, enemy, base_altitude):
        if own is None or enemy is None:
            return base_altitude
        horizontal = math.hypot(own[0] - enemy[0], own[1] - enemy[1])
        if horizontal > self.engage_distance:
            return base_altitude
        desired = enemy[2] + self.vertical_margin
        return min(self.ceiling_altitude, max(base_altitude, desired))
