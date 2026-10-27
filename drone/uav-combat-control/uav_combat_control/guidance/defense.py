import math


class DuelPolicy:
    def __init__(
        self,
        engage_distance,
        vertical_threshold,
        climb_altitude,
        maximum_climb,
    ):
        self.engage_distance = engage_distance
        self.vertical_threshold = vertical_threshold
        self.climb_altitude = climb_altitude
        self.maximum_climb = maximum_climb

    def altitude(self, current_altitude, own, enemy):
        if enemy is None or own is None:
            return current_altitude
        horizontal = math.hypot(own[0] - enemy[0], own[1] - enemy[1])
        vertical = abs(own[2] - enemy[2])
        if horizontal <= self.engage_distance and vertical <= self.vertical_threshold:
            return max(current_altitude, self.climb_altitude)
        return current_altitude

    def yaw(self, own, enemy):
        if enemy is None or own is None:
            return None
        horizontal = math.hypot(own[0] - enemy[0], own[1] - enemy[1])
        if horizontal > self.engage_distance:
            return None
        return math.atan2(enemy[1] - own[1], enemy[0] - own[0])
