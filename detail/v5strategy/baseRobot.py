import math

from V5RPC import Robot, Vector2, Wheel
from GlobalVariable import GlobalVariable as GV


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def wrap_degrees(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def length(dx, dy):
    return math.hypot(dx, dy)


class PID:
    def __init__(self, proportion, integral, derivative):
        self.proportion = proportion
        self.integral = integral
        self.derivative = derivative
        self.last_error = 0.0
        self.previous_error = 0.0
        self.last_output = 0.0
        self.current_error = 0.0
        self.used = False

    def calculate(self, error):
        self.current_error = error
        self.used = True
        return self.last_output + (
            self.proportion * (error - self.last_error)
            + self.integral * error
            + self.derivative * (error - 2.0 * self.last_error + self.previous_error)
        )

    def finish(self):
        if not self.used:
            return
        self.last_output += (
            self.proportion * (self.current_error - self.last_error)
            + self.integral * self.current_error
            + self.derivative * (self.current_error - 2.0 * self.last_error + self.previous_error)
        )
        self.previous_error = self.last_error
        self.last_error = self.current_error
        self.used = False

    def reset(self):
        self.last_error = 0.0
        self.previous_error = 0.0
        self.last_output = 0.0
        self.current_error = 0.0
        self.used = False


class BaseRobot:
    def __init__(self):
        self.robot = None
        self.tick = -1
        self.history = [
            Robot(Vector2(0.0, 0.0), 0.0, Wheel(0.0, 0.0)) for _ in range(8)
        ]
        self.prediction = [
            Robot(Vector2(0.0, 0.0), 0.0, Wheel(0.0, 0.0)) for _ in range(8)
        ]
        self.angular_speed = [0.0] * 8
        self.reset_flag = True
        self.last_angle_dif = 0.0
        self.last_dis_dif = 0.0
        self.pid_move = PID(2.0, 0.0, 3.0)
        self.pid_angle = PID(1.5, 0.0, 5.0)
        self.pid_dis = PID(1.8, 0.0, 8.0)
        self.turn_gain = 8.0

    def update(self, robot, reset=False):
        self.tick = GV.tick
        self.robot = robot
        self.reset_flag = reset
        current = robot.copy()

        if reset:
            self.history = [current.copy() for _ in range(8)]
            self.prediction = [current.copy() for _ in range(8)]
            self.angular_speed = [0.0] * 8
            self.last_angle_dif = 0.0
            self.last_dis_dif = 0.0
            self.pid_move.reset()
            self.pid_angle.reset()
            self.pid_dis.reset()
        else:
            for index in range(7, 0, -1):
                self.history[index] = self.history[index - 1].copy()
            self.history[0] = current.copy()

        self.predict_information(GV.tick_delay)

    def finish_tick(self):
        self.pid_move.finish()
        self.pid_angle.finish()
        self.pid_dis.finish()

    def get_pos(self):
        return self.robot.position

    def get_rotation(self):
        return self.robot.rotation

    def set_wheel_velocity(self, left, right):
        limit = GV.MAX_WHEEL_SPEED
        self.robot.wheel.left_speed = clamp(left, -limit, limit)
        self.robot.wheel.right_speed = clamp(right, -limit, limit)

    def stop(self):
        self.set_wheel_velocity(0.0, 0.0)

    def predicted(self, delay=None):
        if delay is None:
            delay = GV.tick_delay
        state = self.prediction[delay]
        return state.position.x, state.position.y, state.rotation

    def get_robot_information(self, time):
        if time >= 0:
            return self.prediction[time]
        return self.history[-time]

    def predict_information(self, tick_delay):
        self.prediction[0] = self.robot.copy()
        if len(self.angular_speed) > 1:
            self.angular_speed[0] = self.angular_speed[1]

        for i in range(1, tick_delay + 1):
            previous = self.get_robot_information(i - 1)
            previous_previous = self.get_robot_information(i - 2)
            predicted_rotation = previous.rotation
            predicted_position = previous.position.copy()

            dx = predicted_position.x - previous_previous.position.x
            dy = predicted_position.y - previous_previous.position.y
            predicted_line_speed = length(dx, dy)

            logic_angle = math.degrees(math.atan2(dy, dx))
            if predicted_position.x < previous_previous.position.x:
                if logic_angle > 0.0:
                    logic_angle = 180.0 - logic_angle
                else:
                    logic_angle = -180.0 - logic_angle

            delta_angle = logic_angle - previous.rotation
            if delta_angle > 180.0:
                delta_angle -= 360.0
            if delta_angle < -180.0:
                delta_angle += 360.0
            if abs(delta_angle) <= 90.0:
                predicted_line_speed = -predicted_line_speed

            if self.angular_speed[i - 1] > 0.0:
                if predicted_rotation > previous_previous.rotation:
                    predicted_angular_speed = predicted_rotation - previous_previous.rotation
                else:
                    predicted_angular_speed = 360.0 + predicted_rotation - previous_previous.rotation
            else:
                if predicted_rotation > previous_previous.rotation:
                    predicted_angular_speed = predicted_rotation - previous_previous.rotation - 360.0
                else:
                    predicted_angular_speed = predicted_rotation - previous_previous.rotation

            wheel_speed = self.history[tick_delay + 1 - i].wheel.copy()
            wheel_speed.left_speed = clamp(wheel_speed.left_speed, -130.0, 130.0)
            wheel_speed.right_speed = clamp(wheel_speed.right_speed, -130.0, 130.0)

            k1 = 0.002362192
            k2 = math.exp(-1.0 / 0.9231)
            k3 = 1.0 - math.exp(-1.0 / 3.096)
            k4 = 0.53461992

            if wheel_speed.left_speed != 0.0 and wheel_speed.right_speed != 0.0:
                next_line_speed = (
                    predicted_line_speed * 0.939534127623834133
                    + (wheel_speed.left_speed + wheel_speed.right_speed) * 0.5 * k1
                )
            else:
                next_line_speed = predicted_line_speed * k2

            next_angular_speed = (
                predicted_angular_speed
                + (
                    (wheel_speed.right_speed - wheel_speed.left_speed) * 0.5 * k4
                    - predicted_angular_speed
                )
                * k3
            )
            self.angular_speed[i] = next_angular_speed

            self.prediction[i].position.x = (
                self.prediction[i - 1].position.x
                + next_line_speed * math.cos(math.radians(self.prediction[i - 1].rotation))
            )
            self.prediction[i].position.y = (
                self.prediction[i - 1].position.y
                + next_line_speed * math.sin(math.radians(self.prediction[i - 1].rotation))
            )
            self.prediction[i].rotation = self.prediction[i - 1].rotation + next_angular_speed
            self.prediction[i].rotation = wrap_degrees(self.prediction[i].rotation)

    def moveto_dis(self, target_x, target_y):
        if self.tick <= 3 or self.tick % 50 == 0 or self.reset_flag:
            self.last_angle_dif = 0.0
            self.last_dis_dif = 0.0
            self.pid_move.reset()

        predicted = self.prediction[GV.tick_delay]
        dx = target_x - predicted.position.x
        dy = target_y - predicted.position.y
        angle_to = math.degrees(math.atan2(dy, dx))
        angle_diff = wrap_degrees(angle_to - self.robot.rotation)
        distance = length(dx, dy)

        if distance < 1.2:
            self.stop()
            return

        maximum = 0.3 * distance + 1.1 * (distance - self.last_dis_dif)
        maximum = clamp(maximum, 0.0, 130.0)

        if abs(angle_diff) < 90.0:
            delta = self.turn_gain * angle_diff
            right = maximum + delta
            left = maximum - delta
        else:
            angle_diff = wrap_degrees(angle_diff + 180.0)
            delta = self.turn_gain * angle_diff
            right = -maximum + delta
            left = -maximum - delta

        self.last_angle_dif = angle_diff
        self.last_dis_dif = distance
        self.set_wheel_velocity(left, right)

    def move_to(self, target_x, target_y, max_speed=130.0, tolerance=2.0):
        self.moveto_dis(target_x, target_y)

    def moveto(self, target_x, target_y):
        if self.tick <= 3 or self.tick % 50 == 0 or self.reset_flag:
            self.pid_move.reset()

        predicted = self.prediction[GV.tick_delay]
        dx = target_x - predicted.position.x
        dy = target_y - predicted.position.y
        angle_to = math.degrees(math.atan2(dy, dx))
        angle_diff = wrap_degrees(angle_to - self.robot.rotation)

        if length(dx, dy) < 1.2:
            self.stop()
            return

        maximum = GV.MAX_WHEEL_SPEED
        if abs(angle_diff) < 90.0:
            delta = self.turn_gain * angle_diff
            right = maximum + delta
            left = maximum - delta
        else:
            angle_diff = wrap_degrees(angle_diff + 180.0)
            delta = self.turn_gain * angle_diff
            right = -maximum + delta
            left = -maximum - delta
        self.set_wheel_velocity(left, right)

    def turn_to_angle(self, target_angle):
        predicted = self.prediction[GV.tick_delay]
        angle_diff = wrap_degrees(target_angle - self.robot.rotation)
        if abs(angle_diff) < 90.0:
            delta = self.turn_gain * angle_diff
            right = delta
            left = -delta
        else:
            angle_diff = wrap_degrees(angle_diff + 180.0)
            delta = self.turn_gain * angle_diff
            right = -delta
            left = delta
        self.set_wheel_velocity(left, right)

    def drive_through(self, ball_x, ball_y, target_x, target_y, power=140.0, allow_goal_area=False):
        dir_x = target_x - ball_x
        dir_y = target_y - ball_y
        magnitude = length(dir_x, dir_y)
        if magnitude < 1e-6:
            dir_x, dir_y = -1.0, 0.0
        else:
            dir_x /= magnitude
            dir_y /= magnitude

        px, py, _ = self.predicted()
        rx = px - ball_x
        ry = py - ball_y
        behind_x = ball_x - dir_x * 5.5
        behind_y = ball_y - dir_y * 5.5
        behind_distance = length(px - behind_x, py - behind_y)
        alignment = rx * (-dir_x) + ry * (-dir_y)

        if not allow_goal_area and behind_x > 58.0 and abs(behind_y) < 10.0:
            behind_x = 58.0
            behind_distance = length(px - behind_x, py - behind_y)

        if alignment > 0.05 and behind_distance < 12.0:
            aim_x = ball_x + dir_x * 30.0
            aim_y = ball_y + dir_y * 30.0
            if not allow_goal_area and aim_x > 58.0 and abs(aim_y) < 10.0:
                aim_x = 58.0
            self.moveto_dis(aim_x, aim_y)
            return

        perp_x = -dir_y
        perp_y = dir_x
        side = 1.0 if rx * perp_x + ry * perp_y >= 0.0 else -1.0
        near_ball = length(rx, ry) < 8.0
        offset = 7.0 if near_ball else 0.0
        waypoint_x = behind_x + perp_x * offset * side
        waypoint_y = behind_y + perp_y * offset * side
        if not allow_goal_area and waypoint_x > 58.0 and abs(waypoint_y) < 10.0:
            waypoint_x = 58.0
            waypoint_y = 12.0 if waypoint_y >= 0.0 else -12.0
        self.moveto_dis(waypoint_x, waypoint_y)
