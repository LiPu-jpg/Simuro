import math
import random
from typing import List, Tuple, Union

from V5RPC import *
from baseRobot import BaseRobot, clamp, wrap_degrees, length
from GlobalVariable import GlobalVariable as GV

TEAM_NAME = "LiPu"

SELF_GOAL_X = 110.0
OPP_GOAL_X = -110.0
GOAL_HALF_WIDTH = 20.0
KEEPER_X = 108.0
PENALTY_SPOT_X = 72.5
FREE_BALL_X = 55.0
FREE_BALL_Y = 60.0

controllers = [BaseRobot() for _ in range(5)]
ball_history = []

race_state = -1
race_state_trigger = -1
in_shootout = False
active_set_piece = None
set_piece_start = 0
set_piece_ball = (0.0, 0.0)
set_piece_aux = 0.0
last_tick = -1
reset_controllers = True
last_chaser = -1


def _clamp(value, low, high):
    return clamp(value, low, high)


def _wrap(angle):
    return wrap_degrees(angle)


def _distance(x1, y1, x2, y2):
    return length(x2 - x1, y2 - y1)


def _sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def _safe_own_area(x, y):
    if x >= 60.0 and abs(y) <= 7.5:
        x = 58.0
        y = 12.0 if y >= 0.0 else -12.0
    return x, y


def _safe_own_penalty(x, y):
    if x >= 30.0 and abs(y) <= 17.5:
        x = 28.0
    return x, y


def _update_ball_history(ball):
    ball_history.append((ball.x, ball.y))
    if len(ball_history) > 8:
        ball_history.pop(0)


def _ball_velocity():
    if len(ball_history) < 2:
        return 0.0, 0.0
    x1, y1 = ball_history[-1]
    x0, y0 = ball_history[-2]
    vx = x1 - x0
    vy = y1 - y0
    if len(ball_history) >= 3:
        x2, y2 = ball_history[-3]
        vx = 0.6 * vx + 0.4 * (x0 - x2) * 0.5
        vy = 0.6 * vy + 0.4 * (y0 - y2) * 0.5
    return _clamp(vx, -8.0, 8.0), _clamp(vy, -8.0, 8.0)


class World:
    def __init__(self, field):
        self.field = field
        self.tick = field.tick
        self.ball = field.ball.position
        self.self_robots = field.self_robots
        self.opp_robots = field.opponent_robots
        self.ball_vx, self.ball_vy = _ball_velocity()
        self.self_pred = [controllers[i].predicted(4) for i in range(5)]
        self.ball_pred = (
            _clamp(self.ball.x + self.ball_vx * 3.0, -110.0, 110.0),
            _clamp(self.ball.y + self.ball_vy * 3.0, -90.0, 90.0),
        )

    def dist_to_ball(self, index):
        x, y, _ = self.self_pred[index]
        return length(x - self.ball.x, y - self.ball.y)

    def opp_goalkeeper_y(self):
        keeper = min(self.opp_robots, key=lambda robot: robot.position.x)
        return keeper.position.y

    def most_advanced_opponent(self):
        return max(self.opp_robots, key=lambda robot: robot.position.x)

    def nearest_opponent_distance(self, x, y):
        return min(_distance(robot.position.x, robot.position.y, x, y) for robot in self.opp_robots)

    def closest_field_player_to_ball(self):
        return min([0, 2, 3, 4], key=self.dist_to_ball)


@unbox_event
def on_event(event_type: int, args: EventArguments):
    global race_state, race_state_trigger, in_shootout
    global active_set_piece, reset_controllers, last_tick

    if event_type == 0 and args is not None and getattr(args, "judge_result", None) is not None:
        result = args.judge_result
        race_state = result.type
        race_state_trigger = result.offensive_team
        active_set_piece = None
    elif event_type in (1, 3, 4, 5):
        race_state = -1
        race_state_trigger = -1
        in_shootout = False
        active_set_piece = None
        reset_controllers = True
        last_tick = -1
        ball_history.clear()
    elif event_type in (6, 7):
        in_shootout = True
        race_state = 7
        active_set_piece = None
        reset_controllers = True


@unbox_int
def get_team_info(server_version: int) -> str:
    global reset_controllers
    reset_controllers = True
    return TEAM_NAME


def _fmt_placement(robots, ball):
    result = []
    for x, y, rotation in robots:
        result.append(
            (
                float(_clamp(x, -110.0, 110.0)),
                float(_clamp(y, -90.0, 90.0)),
                float(_wrap(rotation)),
            )
        )
    result.append(
        (
            float(_clamp(ball[0], -110.0, 110.0)),
            float(_clamp(ball[1], -90.0, 90.0)),
            float(ball[2]),
        )
    )
    return result


def _choose_wing(world):
    lower = 0
    upper = 0
    for robot in world.opp_robots:
        if robot.position.x < world.ball.x:
            if robot.position.y < 0.0:
                lower += 1
            elif robot.position.y > 0.0:
                upper += 1
    if lower < upper:
        return -1.0
    if upper < lower:
        return 1.0
    return -1.0 if random.random() < 0.5 else 1.0


def _choose_penalty_corner(field):
    defenders = [robot for robot in field.opponent_robots if robot.position.x < -100.0]
    if defenders:
        keeper = min(defenders, key=lambda robot: robot.position.x)
        if keeper.position.y >= 0.0:
            return -14.0
        return 14.0
    return -14.0 if random.random() < 0.5 else 14.0


def _kickoff_placement(attacking):
    if attacking:
        robots = [
            (12.0, 0.0, 180.0),
            (108.0, 0.0, 90.0),
            (10.0, -27.0, 180.0),
            (10.0, 27.0, 180.0),
            (50.0, 0.0, 180.0),
        ]
    else:
        robots = [
            (30.0, -12.0, 180.0),
            (108.0, 0.0, 90.0),
            (30.0, 12.0, 180.0),
            (50.0, -30.0, 180.0),
            (50.0, 30.0, 180.0),
        ]
    return _fmt_placement(robots, (0.0, 0.0, 0.0))


def _goalkick_placement(attacking):
    if attacking:
        robots = [
            (106.0, 0.0, 180.0),
            (108.0, 0.0, 90.0),
            (30.0, -25.0, 180.0),
            (30.0, 25.0, 180.0),
            (50.0, 0.0, 180.0),
        ]
        ball = (103.0, 0.0, 0.0)
    else:
        robots = [
            (20.0, 0.0, 180.0),
            (108.0, 0.0, 90.0),
            (35.0, -20.0, 180.0),
            (35.0, 20.0, 180.0),
            (55.0, -35.0, 180.0),
        ]
        ball = (-103.0, 0.0, 0.0)
    return _fmt_placement(robots, ball)


def _penalty_placement(field, attacking):
    global set_piece_aux
    if attacking:
        target_y = _choose_penalty_corner(field)
        set_piece_aux = target_y
        ball_x = -PENALTY_SPOT_X
        dir_x = OPP_GOAL_X - ball_x
        dir_y = target_y
        magnitude = math.hypot(dir_x, dir_y)
        dir_x /= magnitude
        dir_y /= magnitude
        kicker_x = ball_x - dir_x * 6.0
        kicker_y = -dir_y * 6.0
        angle = math.degrees(math.atan2(dir_y, dir_x))
        robots = [
            (kicker_x, kicker_y, angle),
            (108.0, 0.0, 90.0),
            (30.0, -20.0, 180.0),
            (30.0, 20.0, 180.0),
            (50.0, 0.0, 180.0),
        ]
        ball = (ball_x, 0.0, 0.0)
    else:
        set_piece_aux = 0.0
        robots = [
            (-20.0, -30.0, 180.0),
            (110.0, 0.0, 180.0),
            (-20.0, 30.0, 180.0),
            (-40.0, -50.0, 180.0),
            (-40.0, 50.0, 180.0),
        ]
        ball = (PENALTY_SPOT_X, 0.0, 0.0)
    return _fmt_placement(robots, ball)


def _free_ball_point(state):
    if state == JudgeResultEvent.ResultType.FreeKickRightTop:
        return FREE_BALL_X, FREE_BALL_Y
    if state == JudgeResultEvent.ResultType.FreeKickRightBot:
        return FREE_BALL_X, -FREE_BALL_Y
    if state == JudgeResultEvent.ResultType.FreeKickLeftTop:
        return -FREE_BALL_X, FREE_BALL_Y
    return -FREE_BALL_X, -FREE_BALL_Y


def _freekick_placement():
    ball_x, ball_y = _free_ball_point(race_state)
    side_x = 1.0 if ball_x > 0.0 else -1.0
    side_y = 1.0 if ball_y > 0.0 else -1.0
    robots = [
        (ball_x + 25.0, ball_y, 180.0),
        (108.0, 0.0, 90.0),
        (-side_x * 20.0, side_y * 30.0, 180.0),
        (side_x * 30.0, -side_y * 30.0, 180.0),
        (-side_x * 40.0, -side_y * 20.0, 180.0),
    ]
    return _fmt_placement(robots, (ball_x, ball_y, 0.0))


def _is_shootout_attack(field):
    if field.ball.position.x < -5.0:
        return True
    if field.ball.position.x > 5.0:
        return False
    return race_state_trigger == Team.Self


@unbox_field
def get_placement(field: Field) -> List[Tuple[float, float, float]]:
    global active_set_piece, set_piece_start, set_piece_ball, reset_controllers
    reset_controllers = True
    set_piece_start = field.tick
    set_piece_ball = (field.ball.position.x, field.ball.position.y)
    ball_history.clear()

    if in_shootout:
        attacking = _is_shootout_attack(field)
        active_set_piece = "shootout_attack" if attacking else "shootout_defense"
        return _penalty_placement(field, attacking)

    if race_state == JudgeResultEvent.ResultType.PlaceKick:
        attacking = race_state_trigger == Team.Self
        active_set_piece = "kickoff_attack" if attacking else "kickoff_defense"
        return _kickoff_placement(attacking)

    if race_state == JudgeResultEvent.ResultType.GoalKick:
        attacking = race_state_trigger == Team.Self
        active_set_piece = "goalkick_attack" if attacking else "goalkick_defense"
        return _goalkick_placement(attacking)

    if race_state == JudgeResultEvent.ResultType.PenaltyKick:
        attacking = race_state_trigger == Team.Self
        active_set_piece = "penalty_attack" if attacking else "penalty_defense"
        return _penalty_placement(field, attacking)

    if race_state in (
        JudgeResultEvent.ResultType.FreeKickRightTop,
        JudgeResultEvent.ResultType.FreeKickRightBot,
        JudgeResultEvent.ResultType.FreeKickLeftTop,
        JudgeResultEvent.ResultType.FreeKickLeftBot,
    ):
        attacking = race_state_trigger == Team.Self
        active_set_piece = "freekick_attack" if attacking else "freekick_defense"
        return _freekick_placement()

    active_set_piece = None
    robots = [
        (30.0, -20.0, 180.0),
        (108.0, 0.0, 90.0),
        (30.0, 20.0, 180.0),
        (50.0, -35.0, 180.0),
        (50.0, 35.0, 180.0),
    ]
    return _fmt_placement(robots, (field.ball.position.x, field.ball.position.y, 0.0))


def _attack_target(world):
    ball_x = world.ball.x
    ball_y = world.ball.y
    keeper_y = world.opp_goalkeeper_y()

    if ball_x < -72.0 and abs(ball_y) < 30.0:
        corner = -14.0 if keeper_y >= 0.0 else 14.0
        return OPP_GOAL_X, corner, 140.0

    if ball_x > 25.0 and abs(ball_y) < 20.0:
        return -20.0, _sign(ball_y) * 55.0 if abs(ball_y) > 0.1 else 55.0, 140.0

    corner = -12.0 if keeper_y >= 0.0 else 12.0
    if abs(keeper_y) < 2.0:
        corner = -14.0 if ball_y >= 0.0 else 14.0

    for robot in world.opp_robots:
        if robot.position.x < ball_x and robot.position.x > -100.0:
            if abs(robot.position.y - ball_y) < 12.0:
                corner = 14.0 if ball_y >= 0.0 else -14.0
                break
    return OPP_GOAL_X, corner, 120.0


def _chaser_action(world, index):
    ball_x = world.ball.x
    ball_y = world.ball.y

    if ball_x > 58.0 and abs(ball_y) < 10.0:
        safe_y = 12.0 if ball_y >= 0.0 else -12.0
        return ("move", 55.0, safe_y, 120.0)

    target_x, target_y, power = _attack_target(world)

    if ball_x > 0.0 and world.ball_vx > 1.0:
        intercept_x = ball_x + world.ball_vx * 3.0
        intercept_y = ball_y + world.ball_vy * 3.0
        if intercept_x > 25.0 and abs(intercept_y) < 25.0:
            target_x = -20.0
            target_y = 55.0 if intercept_y >= 0.0 else -55.0
            power = 140.0

    return ("drive", ball_x, ball_y, target_x, target_y, power)


def _support_action(world, index, chaser):
    ball_x = world.ball.x
    ball_y = world.ball.y
    if ball_x < 0.0:
        side = -1.0 if ball_y >= 0.0 else 1.0
        support_y = 25.0 * side
        if abs(support_y) < 10.0:
            support_y = 20.0 * side
        support_x = _clamp(ball_x - 12.0, -50.0, -10.0)
        return ("move", support_x, support_y, 120.0)
    target_x = _clamp(ball_x + 20.0, 25.0, 60.0)
    target_y = _clamp(ball_y * 0.5, -25.0, 25.0)
    target_x, target_y = _safe_own_penalty(target_x, target_y)
    target_x, target_y = _safe_own_area(target_x, target_y)
    return ("move", target_x, target_y, 120.0)


def _defender_action(world, index, primary, chaser):
    ball_x = world.ball.x
    ball_y = world.ball.y

    if ball_x >= 30.0 and abs(ball_y) <= 17.5:
        if primary:
            return ("move", 28.0, _clamp(ball_y * 0.5, -25.0, 25.0), 120.0)
        return ("move", 45.0, 45.0 if ball_y >= 0.0 else -45.0, 120.0)

    if not primary:
        opponent = world.most_advanced_opponent()
        if opponent.position.x > 0.0:
            x = _clamp(opponent.position.x + 10.0, 25.0, 58.0)
            y = _clamp(opponent.position.y, -40.0, 40.0)
            x, y = _safe_own_area(x, y)
            return ("move", x, y, 120.0)

    if ball_x >= 0.0:
        x = _clamp(ball_x + 20.0, 20.0, 28.0)
        y = _clamp(ball_y * 0.6, -25.0, 25.0)
    else:
        x = 25.0
        if primary:
            y = _clamp(ball_y * 0.5, -20.0, 20.0)
        else:
            y = 35.0 if ball_y >= 0.0 else -35.0
    x, y = _safe_own_area(x, y)
    return ("move", x, y, 120.0)


def _gk_action(world):
    ball_x = world.ball.x
    ball_y = world.ball.y
    nearest = min(world.dist_to_ball(index) for index in [0, 2, 3, 4])

    if ball_x > 100.0 and abs(ball_y) < 15.0:
        if ball_x > 106.0 or nearest > 10.0:
            clear_y = 55.0 if ball_y >= 0.0 else -55.0
            return ("drive", ball_x, ball_y, -20.0, clear_y, 140.0)

    if ball_x > 70.0 and abs(ball_y) < 30.0 and nearest > 12.0:
        clear_y = 55.0 if ball_y >= 0.0 else -55.0
        return ("drive", ball_x, ball_y, -20.0, clear_y, 130.0)

    if world.ball_vx > 0.5:
        time_to_line = (KEEPER_X - ball_x) / world.ball_vx
        if 0.0 <= time_to_line <= 12.0:
            target_y = ball_y + world.ball_vy * time_to_line
        else:
            target_y = ball_y * 0.6
    else:
        target_y = ball_y * 0.6

    target_y = _clamp(target_y, -18.0, 18.0)
    return ("move", KEEPER_X, target_y, 110.0)


def _select_roles(world):
    global last_chaser
    candidates = [0, 2, 3, 4]
    ranked = sorted(candidates, key=world.dist_to_ball)
    chaser = ranked[0]
    if last_chaser in candidates:
        if world.dist_to_ball(last_chaser) <= world.dist_to_ball(chaser) + 4.0:
            chaser = last_chaser
    last_chaser = chaser

    rest = [index for index in candidates if index != chaser]
    rest.sort(key=world.dist_to_ball)
    support = rest[0]
    defenders = rest[1:]
    defender_a = max(defenders, key=lambda index: world.self_pred[index][0])
    defender_b = defenders[0] if defenders[1] == defender_a else defenders[1]
    return chaser, support, defender_a, defender_b


def _normal_actions(world):
    actions = [("stop",) for _ in range(5)]
    chaser, support, defender_a, defender_b = _select_roles(world)

    actions[1] = _gk_action(world)
    actions[chaser] = _chaser_action(world, chaser)
    actions[support] = _support_action(world, support, chaser)
    actions[defender_a] = _defender_action(world, defender_a, True, chaser)
    actions[defender_b] = _defender_action(world, defender_b, False, chaser)
    return actions


def _set_piece_active(world):
    global active_set_piece
    if active_set_piece is None:
        return False

    elapsed = world.tick - set_piece_start
    moved = _distance(
        world.ball.x,
        world.ball.y,
        set_piece_ball[0],
        set_piece_ball[1],
    )
    limits = {
        "kickoff_attack": 60,
        "kickoff_defense": 60,
        "goalkick_attack": 80,
        "goalkick_defense": 40,
        "penalty_attack": 40,
        "penalty_defense": 66,
        "freekick_attack": 50,
        "freekick_defense": 40,
    }
    limit = limits.get(active_set_piece, 30)

    defensive = active_set_piece in (
        "kickoff_defense",
        "goalkick_defense",
        "penalty_defense",
        "freekick_defense",
    )

    if defensive:
        if moved > 8.0 or elapsed > limit:
            active_set_piece = None
            return False
        return True

    if moved > 8.0 or elapsed > limit:
        active_set_piece = None
        return False
    return True


def _freekick_attack_target(world):
    if world.ball.x < -30.0:
        keeper_y = world.opp_goalkeeper_y()
        corner = -14.0 if keeper_y >= 0.0 else 14.0
        return OPP_GOAL_X, corner
    return -30.0, 50.0 if world.ball.y >= 0.0 else -50.0


def _set_piece_actions(world):
    actions = [("stop",) for _ in range(5)]
    mode = active_set_piece
    ball_x = world.ball.x
    ball_y = world.ball.y

    if mode == "kickoff_attack":
        side = _choose_wing(world)
        actions[0] = ("drive", ball_x, ball_y, -30.0, side * 35.0, 140.0)
        actions[1] = _gk_action(world)
        actions[2] = ("move", -25.0, side * 30.0, 120.0)
        actions[3] = ("move", -20.0, -side * 30.0, 120.0)
        actions[4] = ("move", 30.0, 0.0, 110.0)
    elif mode == "kickoff_defense":
        if world.tick - set_piece_start < 8:
            actions[1] = _gk_action(world)
        else:
            actions = _normal_actions(world)
    elif mode == "goalkick_attack":
        side = _choose_wing(world)
        actions[0] = ("drive", ball_x, ball_y, 30.0, side * 25.0, 140.0)
        actions[1] = ("move", 108.0, 0.0, 100.0)
        actions[2] = ("move", 30.0, side * 25.0, 120.0)
        actions[3] = ("move", 35.0, -side * 25.0, 120.0)
        actions[4] = ("move", 50.0, 0.0, 110.0)
    elif mode == "goalkick_defense":
        if world.tick - set_piece_start < 10:
            actions[1] = _gk_action(world)
        else:
            actions = _normal_actions(world)
    elif mode == "penalty_attack":
        actions[0] = ("drive", ball_x, ball_y, OPP_GOAL_X, set_piece_aux, 140.0)
        if world.tick - set_piece_start < 10:
            actions[1] = ("stop",)
        else:
            normal = _normal_actions(world)
            for index in (1, 2, 3, 4):
                actions[index] = normal[index]
    elif mode == "penalty_defense":
        actions[1] = _penalty_gk_action(world)
    elif mode == "freekick_attack":
        target_x, target_y = _freekick_attack_target(world)
        actions[0] = ("drive", ball_x, ball_y, target_x, target_y, 140.0)
        actions[1] = _gk_action(world)
        normal = _normal_actions(world)
        for index in (2, 3, 4):
            actions[index] = normal[index]
    elif mode == "freekick_defense":
        target_y = 55.0 if ball_y >= 0.0 else -55.0
        actions[0] = ("drive", ball_x, ball_y, -30.0, target_y, 140.0)
        actions[1] = _gk_action(world)
        normal = _normal_actions(world)
        for index in (2, 3, 4):
            actions[index] = normal[index]
    return actions


def _penalty_gk_action(world):
    ball_x = world.ball.x
    ball_y = world.ball.y

    if world.ball_vx > 0.3:
        time_to_line = (110.0 - ball_x) / world.ball_vx
        if 0.0 <= time_to_line <= 8.0:
            target_y = ball_y + world.ball_vy * time_to_line
        else:
            target_y = ball_y
    else:
        target_y = ball_y * 0.8

    target_y = _clamp(target_y, -18.0, 18.0)

    if ball_x > 104.0 and abs(ball_y) < 20.0:
        clear_y = 50.0 if ball_y >= 0.0 else -50.0
        return ("drive", ball_x, ball_y, -20.0, clear_y, 140.0)
    return ("move", 109.0, target_y, 130.0)


def _prepare_shootout(world):
    global active_set_piece, set_piece_start, set_piece_aux, set_piece_ball

    attacking = _is_shootout_attack(world.field)
    mode = "shootout_attack" if attacking else "shootout_defense"
    at_spot = abs(abs(world.ball.x) - PENALTY_SPOT_X) < 3.0 and abs(world.ball.y) < 5.0

    previous_at_spot = False
    if len(ball_history) >= 2:
        previous_x, previous_y = ball_history[-2]
        previous_at_spot = abs(abs(previous_x) - PENALTY_SPOT_X) < 3.0 and abs(previous_y) < 5.0

    new_shot = active_set_piece != mode or (
        at_spot and not previous_at_spot and world.tick - set_piece_start > 2
    )

    if new_shot:
        active_set_piece = mode
        set_piece_start = world.tick
        set_piece_ball = (world.ball.x, world.ball.y)
        if attacking:
            set_piece_aux = -14.0 if random.random() < 0.5 else 14.0


def _shootout_actions(world):
    actions = [("stop",) for _ in range(5)]
    elapsed = world.tick - set_piece_start
    attacking = _is_shootout_attack(world.field)

    if elapsed <= 66:
        if attacking:
            actions[0] = ("drive", world.ball.x, world.ball.y, OPP_GOAL_X, set_piece_aux, 140.0)
        else:
            actions[1] = _penalty_gk_action(world)
    return actions


def _apply_actions(actions):
    for index, action in enumerate(actions):
        controller = controllers[index]
        if action is None or action[0] == "stop":
            controller.stop()
        elif action[0] == "move":
            controller.move_to(action[1], action[2], max_speed=action[3])
        elif action[0] == "drive":
            allow_goal_area = index == 1 or (active_set_piece == "goalkick_attack" and index == 0)
            controller.drive_through(
                action[1],
                action[2],
                action[3],
                action[4],
                power=action[5],
                allow_goal_area=allow_goal_area,
            )
        else:
            controller.stop()


@unbox_field
def get_instruction(field: Field):
    global last_tick, reset_controllers

    if field is None:
        return [(0.0, 0.0)] * 5, 0

    tick = field.tick
    GV.tick = tick
    if last_tick != -1 and tick < last_tick:
        reset_controllers = True
    last_tick = tick

    _update_ball_history(field.ball.position)

    for index in range(5):
        controllers[index].update(field.self_robots[index], reset_controllers)
    reset_controllers = False

    world = World(field)

    if in_shootout:
        _prepare_shootout(world)
        actions = _shootout_actions(world)
    elif active_set_piece is not None and _set_piece_active(world):
        actions = _set_piece_actions(world)
    else:
        actions = _normal_actions(world)

    _apply_actions(actions)

    for controller in controllers:
        controller.finish_tick()

    velocities = []
    for index in range(5):
        robot = controllers[index].robot
        velocities.append((robot.wheel.left_speed, robot.wheel.right_speed))
    return velocities, 0
