import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "v5strategy"))

import v5strategy as strategy
from V5RPC import Ball, EventArguments, Field, JudgeResultEvent, Robot, Team, Vector2, Wheel


def make_robot(x=0.0, y=0.0, rotation=0.0):
    return Robot(Vector2(x, y), rotation, Wheel(0.0, 0.0))


def make_field(tick=100, ball=(0.0, 0.0), self_positions=None, opp_positions=None):
    field = Field()
    field.tick = tick
    field.ball = Ball()
    field.ball.position = Vector2(ball[0], ball[1])
    if self_positions is None:
        self_positions = [(30.0, -20.0), (108.0, 0.0), (30.0, 20.0), (50.0, -35.0), (50.0, 35.0)]
    if opp_positions is None:
        opp_positions = [(-30.0, -20.0), (-108.0, 0.0), (-30.0, 20.0), (-50.0, -35.0), (-50.0, 35.0)]
    field.self_robots = [make_robot(x, y, 180.0) for x, y in self_positions]
    field.opponent_robots = [make_robot(x, y, 0.0) for x, y in opp_positions]
    return field


def trigger_event(event_type, result_type=None, trigger=None):
    args = EventArguments()
    if result_type is not None:
        result = JudgeResultEvent()
        result.type = result_type
        result.offensive_team = trigger
        result.reason = "test"
        args.judge_result = result
    strategy.on_event(event_type, args)


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def check_bounds(placement):
    for x, y, rotation in placement:
        check(-110.0 <= x <= 110.0, f"x out of bounds: {x}")
        check(-90.0 <= y <= 90.0, f"y out of bounds: {y}")
        check(-180.0 <= rotation <= 180.0, f"rotation out of bounds: {rotation}")


def in_circle(x, y, radius=25.0):
    return math.hypot(x, y) < radius


def in_goal_area(x, y):
    return x >= 60.0 and abs(y) <= 7.5


def in_penalty_area(x, y):
    return x >= 30.0 and abs(y) <= 17.5


def in_quarter(x, y, qx, qy):
    if qx > 0:
        x_ok = x > 0.0
    else:
        x_ok = x < 0.0
    if qy > 0:
        y_ok = y > 0.0
    else:
        y_ok = y < 0.0
    return x_ok and y_ok


def test_kickoff():
    trigger_event(0, 0, Team.Self)
    placement = strategy.get_placement(make_field())
    check_bounds(placement)
    robots = placement[:5]
    inside = [r for r in robots if in_circle(r[0], r[1])]
    check(len(inside) == 1, f"kickoff attack inside circle: {inside}")
    for r in robots:
        if r is not inside[0]:
            check(r[0] >= 0.0, f"kickoff attack robot not in own half: {r}")

    trigger_event(0, 0, Team.Opponent)
    placement = strategy.get_placement(make_field())
    check_bounds(placement)
    for r in placement[:5]:
        check(r[0] >= 0.0, f"kickoff defense robot not in own half: {r}")
        check(not in_circle(r[0], r[1]), f"kickoff defense robot in circle: {r}")


def test_goalkick():
    trigger_event(0, 1, Team.Self)
    placement = strategy.get_placement(make_field())
    check_bounds(placement)
    ball = placement[5]
    check(in_goal_area(ball[0], ball[1]), f"goal kick ball not in goal area: {ball}")
    check(any(in_goal_area(r[0], r[1]) for r in placement[:5]), "goal kick no robot in goal area")

    trigger_event(0, 1, Team.Opponent)
    placement = strategy.get_placement(make_field())
    check_bounds(placement)
    for r in placement[:5]:
        check(r[0] >= 0.0, f"goal kick defense robot not in own half: {r}")


def test_penalty():
    trigger_event(0, 2, Team.Self)
    placement = strategy.get_placement(make_field())
    check_bounds(placement)
    check(abs(placement[5][0] + 72.5) < 1e-6, f"penalty attack ball wrong: {placement[5]}")
    attackers = [r for r in placement[:5] if r[0] < 0.0]
    check(len(attackers) == 1, f"penalty attack must have one player in opponent half: {attackers}")
    for r in placement[:5]:
        if r is not attackers[0]:
            check(r[0] >= 0.0, f"penalty attack other robot not in own half: {r}")

    trigger_event(0, 2, Team.Opponent)
    placement = strategy.get_placement(make_field())
    check_bounds(placement)
    check(abs(placement[5][0] - 72.5) < 1e-6, f"penalty defense ball wrong: {placement[5]}")
    keeper = placement[1]
    check(keeper[0] >= 109.0, f"penalty defense keeper not on line: {keeper}")
    for i, r in enumerate(placement[:5]):
        if i != 1:
            check(r[0] <= 0.0, f"penalty defense other robot not in opponent half: {r}")


def test_freekick():
    cases = [
        (3, 55.0, 60.0),
        (4, 55.0, -60.0),
        (5, -55.0, 60.0),
        (6, -55.0, -60.0),
    ]
    for state, fb_x, fb_y in cases:
        for trigger in (Team.Self, Team.Opponent):
            trigger_event(0, state, trigger)
            placement = strategy.get_placement(make_field())
            check_bounds(placement)
            check(abs(placement[5][0] - fb_x) < 1e-6 and abs(placement[5][1] - fb_y) < 1e-6, f"free kick ball wrong: {placement[5]}")
            taker = placement[0]
            check(abs(taker[0] - (fb_x + 25.0)) < 1e-6 and abs(taker[1] - fb_y) < 1e-6, f"free kick taker wrong: {taker}")
            qx = 1.0 if fb_x > 0 else -1.0
            qy = 1.0 if fb_y > 0 else -1.0
            for index in (2, 3, 4):
                r = placement[index]
                check(not in_quarter(r[0], r[1], qx, qy), f"free kick robot in quarter: {r}")


def test_shootout_placement():
    trigger_event(6)
    placement = strategy.get_placement(make_field(ball=(-72.5, 0.0)))
    check_bounds(placement)
    check(abs(placement[5][0] + 72.5) < 1e-6, f"shootout attack ball wrong: {placement[5]}")
    attackers = [r for r in placement[:5] if r[0] < 0.0]
    check(len(attackers) == 1, f"shootout attack players wrong: {attackers}")

    placement = strategy.get_placement(make_field(ball=(72.5, 0.0)))
    check_bounds(placement)
    check(abs(placement[5][0] - 72.5) < 1e-6, f"shootout defense ball wrong: {placement[5]}")
    check(placement[1][0] >= 109.0, f"shootout defense keeper not on line: {placement[1]}")


def test_instruction_loop():
    trigger_event(0, 0, Team.Self)
    field = make_field(tick=2)
    strategy.get_placement(field)
    for tick in range(2, 1200):
        field.tick = tick
        field.ball.position.x += random.uniform(-0.6, 0.6)
        field.ball.position.y += random.uniform(-0.6, 0.6)
        field.ball.position.x = max(-110.0, min(110.0, field.ball.position.x))
        field.ball.position.y = max(-90.0, min(90.0, field.ball.position.y))
        for i, robot in enumerate(field.self_robots):
            robot.position.x += random.uniform(-0.4, 0.4)
            robot.position.y += random.uniform(-0.4, 0.4)
            robot.position.x = max(-110.0, min(110.0, robot.position.x))
            robot.position.y = max(-90.0, min(90.0, robot.position.y))
            robot.rotation = (robot.rotation + random.uniform(-5.0, 5.0)) % 360.0
            if robot.rotation > 180.0:
                robot.rotation -= 360.0
        velocities, reset = strategy.get_instruction(field)
        check(len(velocities) == 5, "velocity count")
        check(reset == 0, "reset flag")
        for left, right in velocities:
            check(-140.0 <= left <= 140.0, f"left speed out of range: {left}")
            check(-140.0 <= right <= 140.0, f"right speed out of range: {right}")
        check(len(strategy.controllers) == 5, "controller list grew")


def test_all_set_pieces_instruction():
    cases = [
        (0, 0, Team.Self),
        (0, 0, Team.Opponent),
        (0, 1, Team.Self),
        (0, 1, Team.Opponent),
        (0, 2, Team.Self),
        (0, 2, Team.Opponent),
        (0, 3, Team.Self),
        (0, 3, Team.Opponent),
        (0, 4, Team.Self),
        (0, 4, Team.Opponent),
        (0, 5, Team.Self),
        (0, 5, Team.Opponent),
        (0, 6, Team.Self),
        (0, 6, Team.Opponent),
        (6, None, None),
    ]
    for event_type, state, trigger in cases:
        if event_type == 6:
            trigger_event(6)
            field = make_field(tick=100, ball=(-72.5, 0.0))
        else:
            trigger_event(event_type, state, trigger)
            field = make_field(tick=100)
        strategy.get_placement(field)
        for tick in range(100, 180):
            field.tick = tick
            velocities, _ = strategy.get_instruction(field)
            check(len(velocities) == 5, f"velocity count for {state}/{trigger}")
            for left, right in velocities:
                check(-140.0 <= left <= 140.0, f"left speed out of range for {state}/{trigger}: {left}")
                check(-140.0 <= right <= 140.0, f"right speed out of range for {state}/{trigger}: {right}")
        trigger_event(1)


def main():
    random.seed(12345)
    test_kickoff()
    test_goalkick()
    test_penalty()
    test_freekick()
    test_shootout_placement()
    test_instruction_loop()
    test_all_set_pieces_instruction()
    print("ALL_TESTS_PASSED")


if __name__ == "__main__":
    main()

