'''
这个文件是主要开发文件，涵盖了策略全部的四个接口
-on_event接收比赛状态变化的信息。
    参数event_type type表示事件类型；
    参数EventArgument表示该事件的参数，如果不含参数，则为NULL。
-get_team_info控制队名。
    修改返回值的字符串即可修改自己的队名
-get_instruction控制5个机器人的轮速(leftspeed,rightspeed)，以及最后的reset(1即表明需要reset)
    通过返回值来给机器人赋轮速
    比赛中的每拍被调用，需要策略指定轮速，相当于旧接口的Strategy。
    参数field为In/Out参数，存储当前赛场信息，并允许策略修改己方轮速。
    ！！！所有策略的开发应该在此模块
-get_placement控制5个机器人及球在需要摆位时的位置
    通过返回值来控制机器人和球的摆位。
    每次自动摆位时被调用，需要策略指定摆位信息。
    定位球类的摆位需要符合规则，否则会被重摆
'''
import random
from typing import Tuple, Union

import numpy as np
from V5RPC import *
import math
from baseRobot import *
from GlobalVariable import *

baseRobots = []# 定义我方机器人数组
oppRobots = []# 定义对方机器人数组
data_loader = DataLoader()
race_state = -1  # 定位球状态
race_state_trigger = -1    # 触发方
num_goalkepper = -1         #对方守门员编号 1代表有守门员  其余为有
penalty = 1               #点球的决策树 目前有三种
flag_penalty = -1                 #如果为1 则换点球策略
defend_flag = 0           #0为正常防守  1为点球防守

goal_time = 0
time = 0                  #统计门球造犯规失败次数
goal = 1                  #门球的决策树 目前有两种
flag_goal = 0             #如果为1 则换门球策略
last_race_state = -1      #上一次比赛情况
last_race_state_trigger = -1  #上一次触发方
guard_goal = 1            #门球防守的决策树  防止被对方造犯规

last_futuerballx = 0
last_futuerbally = 0
tickBeginPenalty = 0
tickBeginGoalKick = 0
tickBeginPlaceKick = 0
lastBallx = -110 + 37.5
lastBally = 0
BallPos = [Vector2(0, 0)] * 100000
resetHistoryRecord = False
newMatch = False

# 打印比赛状态，详细请对比v5rpc.py
@unbox_event
def on_event(event_type: int, args: EventArguments):
    event = {
        0: lambda: print(args.judge_result.reason),
        1: lambda: print("Match Start"),
        2: lambda: print("Match Stop"),
        3: lambda: print("First Half Start"),
        4: lambda: print("Second Half Start"),
        5: lambda: print("Overtime Start"),
        6: lambda: print("Penalty Shootout Start"),
        7: lambda: print("MatchShootOutStart"),
        8: lambda: print("MatchBlockStart")
    }
    global race_state_trigger
    global race_state
    if event_type == 0:
        race_state = args.judge_result.type
        race_state_trigger = args.judge_result.offensive_team
        if race_state == JudgeResultEvent.ResultType.PlaceKick:
            print("Place Kick")
        elif race_state == JudgeResultEvent.ResultType.PenaltyKick:
            print("Penalty Kick")
        elif race_state == JudgeResultEvent.ResultType.GoalKick:
            print("Goal Kick")
        elif (race_state == JudgeResultEvent.ResultType.FreeKickLeftBot
              or race_state == JudgeResultEvent.ResultType.FreeKickRightBot
              or race_state == JudgeResultEvent.ResultType.FreeKickLeftTop
              or race_state == JudgeResultEvent.ResultType.FreeKickRightTop):
            print("Free Kick")

        actor = {
            Team.Self: lambda: print("By Self"),
            Team.Opponent: lambda: print("By Opp"),
            Team.Nobody: lambda: print("By Nobody"),
        }
        actor[race_state_trigger]()
    elif event_type == 6:
        race_state = 7 #点球大战

    event[event_type]()


@unbox_int
def get_team_info(server_version: int) -> str:
    version = {
        0: "V1.0",
        1: "V1.1"
    }
    print(f'server rpc version: {version.get(server_version, "V1.0")}')
    global newMatch
    newMatch = True #标记一场新比赛的开始
    return '赤足1.01'# 在此行修改双引号中的字符串为自己的队伍名

'守门员函数'
def goalkeeper(field):
    # if baseRobots[0].get_pos().x >= 110 - 2.4: #守门员到球门里面去了
    #     baseRobots[0].moveto(110 - 1.5, futureBally)
    # else:
    global football_now_y,football_now_x
    global futureBally,futureBallx
    global last_futuerballx

    football_now_x= field.ball.position.x
    football_now_y = field.ball.position.y

    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y

    dx = futureBallx - baseRobots[1].get_pos().x
    dy = futureBally - baseRobots[1].get_pos().y
    distance = math.sqrt(dx * dx + dy * dy)

    global defend_flag

    num_in_goal = 0
    for i in range(0,5): #判断有没有对方守门员
        if oppRobots[i].get_pos().x >= 75 and np.fabs(oppRobots[i].get_pos().y) <= 40: #如果有
            num_in_goal = 1 + num_in_goal

    if np.fabs(futureBally) <= 45: #球在球门y轴范围内
        if futureBallx < 0:  #球离球门还有一段距离
            if football_now_y*0.8 + futureBally*0.2 >= 20:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 18)
            elif football_now_y*0.8 + futureBally*0.2 <= -20:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -18)
            else:
                baseRobots[1].moveto(GlobalVariable.goalkepper_X, football_now_y*0.8 + futureBally*0.2)
        else:               #球在我们半场
            if defend_flag == 1:  #点球防守
                baseRobots[1].moveto(GlobalVariable.goalkepper_X, futureBally)
            else:                 #正常防守
                if (baseRobots[1].get_pos().x-futureBallx > 0) and (futureBallx>=65):  #守门员在球后面
                    baseRobots[1].moveto(futureBallx, futureBally)  #把球踢出去
                else:
                    baseRobots[1].moveto(GlobalVariable.goalkepper_X, futureBally)
                    if futureBally >= 20:
                        baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 18)
                    elif futureBally <= -20:
                        baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -18)
    else:       #不在球门y轴范围
        if futureBallx <= 0:
            if futureBally > 0:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 18)
            else:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -18)
        elif futureBallx <=93:
            if futureBally >= 20:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 15)
            elif futureBally <= -20:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -15)
            else:  #球门范围内
                baseRobots[1].moveto(GlobalVariable.goalkepper_X, futureBally)
        else:
            baseRobots[1].moveto(futureBallx, futureBally)
            if futureBallx >= baseRobots[1].get_pos().x: #球x轴距离比守门员大则不动
                baseRobots[1].breakthrough(futureBallx, futureBally, futureBallx+3, futureBally)

    last_futuerballx = futureBallx

"""
def goalkeeper(field):
    # if baseRobots[0].get_pos().x >= 110 - 2.4: #守门员到球门里面去了
    #     baseRobots[0].moveto(110 - 1.5, futureBally)
    # else:
    global football_now_y,football_now_x #存储当前足球位置
    global futureBally,futureBallx #未来足球位置
    global last_futuerballx #上一次预测足球位置

    football_now_x= field.ball.position.x
    football_now_y = field.ball.position.y

    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y
    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y

    dx = futureBallx - baseRobots[1].get_pos().x
    dy = futureBally - baseRobots[1].get_pos().y
    distance = math.sqrt(dx * dx + dy * dy)

    global defend_flag

    num_in_goal = 0
    for i in range(0,5): #判断有没有对方守门员
        if oppRobots[i].get_pos().x >= 75 and np.fabs(oppRobots[i].get_pos().y) <= 40: #如果有
            num_in_goal = 1 + num_in_goal

    # 新增逻辑：判断是否为斜对角线射向球门边角的情况
    is_diagonal_shot = (
        (futureBallx < 0) and 
        (np.fabs(futureBally) in [19, 20, 21]) and  # 考虑一定的误差范围
        (np.fabs(football_now_y - futureBally) > 10) and  # 球有较大的y轴移动
        (np.fabs(football_now_x - futureBallx) > 10)  # 球有较大的x轴移动
    )

    if is_diagonal_shot:
        # 提前预判并移动到合适的位置
        if futureBally > 0:
            target_y = 20
        else:
            target_y = -20
        baseRobots[1].moveto(GlobalVariable.goalkepper_X, target_y)
    elif np.fabs(futureBally) <= 45: #球在球门y轴范围内
        if futureBallx < 0:  #球离球门还有一段距离--球还在对方半场
            if football_now_y*0.70 + futureBally*0.30 >= 20:   #0.8 0.2
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 18) #18
            elif football_now_y*0.70 + futureBally*0.30 <= -20:  #0.8 0.2
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -18) #-18
            else:
                baseRobots[1].moveto(GlobalVariable.goalkepper_X, football_now_y*0.70 + futureBally*0.30)
        else:               #球在我们半场
            if defend_flag == 1:  #点球防守
                baseRobots[1].moveto(GlobalVariable.goalkepper_X, futureBally)
            else:                 #正常防守
                if (baseRobots[1].get_pos().x-futureBallx > 0) and (futureBallx>=65):  #守门员在球后面
                    baseRobots[1].moveto(futureBallx, futureBally)  #把球踢出去
                elif(baseRobots[1].get_pos().x-futureBallx < 0) and (futureBallx>=85):  # 球在守门员身后
                    # 计算球后方的位置
                    target_x = futureBallx - 5  # 假设在球后方3个单位
                    target_y = futureBally
                    baseRobots[1].moveto(target_x, target_y)
                else:
                    baseRobots[1].moveto(GlobalVariable.goalkepper_X, futureBally)
                    if futureBally >= 20:
                        baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 18)  #18
                    elif futureBally <= -20:
                        baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -18)  #-18
    else:       #不在球门y轴范围
        if futureBallx <= 0: #对方半场
            if futureBally > 0: #上半区
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 18) #18
            else: #下半区
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -18) #18
        elif futureBallx <=93:
            if futureBally >= 20:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 15) #15
            elif futureBally <= -20:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -15) #15
            else:  #球门范围内
                baseRobots[1].moveto(GlobalVariable.goalkepper_X, futureBally)
        else:
            baseRobots[1].moveto(futureBallx, futureBally)
            if futureBallx >= baseRobots[1].get_pos().x: #球x轴距离比守门员大则不动
                baseRobots[1].breakthrough(futureBallx, futureBally, futureBallx+3, futureBally)

    last_futuerballx = futureBallx
    """
    
# 策略行为主函数，可将以下函数用策略模式封装
def strategy_common(field):      #正常的时候
    # 最基本最常规情况下的执行策略
    global football_now_x,football_now_y

    football_now_x= field.ball.position.x
    football_now_y = field.ball.position.y
    # 预测足球位置
    global futureBallx,futureBally

    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y

    # 获取所有机器人位置用于避障
    obstacles = []
    for i in range(5):
        if i != 0:  # 不考虑自己
            obstacles.append((field.opponent_robots[i].position.x,
                            field.opponent_robots[i].position.y))

    # print ('dis',baseRobots[1].PredictInformation[GlobalVariable.tick_delay].position.x,'tick',GlobalVariable.tick)
    # baseRobots[1].moveto_dis (110,20)
    #1号一直作为防守人员   2号控场  0号主要进攻
    '********************************防守***********************************************************'
    if football_now_x > 5:

        # if baseRobots[1].PredictInformation[GlobalVariable.tick_delay].position.x > 85 or np.fabs(baseRobots[1].PredictInformation[GlobalVariable.tick_delay].position.y) < 25 + 3: #防止进入守门区

        #     # if np.fabs(baseRobots[1].get_pos().x - 85) > np.fabs(np.fabs(baseRobots[1].get_pos().y) - 25):  #离y轴近
        #     #     if baseRobots[1].get_pos().y > 0:
        #     #         baseRobots[1].moveto (80,40)
        #     #     else :
        #             #  baseRobots[1].moveto (80,-40)
        #     # else :            #离x轴近
        #     baseRobots[1].moveto (80,baseRobots[1].get_pos().y)
        # else :
        # if  baseRobots[1].get_pos ().x > football_now_x:
        if (futureBallx > 80 or np.fabs(futureBally) < 40) or (football_now_x > 80 or np.fabs(football_now_y) < 40): #防止进入守门区
            baseRobots[0].moveto_dis (65,futureBally*0.8)
        else :
            # 计算目标位置并避障
            target_x, target_y = futureBallx, futureBally
            if futureBallx > baseRobots[1].get_pos().x:
                target_x += 3  # 保持3单位距离
                target_x, target_y = baseRobots[0].avoid_obstacles(
                    target_x, target_y, obstacles)
                baseRobots[0].breakthrough(futureBallx, futureBally,
                                          target_x, target_y)
            else:
                target_x, target_y = baseRobots[0].avoid_obstacles(
                    futureBallx, futureBally, obstacles)
                baseRobots[0].shoot(target_x, target_y, 10)



        # print (baseRobots[3].PredictInformation[GlobalVariable.tick_delay].position.x)
        # print (baseRobots[3].PredictInformation[GlobalVariable.tick_delay].position.x)
        if (futureBallx > 70.5 or np.fabs(futureBally) < 40) or (
                football_now_x > 70 or np.fabs(football_now_y) < 40):  # 防止进入守门区
            baseRobots[3].moveto_dis(60, -70)  # 将机器人3移动到x=60，y=-70的位置
        else:
            # 根据足球的y轴位置，决定机器人3的防守位置
            if football_now_y >= 0:  # 如果足球在y轴正方向
                defense_y = 40  # 设置防守位置在y轴正方向
            else:  # 如果足球在y轴负方向
                defense_y = -40  # 设置防守位置在y轴负方向

            if baseRobots[0].get_pos().x > football_now_x and football_now_x > baseRobots[
                3].get_pos().x:  # 3号机器人挡住了0号主攻的路线就避开
                # 根据足球的y轴方向设置防守位置
                if football_now_y >= 0:
                    baseRobots[3].moveto(futureBallx, defense_y)  # 移动到未来球x位置，y轴正方向防守位置
                else:
                    baseRobots[3].moveto(futureBallx, defense_y)  # 移动到未来球x位置，y轴负方向防守位置
            else:
                if football_now_x > baseRobots[3].get_pos().x:  # 如果足球当前x坐标大于机器人3的x坐标
                    target_x, target_y = futureBallx + 3, futureBally  # 目标位置x坐标为未来球x+3，保持距离
                    target_x, target_y = baseRobots[3].avoid_obstacles(  # 调用机器人3的避障方法更新目标位置
                        target_x, target_y, obstacles)
                    baseRobots[3].breakthrough(futureBallx, futureBally,  # 机器人3突破到目标位置
                                               target_x, target_y)
                else:
                    target_x, target_y = baseRobots[3].avoid_obstacles(  # 调用机器人3的避障方法更新目标位置
                        futureBallx, futureBally, obstacles)
                    # 根据足球的y轴方向设置防守位置
                    if football_now_y >= 0:
                        target_y = defense_y  # 设置目标y坐标为防守位置（正方向）
                    else:
                        target_y = defense_y  # 设置目标y坐标为防守位置（负方向）
                    baseRobots[3].moveto(target_x, target_y)  # 机器人3移动到目标位置

        if (futureBallx > 70.5 or np.fabs(futureBally) < 40) or (
                football_now_x > 70 or np.fabs(football_now_y) < 40):  # 防止进入守门区
            baseRobots[4].moveto_dis(60, 70)  # 将机器人4移动到x=60，y=70的位置
        else:
            # 根据足球的y轴位置，决定机器人4的防守位置
            if football_now_y >= 0:  # 如果足球在y轴正方向
                defense_y = 70  # 设置防守位置在y轴正方向
            else:  # 如果足球在y轴负方向
                defense_y = -70  # 设置防守位置在y轴负方向

            if football_now_x > baseRobots[4].get_pos().x:  # 如果足球当前x坐标大于机器人4的x坐标
                target_x, target_y = futureBallx + 1, futureBally  # 目标位置x坐标为未来球x+1，保持距离
                target_x, target_y = baseRobots[4].avoid_obstacles(  # 调用机器人4的避障方法更新目标位置
                    target_x, target_y, obstacles)
                # 根据足球的y轴方向设置防守位置
                if football_now_y >= 0:
                    target_y = defense_y  # 设置目标y坐标为防守位置（正方向）
                else:
                    target_y = defense_y  # 设置目标y坐标为防守位置（负方向）
                baseRobots[4].breakthrough(football_now_x, football_now_y,  # 机器人4突破到目标位置
                                           target_x, target_y)
            else:
                target_x, target_y = baseRobots[4].avoid_obstacles(  # 调用机器人4的避障方法更新目标位置
                    futureBallx, futureBally, obstacles)
                # 根据足球的y轴方向设置防守位置
                if football_now_y >= 0:
                    target_y = defense_y  # 设置目标y坐标为防守位置（正方向）
                else:
                    target_y = defense_y  # 设置目标y坐标为防守位置（负方向）
                baseRobots[4].moveto(target_x, target_y)  # 机器人4移动到目标位置



        goalkeeper (field)

        if futureBallx > baseRobots[2].get_pos ().x :
             baseRobots[2].move_in_still_x  (55,-futureBally)
        else:
            baseRobots[2].move_in_still_x  (55,football_now_y*0.8 + futureBally*0.2)  #2号后卫起一个拦截作用
            dx = football_now_x - baseRobots[2].get_pos().x
            dy = football_now_y - baseRobots[2].get_pos().y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance < 2 and dx > 0:
                if football_now_y < 0:
                    baseRobots[2].set_wheel_velocity(125, -125)
                else:
                    baseRobots[2].set_wheel_velocity(-125, 125)

    '********************************进攻***********************************************************'
    if football_now_x <= 5:
        if  futureBallx+1 > -75 + 3 :
            baseRobots[0].shoot (futureBallx+1,futureBally,10)
            baseRobots[3].move_with_angle(futureBallx+1, futureBally,180)  # 4号自由人追球
        else :
            num_in_goal = 0
            for i in range(0,5): #判断有没有对方守门员
                if oppRobots[i].get_pos().x < -94 and  np.fabs(oppRobots[i].get_pos().y) < 30: #如果有
                    num_in_goal = 1 +  num_in_goal

            if num_in_goal == 1:  #有守门员
                distance1_ball = math.sqrt((football_now_x - baseRobots[0].get_pos().x) ** 2 + (football_now_y - baseRobots[0].get_pos().y) ** 2)
                if futureBallx < -80 and distance1_ball < 10:  #在球门区而且球在跟前
                    if baseRobots[0].get_pos ().x <= futureBallx :  #0号在球的左侧
                         baseRobots[0].breakthrough (futureBallx,futureBally,futureBallx+3,futureBally)
                    else :
                        if football_now_y < 0:
                            baseRobots[0].set_wheel_velocity(125, -125)
                        else:
                            baseRobots[0].set_wheel_velocity(-125, 125)
                else :
                    # baseRobots[0].shoot (futureBallx+1,futureBally,10)
                    baseRobots[0].shoot(futureBallx, futureBally,10)
                    if (futureBallx<-90 and np.fabs(futureBally)<=60):
                        baseRobots[0].moveto_dis (-80,futureBally)

                if baseRobots[3].get_pos ().x <= futureBallx :  #0号在球的左侧
                    baseRobots[3].breakthrough (futureBallx,futureBally,futureBallx+3,futureBally)
                else:
                    baseRobots[3].move_with_angle(futureBallx+1, futureBally,180)
                distance3_ball = math.sqrt((futureBallx - baseRobots[3].get_pos().x) ** 2 + (futureBally - baseRobots[3].get_pos().y) ** 2)
                if (baseRobots[3].get_pos().x > futureBallx and distance3_ball<10):
                    if football_now_y < 0:
                        baseRobots[3].set_wheel_velocity(125, -125)
                    else:
                        baseRobots[3].set_wheel_velocity(-125, 125)
                if (futureBallx<-90 and np.fabs(futureBally)<=50):
                    baseRobots[3].moveto_dis (-75,futureBally)
            else :               #没有守门员
                baseRobots[0].shoot (futureBallx,futureBally,10)
                # baseRobots[3].shoot(futureBallx, futureBally,10)


        distance2_ball = math.sqrt((football_now_x - baseRobots[2].get_pos().x) ** 2 + (football_now_y - baseRobots[2].get_pos().y) ** 2)
        if distance2_ball < 5 and  (baseRobots[2].get_pos().x - football_now_x) > 0 : #求到眼前了就中场射门
            if football_now_y < 0:
                baseRobots[2].set_wheel_velocity(125, -125)
            else:
                baseRobots[2].set_wheel_velocity(-125, 125)
        else :
                baseRobots[2].move_in_still_x( 45,  football_now_y*0.8 + futureBally*0.2)

        distance4_ball = math.sqrt((football_now_x - baseRobots[4].get_pos().x) ** 2 + (football_now_y - baseRobots[4].get_pos().y) ** 2)
        if distance4_ball < 5 and  (baseRobots[4].get_pos().x - football_now_x) > 0 : #求到眼前了就中场射门
            if football_now_y < 0:
                baseRobots[4].set_wheel_velocity(125, -125)
            else:
                baseRobots[4].set_wheel_velocity(-125, 125)
        else :
             baseRobots[4].move_in_still_x( 0,  futureBally)

        goalkeeper (field)


def strategy_penalty(field): #点球策略
    football_now_x = field.ball.position.x
    football_now_y = field.ball.position.y
    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y
    global tickBeginPenalty
    global race_state_trigger
    global flag_penalty,penalty
    goalkeeper_num = 0

    if race_state_trigger == Team.Self:
        for i in range(0, 5):
            baseRobots[i].set_wheel_velocity(0, 0)
        # print(np.fabs(baseRobots[1].get_pos().x))
        if  penalty == 1 :   #点球策略1： 守门员定点射门#
            if GlobalVariable.tick - tickBeginPenalty <=30:
                if np.fabs(baseRobots[0].get_pos().x) < 64.5001:
                    baseRobots[0].set_wheel_velocity(-125,-125)
                else:
                    if direction_flag1 == 1 :
                        baseRobots[0].set_wheel_velocity(135,-135)
                    elif direction_flag1 == 2 :
                        baseRobots[0].set_wheel_velocity(-135,135)                        
            else:
                flag_penalty = 1 #30拍没进球就是点球没进  换策略2
                strategy_common(field)
        elif penalty == 2:       #点球策略2： 守门员绕C字#
            if direction_flag1 == 1 :
                if GlobalVariable.tick - tickBeginPenalty <= 8:
                    baseRobots[0].set_wheel_velocity(-110, -82)
                elif GlobalVariable.tick - tickBeginPenalty <= 19:
                    baseRobots[0].set_wheel_velocity(-95, -125)
                elif GlobalVariable.tick - tickBeginPenalty <= 28:
                    baseRobots[0].set_wheel_velocity(-10, -115)
                elif GlobalVariable.tick - tickBeginPenalty <= 68:
                    baseRobots[0].shoot(futureBallx, futureBally,12)
                else:
                    flag_penalty = 1
                    strategy_common(field)
            elif direction_flag1 == 2 :
                if GlobalVariable.tick - tickBeginPenalty <= 8:
                    baseRobots[0].set_wheel_velocity(-82, -115) #-110,-77
                elif GlobalVariable.tick - tickBeginPenalty <= 19:
                    baseRobots[0].set_wheel_velocity(-125, -95) #-90,-125
                elif GlobalVariable.tick - tickBeginPenalty <= 28:
                    baseRobots[0].set_wheel_velocity(-115, -10) #-10, -125
                elif GlobalVariable.tick - tickBeginPenalty <= 68:
                    baseRobots[0].shoot(futureBallx, futureBally,10)
                else:
                    flag_penalty = 1
                    strategy_common(field)
        elif penalty == 3:    #3号战术，队友辅助射门
          if GlobalVariable.tick - tickBeginPenalty <= 65:
            baseRobots[3].set_wheel_velocity(80, 80)
            if  8 > np.fabs(baseRobots[4].get_pos().x) :
                baseRobots[0].set_wheel_velocity(80, 80)
                baseRobots[4].set_wheel_velocity(125,125)
            elif 8 < np.fabs(baseRobots[4].get_pos().x) < 31:
                baseRobots[4].set_wheel_velocity(105, 125)
                if 12 < np.fabs(baseRobots[4].get_pos().x) < 31:
                    baseRobots[0].set_wheel_velocity(40,20)
            elif  50 > np.fabs(baseRobots[4].get_pos().x) > 31:
                baseRobots[0].set_wheel_velocity(65,45)
                baseRobots[4].set_wheel_velocity(125,125)
            elif  np.fabs(baseRobots[4].get_pos().x) > 50:
                baseRobots[4].set_wheel_velocity(125,-125)
          else:
                flag_penalty = 1
                strategy_common(field)
        elif penalty == 4:    #4号战术，队友辅助射门
          if GlobalVariable.tick - tickBeginPenalty <= 55:
            if  10 > np.fabs(baseRobots[3].get_pos().x) :
                baseRobots[0].set_wheel_velocity(90, 90)
                baseRobots[3].set_wheel_velocity(125,125)
            elif 10 < np.fabs(baseRobots[3].get_pos().x) < 35:
                baseRobots[3].set_wheel_velocity(102.7, 125)
                if 12 < np.fabs(baseRobots[3].get_pos().x) < 35:
                    baseRobots[0].set_wheel_velocity(40,20)
            elif 60 > np.fabs(baseRobots[3].get_pos().x) > 35:
                baseRobots[0].set_wheel_velocity(65,45)
                baseRobots[3].set_wheel_velocity(125,125)
          else:
                flag_penalty = 1
                strategy_common(field)
        elif penalty == 5:    #点球策略5： 简易c字
          if GlobalVariable.tick - tickBeginPenalty <=55:
             if  np.fabs(baseRobots[0].get_pos().x) < 69:
                baseRobots[0].set_wheel_velocity(91,95)
             elif  74 > np.fabs(baseRobots[0].get_pos().x) > 69:
                baseRobots[0].set_wheel_velocity(60,95)
             elif  np.fabs(baseRobots[0].get_pos().x) > 74:
                baseRobots[0].set_wheel_velocity(125,-125)
          else:
                flag_penalty = 1
                strategy_common(field)

    if race_state_trigger == Team.Opponent:
        if GlobalVariable.tick - tickBeginPenalty <= 60 :
            baseRobots[0].moveto_dis(50, -15)
            baseRobots[2].moveto_dis(50, 15)
            baseRobots[3].moveto_dis(70, -65)
            baseRobots[4].moveto_dis(70, 65)
            goalkeeper (field)
        else:
            strategy_common(field)

def strategy_goalkick(field):  #门球策略
    global tickBeginGoalKick
    global race_state_trigger
    global goal,guard_goal

    football_now_x = field.ball.position.x
    football_now_y = field.ball.position.y
    # 预测足球位置
    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y

    if race_state_trigger == Team.Self:
        if goal == 1:  #造犯规策略
            for i in range(2, 5):
                baseRobots[i].set_wheel_velocity(125, -125)
            # baseRobots[0].set_wheel_velocity(125,125)
            baseRobots[1].set_wheel_velocity(0, 0)  #守门员

        elif goal == 2:  #正常策略
            if GlobalVariable.tick - tickBeginGoalKick <= 75:
                for i in range(0, 5):
                    baseRobots[i].set_wheel_velocity(0, 0)
                baseRobots[1].set_wheel_velocity(125, 125)
            else:
                strategy_common(field)
        elif goal == 3: #开球直踢型策略
            for i in range(0, 5):
                baseRobots[i].set_wheel_velocity(0, 0)
            if GlobalVariable.tick - tickBeginGoalKick <= 30:
                 baseRobots[1].set_wheel_velocity(125, 125)
            elif GlobalVariable.tick - tickBeginGoalKick <= 45:
                 baseRobots[1].set_wheel_velocity(125, -125)
            else:
                 strategy_common(field)

    if race_state_trigger == Team.Opponent:

        if guard_goal == 1:
            if GlobalVariable.tick - tickBeginGoalKick <= 80:
                baseRobots[0].shoot (futureBallx,futureBally,10)
                baseRobots[2].move_in_still_x  (65,futureBally)  #2号后卫起一个拦截作用
                baseRobots[3].moveto(25, futureBally)
                baseRobots[4].moveto(50, futureBally)
                goalkeeper (field)
            else:
                strategy_common(field)
        elif guard_goal == 2:  #防止被对方造犯规
            goalkeeper (field)
            if futureBallx >= - 55:
                baseRobots[0].shoot  (futureBally,futureBally,10)
            else:
                baseRobots[0].moveto (-5, futureBally)

            baseRobots[2].move_in_still_x  (65,futureBally*0.9)  #2号后卫起一个拦截作用
            baseRobots[3].moveto(25, football_now_y)
            baseRobots[4].moveto(50, football_now_y)

def strategy_PlaceKick (field): #开球策略             开球改动地方
    global tickBeginPlaceKick
    global race_state_trigger

    football_now_x = field.ball.position.x
    football_now_y = field.ball.position.y
    # 预测足球位置
    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y

    if race_state_trigger == Team.Self:
        for i in range(2, 5):
            baseRobots[i].set_wheel_velocity(0, 0)
        #if GlobalVariable.tick - tickBeginPlaceKick <= 20:
            #baseRobots[0].set_wheel_velocity (125,125)
        if GlobalVariable.tick - tickBeginPlaceKick <= 70:

            baseRobots[0].set_wheel_velocity(140,140)
            #if football_now_y < 0:
             #   baseRobots[0].set_wheel_velocity(125, -125)
            #else:
             #   baseRobots[0].set_wheel_velocity(-125, 125)
            goalkeeper (field)
        else:
            strategy_common(field)
    if race_state_trigger == Team.Opponent:
        if GlobalVariable.tick - tickBeginPlaceKick <= 80:
            strategy_common(field)
            # baseRobots[0].shoot (futureBallx,futureBally,10)
            baseRobots[2].set_wheel_velocity  (0,0)  #2号后卫起一个拦截作用
            baseRobots[3].set_wheel_velocity(0, 0)
            baseRobots[4].set_wheel_velocity(0, 0)
            goalkeeper (field)
        else:
            strategy_common(field)

@unbox_field
def get_instruction(field: Field):  #策略接口
    # python start.py 20000    print(field.tick)  # tick从2起始
    GlobalVariable.tick = field.tick
    global resetHistoryRecord

    for i in range(0, 5): # 0 1 2 3 4
        baseRobots.append(BaseRobot())
        oppRobots.append(BaseRobot())
        baseRobots[i].update(field.self_robots[i], resetHistoryRecord)
        oppRobots[i].update(field.opponent_robots[i], resetHistoryRecord)
        global newMatch
        if field.tick == 2: #newMatch is True:
            for j in range(0, 8):
                baseRobots[i].HistoryInformation[j] = field.self_robots[i].copy()   # 第0拍主动维护历史数据
                baseRobots[i].PredictInformation[j] = field.self_robots[i].copy()	# 第0拍主动维护预测数据
            newMatch = False
        baseRobots[i].PredictRobotInformation(GlobalVariable.tick_delay)#(GlobalVariable.tick_delay)

    football_now_x = field.ball.position.x   # 蓝方假设
    football_now_y = field.ball.position.y

    field.ball.position.x = field.ball.position.x
    field.ball.position.y = field.ball.position.y

    global BallPos
    BallPos[GlobalVariable.tick] = Vector2(football_now_x, football_now_y)
    if resetHistoryRecord is True:
        for i in range(GlobalVariable.tick, GlobalVariable.tick - 11, -1):
            BallPos[i] = Vector2(football_now_x, football_now_y)
    #print (race_state)
    #根据情况执行什么样的策略
    if race_state == JudgeResultEvent.ResultType.PenaltyKick:
        strategy_penalty(field)  #点球策略 2
    elif race_state == JudgeResultEvent.ResultType.GoalKick:
        strategy_goalkick(field) #门球策略 1
    elif race_state == JudgeResultEvent.ResultType.PlaceKick:
        strategy_PlaceKick (field) #开球策略0
    else :
        strategy_common(field)


    for i in range(0, 5):
        baseRobots[i].save_last_information(football_now_x, football_now_y)
    data_loader.set_tick_state(GlobalVariable.tick, race_state)
    resetHistoryRecord = False

    velocity_to_set = []
    for i in range(0, 5):
        velocity_to_set.append((baseRobots[i].robot.wheel.left_speed, baseRobots[i].robot.wheel.right_speed))

    return velocity_to_set, 0    # 以第二元素的(0,1)表明重置开关,1表示重置


@unbox_field
def get_placement(field: Field) -> List[Tuple[float, float, float]]:  #返回摆球时的坐标 接口
    final_set_pos: List[Union[Tuple[int, int, int], Tuple[float, float, float]]]
    global resetHistoryRecord, flag_penalty, penalty, flag_goal, goal
    global last_race_state, last_race_state_trigger, guard_goal
    global defend_flag, time, goal_time
    global tickBeginPlaceKick, tickBeginPenalty, tickBeginGoalKick

    resetHistoryRecord = True
    defend_flag = 0

    from GlobalVariable import GlobalVariable as GV
    # 场地边界计算
    half_length = GV.FIELD_LENGTH / 2
    half_width = GV.FIELD_WIDTH / 2

    '点球决策树'
    if  flag_penalty == 1 :
        penalty = penalty + 1
        if penalty > 5:
            penalty = 1
    flag_penalty = -1

    '点球决策树'

    '门球决策树'
    if flag_goal == 1:  #进入门球摆位了置1
        #1.造犯规成功了继续造犯规策略  2.对方连续犯了5次
        if last_race_state == race_state or (race_state == JudgeResultEvent.ResultType.PlaceKick and race_state_trigger != Team.Self and goal_time>=3):
            goal = 1
            goal_time = goal_time + 1
            if goal_time>=3:
                goal_time = 0
        #1.对方不犯规  2.其他没有造成功的情况
        else:    #两种点球策略谁进球了就继续
            if race_state == JudgeResultEvent.ResultType.PlaceKick and race_state_trigger != Team.Self: #上一种门球进球了
                # goal = goal‘
                flag_goal = 0
            else:
                goal = goal+1
                if goal >= 4:
                    if time >= 3:
                        goal = 2
                    else:
                        goal = 1
                time = time + 1

    flag_goal = 0

    #连续两次次触发对方门球 则说明对方在造犯规
    if (last_race_state == 1 and race_state == 1) and (last_race_state_trigger != Team.Self and race_state != Team.Self):
            guard_goal = 2
    else :
            guard_goal = 1

    '门球决策树'

    if race_state == JudgeResultEvent.ResultType.PlaceKick:
        global tickBeginPlaceKick
        global direction_flag2 #开球摆位标志
        direction_flag2 = random.randint(1,2)
        tickBeginPlaceKick = field.tick
        if race_state_trigger == Team.Self:
            print("开球进攻摆位")#        开球改动地方
            if direction_flag2 == 1 :
                set_pos = [
                        [-13, 6.2,-25],
                        [GlobalVariable.goalkepper_X, 0, 90],#守门员
                        [5, -90, 180],
                        [50, 0, 180],
                        [44, -84, 150],
                        [0.0, 0.0, 0.0]]
            elif direction_flag2 == 2 :
                set_pos = [
                        [-13, -6.2,25],
                        [GlobalVariable.goalkepper_X, 0, 90],#守门员
                        [5, 90, -180],
                        [50, 0, 180],
                        [44, 84, -150],
                        [0.0, 0.0, 0.0]]                   
            # set_pos = [(-103, 0, 90), (30, 0, 0), (-3, -10, 0), (-3, 10, 0), (-3, 0, 0), (0.0, 0.0, 0.0)]
        else:   # if race_state_trigger == Team.Opponent:
            print("开球防守摆位")
            if direction_flag2 == 1 :
                set_pos = [
                        [30, 0, 180],
                        [GlobalVariable.goalkepper_X, 0, 90],
                        [70, -13, 0],
                        [70, 85, 180],
                        [70, 13, 0],
                        [0.0, 0.0, 0.0]]
            elif direction_flag2 == 2 :
                set_pos = [
                        [30, 0, 180],
                        [GlobalVariable.goalkepper_X, 0, 90],
                        [70, -13, 0],
                        [70, -85, -180],
                        [70, 13, 0],
                        [0.0, 0.0, 0.0]]                       
            # set_pos = [(-105, 0, 90), (10, 20, -90), (10, -20, -90), (10, 40, -90), (10, -40, -90), (0.0, 0.0, 0.0)]
    elif race_state == JudgeResultEvent.ResultType.PenaltyKick:
        global tickBeginPenalty
        tickBeginPenalty = field.tick
        if race_state_trigger == Team.Self:
            global direction_flag1
            direction_flag1 = random.randint(1,2)
            print("点球进攻摆位")
            if penalty == 1:#直接射门策略摆位
                if direction_flag1 == 1 :
                    set_pos = [
                        [-64.5,6,35],
                        [GlobalVariable.goalkepper_X, 0, 90],
                        [0, 50, 180],
                        [10, -50, 160],
                        [0, 0, -180],
                        [-5, -10, 0.0]]
                elif direction_flag1 == 2 :
                    set_pos = [
                        [-64.5,-6,-35],
                        [GlobalVariable.goalkepper_X, 0, 90],
                        [0, 50, 180],
                        [10, -50, 160],
                        [0, 0, -180],
                        [-5, -10, 0.0]]                    
            elif penalty == 2:   #复杂c字种策略摆位   
                if direction_flag1 == 1 :      
                    set_pos = [
                            [-64, -1.2, -8.6],
                            [GlobalVariable.goalkepper_X, 0, 90],
                            [10, -70, 14],
                            [10, 60, -30],
                            [10, -60, 14],
                            [-5, 10, 0.0]]
                elif direction_flag1 == 2 :
                    set_pos = [
                            [-64, 1.2, 8.6], #-64,-1.2,-8.6
                            [GlobalVariable.goalkepper_X, 0, 90],
                            [10, -70, 14],
                            [10, 60, -30],
                            [10, -60, 14],
                            [-5, 10, 0.0]]                    
            elif penalty == 3:#队友辅助射门策略摆位
                set_pos = [
                       [-76.5, -9, 70],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [5, 55, -180],
                       [5, 75, -165],
                       [5, 85,-165],
                       [-5, -10, 0.0]]     
            elif penalty == 4:#队友辅助射门策略摆位
                set_pos = [
                      [-76.5, -9, 70],
                      [GlobalVariable.goalkepper_X, 0, 90],
                       [0, 50, 180],
                       [5, 60,-170],
                       [0, 0, -180],
                       [-5, -10, 0.0]]     
            elif penalty == 5:#简易c字策略摆位
                set_pos = [
                       [-67.5, -8, 90],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [0, 50, 180],
                       [10, -50, 160],
                       [0, 0, -180],
                       [-5, -10, 0.0]]    
            
        else:   # if race_state_trigger == Team.Opponent:
            defend_flag = 1
            print("点球防守摆位")
            set_pos = [
                       [-10, 50,-40],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [-10, -40,40],
                       [-7, -65,-15],
                       [-7, 65, 15],
                       [0, 0.0, 0.0]]
    elif race_state == JudgeResultEvent.ResultType.GoalKick:
        global tickBeginGoalKick
        tickBeginGoalKick = field.tick
        if race_state_trigger == Team.Self:
            flag_goal = 1

            print("门球进攻摆位")
            if goal == 1:
                set_pos = [
                        [15, -10, 30],
                        [100, 9.05, 90],
                        [5, -85, 90],
                        [30, 85, -90],
                        [30, -85, 90],
                        [105, 9.05, 0.0]]
            elif goal == 2:
                set_pos = [
                        [50, -20, 90],
                        [102, 5.05, 90],
                        [50, -40, 90],
                        [30, 0, 90],
                        [50, 0, 90],
                        [104.5, 9.05, 0.0]] 
            elif goal == 3:
                 set_pos = [
                        [5, 85, 90],
                        [106, -4, 180],
                        [5, -80, 90],
                        [30, 80, 90],
                        [30, -80, 90],
                        [98, -4, 0.0]]                                
        else:   # if race_state_trigger == Team.Opponent:
            print("门球防守摆位")
            set_pos = [
                       [5, 0, 180],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [65, -40, 180],
                       [25, 40, 180],
                       [50, 0, 180],
                       [0.0, 0.0, 0.0]]
    elif (race_state == JudgeResultEvent.ResultType.FreeKickLeftTop
          or race_state == JudgeResultEvent.ResultType.FreeKickRightTop
          or race_state == JudgeResultEvent.ResultType.FreeKickRightBot
          or race_state == JudgeResultEvent.ResultType.FreeKickLeftBot):
        if race_state_trigger == Team.Self:
            print("争球进攻摆位")
            set_pos = [
                       [30, 0, 0],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [-3, -10, 0],
                       [-3, 10, 0],
                       [-3, 0, 0],
                       [0.0, 0.0, 0.0]]
        else:   # if race_state_trigger == Team.Opponent:
            print("争球防守摆位")
            set_pos = [
                       [30, 0, 0],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [10, -10, 0],
                       [10, 10, 0],
                       [10, 0, 0],
                       [0.0, 0.0, 0.0]]
    else:
        print("race_state = " + str(race_state))

    final_set_pos = [(set_pos[0][0], set_pos[0][1], set_pos[0][2]),
                     (set_pos[1][0], set_pos[1][1], set_pos[1][2]),
                     (set_pos[2][0], set_pos[2][1], set_pos[2][2]),
                     (set_pos[3][0], set_pos[3][1], set_pos[3][2]),
                     (set_pos[4][0], set_pos[4][1], set_pos[4][2]),
                     (set_pos[5][0], set_pos[5][1], set_pos[5][2])]

    last_race_state = race_state   #记录上一次比赛状态
    last_race_state_trigger  = race_state_trigger

    print(final_set_pos)
    return final_set_pos  # 最后一个是球位置（x,y,角）,角其实没用
