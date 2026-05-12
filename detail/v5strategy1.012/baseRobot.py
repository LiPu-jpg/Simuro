from V5RPC import *
import math
import numpy as np
from GlobalVariable import *
import v5strategy

last_angle_dif = 0
last_dis_dif = 0


class PID:
    lastError = 0 
    preError = 0
    #存储历史误差值，用于计算误差变化率
    now_error = 0 #当前误差值
    lastU = 0 #上一次的控制输出量

    def __init__(self, proportion, integral, derivative): #初始化PID的三个参数
        self.proportion = proportion  # 比例常数——它用于调整当前误差对控制输出的影响程度。
        # 较大的比例常数会使系统对误差的响应更加迅速，但可能会导致系统超调或不稳定。
        self.integral = integral  # 积分常数_通过对误差的积分来消除长时间存在的误差
        # 积分项会随着时间的积累而增加，从而逐渐减小稳态误差。
        self.derivative = derivative  # 微分常数——通过对误差的微分来抑制系统的振荡和超调
        # 微分项可以提前对误差的变化做出反应，使系统更加稳定。

    def pid_cal(self, now_error): #计算当前误差下的PID输出值
        self.now_error = now_error
        now_pid = self.lastU + (self.proportion * (now_error - self.lastError) + self.integral * now_error
                                + self.derivative * (now_error - 2 * self.lastError + self.preError))

        return now_pid

    def update_pid(self): #更新
        self.lastU += (self.proportion * (self.now_error - self.lastError) + self.integral * self.now_error
                       + self.derivative * (self.now_error - 2 * self.lastError + self.preError))
        self.preError = self.lastError
        self.lastError = self.now_error

    def reset_pid(self):
        self.lastError = 0
        self.preError = 0
        self.lastU = 0


# 老PID
class OLD_PID:
    def __init__(self, proportion, integral, derivative, lasterror, pre_error):
        self.proportion = proportion  # 比例常数
        self.integral = integral  # 积分常数
        self.derivative = derivative  # 微分常数
        self.lastError = lasterror  # 上一时刻的误差
        self.preError = pre_error  # 两个时刻前的误差

    def pid_cal(self, nowPoint):
        # 计算误差，这里是指角度误差
        # pid的最终目的是让机器人的运动朝向角度与所要跑位的位置角度差为0，也就是直勾勾跑到指定位置
        now_error = 0 - nowPoint

        # 计算当前次的输出
        """
        1.比例项：将当前误差减去上一次误差后，乘一个比例系数，来调节最终输出
        2.积分项：将当前误差乘以给定的积分系数，也就是误差积累来调节输出
        3.微分项：（当前误差 - 2*上一次误差 + 上上次误差）* 微分系数
        """
        now_pid = (self.proportion * (now_error - self.lastError) + self.integral * now_error
                   + self.derivative * (now_error - 2 * self.lastError + self.preError))

        # 更新误差
        self.preError = self.lastError
        self.lastError = now_error
        return now_pid

    def tune_parameters(self, oscillation_period):
        # 计算临界比例度
        Kcr = self.proportion
        # 根据系统类型选择参数整定规则
        self.proportion = 0.5 * Kcr
        self.integral = 1.2 / oscillation_period
        self.derivative = 0.075 * oscillation_period


class BaseRobot:
    def __init__(self):
        self.robot: Robot
        self.lastTargetX = 0
        self.lastTargetY = 0  # 机器人上一拍目标点
        self.lastRobotX = 0
        self.lastRobotY = 0  # 机器人上一拍位置
        self.lastRotation = 0
        self.tick = GlobalVariable.tick

        self.lastU = 0  # pid控制变量U
        self.lastU1 = 0  # pid控制变量U1

        self.HistoryInformation = [Robot(Vector2(0, 0), 0, Wheel(0, 0))] * 8
        self.PredictInformation = [Robot(Vector2(0, 0), 0, Wheel(0, 0))] * 8
        self.AngularSpeed = [0] * 8

        self.reset = False
        self.pid_oldMoveto = OLD_PID(0.25, 0, 3.8, 0, 0)
        # self.pid_moveto = PID(1.9, 0, 2.2)
        """self.pid_moveto = PID(2.0, 0, 3.0)  # 降低P,增加D
        self.pid_moveto_dis = PID(1.8, 0, 8.0)  # 降低P,增加D
        self.pid_angle = PID(1.5, 0, 5.0)  # 降低P,增加D"""

        self.pid_moveto = PID(2.0, 0, 3.0)  # 降低P,增加D
        self.pid_moveto_dis = PID(1.8, 0, 8.0)  # 降低P,增加D
        self.pid_angle = PID(1.5, 0, 5.0)  # 降低P,增加D

    def update(self, env_robot: Robot, resetHistoryRecord):
        self.robot = env_robot
        self.reset = resetHistoryRecord
        '''
         # 代码黄方假设，但vda是蓝方假设，所以把所有坐标反转，旋转角也反转，从而可以是赋的轮速不变从而达到相同效果
        self.robot.position.x = -self.robot.position.x
        self.robot.position.y = -self.robot.position.y
        self.robot.rotation = self.robot.rotation - 180
        if self.robot.rotation < -180:
            self.robot.rotation += 360     
        '''

        if self.reset is True:
            for i in range(0, 8):
                self.HistoryInformation[i] = self.robot.copy()
        else:
            self.HistoryInformation[0] = self.robot.copy()

    def save_last_information(self, footBallNow_X, footBallNow_Y):  # 保存机器人本拍信息，留作下一拍使用，可用于拓展保存更多信息
        self.lastRotation = self.robot.rotation
        self.lastRobotX = self.robot.position.x
        self.lastRobotY = self.robot.position.y
        self.lastBallx = footBallNow_X
        self.lastBally = footBallNow_Y

        # self.HistoryInformation.pop(0)

        for i in range(7, 0, -1):
            self.HistoryInformation[i] = self.HistoryInformation[i - 1].copy()
        self.HistoryInformation[1] = self.robot.copy()

        self.pid_moveto.update_pid()
        self.pid_angle.update_pid()
        return

    def get_pos(self):  # 获取机器人位置
        return self.robot.position

    def get_last_pos(self):  # 获取机器人上一拍位置
        last_pos = Vector2(self.lastRobotX, self.lastRobotY)
        return last_pos

    def get_rotation(self):  # 获取机器人旋转角
        return self.robot.rotation

    def get_left_wheel_velocity(self):  # 获取机器人左轮速
        return self.robot.wheel.left_speed

    def get_right_wheel_velocity(self):  # 获取机器人右轮速
        return self.robot.wheel.right_speed

    def set_wheel_velocity(self, vl, vr):  # 直接赋左右轮速
        self.robot.wheel.left_speed = vl
        self.robot.wheel.right_speed = vr

    def moveto_dis(self, tar_x, tar_y):
        global last_angle_dif, last_dis_dif

        if self.tick == 1 or self.tick == 2 or self.tick == 3 or self.tick % 50 == 0 or self.reset is True:  # 比赛重启刚开始 #100
            last_angle_dif = 0
            last_dis_dif = 0
            self.pid_moveto.reset_pid()
        v_max = 130

        dx = tar_x - self.PredictInformation[GlobalVariable.tick_delay].position.x
        dy = tar_y - self.PredictInformation[GlobalVariable.tick_delay].position.y
        angle_to = 180.0 / math.pi * np.arctan2(dy, dx)
        angle_diff = angle_to - self.PredictInformation[GlobalVariable.tick_delay].rotation

        dis_diff = math.sqrt(dx * dx + dy * dy)

        'delta_v是转向换的速度'
        # if last_angle_dif != 0 :
        #     angle_diff = (angle_diff + last_angle_dif)/2

        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        #v_max = 0.3 * dis_diff + 1.1 * (dis_diff - last_dis_dif)
        v_max = 0.3 * dis_diff + 1.1 * (dis_diff - last_dis_dif)
        # v_max =  22*dis_diff
        if v_max >= 130:
            v_max = 130
        if math.fabs(angle_diff) < 90:
            delta_v = self.pid_moveto.pid_cal(angle_diff)
            v_r = v_max + delta_v
            v_l = v_max - delta_v
        elif math.fabs(angle_diff) >= 90:
            angle_diff += 180
            if angle_diff > 180:
                angle_diff -= 360
            delta_v = self.pid_moveto.pid_cal(angle_diff)
            v_r = -v_max + delta_v
            v_l = -v_max - delta_v
        # print ('v_max'+str(v_max),'delta_v'+str(delta_v))
        last_angle_dif = angle_diff
        last_dis_dif = dis_diff

        self.set_wheel_velocity(v_l, v_r)

    def is_position_valid(self, x, y):
        """检查位置是否在场地有效区域内"""
        from GlobalVariable import GlobalVariable as GV
        half_length = GV.FIELD_LENGTH / 2
        half_width = GV.FIELD_WIDTH / 2

        # 检查是否在场地边界内
        in_field = (-half_length <= x <= half_length and
                    -half_width <= y <= half_width)

        # 检查是否在禁区外(简单实现)
        in_penalty = False
        if GV.PENALTY_AREA_LENGTH > 0 and GV.PENALTY_AREA_WIDTH > 0:
            penalty_x = half_length - GV.PENALTY_AREA_LENGTH
            penalty_y = GV.PENALTY_AREA_WIDTH / 2
            in_penalty = (x > penalty_x and abs(y) < penalty_y)

        return in_field and not in_penalty

    def adjust_to_boundary(self, x, y):
        """将位置调整到场地边界内"""
        from GlobalVariable import GlobalVariable as GV
        half_length = GV.FIELD_LENGTH / 2
        half_width = GV.FIELD_WIDTH / 2

        # 调整到边界内
        x = max(-half_length, min(x, half_length))
        y = max(-half_width, min(y, half_width))

        # 避开禁区(简单实现)
        if GV.PENALTY_AREA_LENGTH > 0 and GV.PENALTY_AREA_WIDTH > 0:
            penalty_x = half_length - GV.PENALTY_AREA_LENGTH
            penalty_y = GV.PENALTY_AREA_WIDTH / 2
            if x > penalty_x and abs(y) < penalty_y:
                x = penalty_x - 1  # 保持1单位安全距离

        return x, y

    def avoid_obstacles(self, target_x, target_y, obstacles):
                    #简单避障算法
        from GlobalVariable import GlobalVariable as GV
        safe_distance = 4  # 安全距离

        for (obs_x, obs_y) in obstacles:
            dx = target_x - obs_x
            dy = target_y - obs_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < safe_distance:
                # 计算避开方向
                angle = math.atan2(dy, dx)
                target_x = obs_x + safe_distance * math.cos(angle)
                target_y = obs_y + safe_distance * math.sin(angle)

        return target_x, target_y

    """def avoid_obstacles(self, target_x, target_y, obstacles):
        #优化后的避障算法
        from GlobalVariable import GlobalVariable as GV
        safe_distance = 4  # 安全距离
        max_attempts = 3  # 最大避障尝试次数

        for attempt in range(max_attempts):
            new_target_x = target_x
            new_target_y = target_y
            for (obs_x, obs_y) in obstacles:
                dx = new_target_x - obs_x
                dy = new_target_y - obs_y
                dist_squared = dx * dx + dy * dy

                if dist_squared < safe_distance * safe_distance:
                    # 计算避开方向
                    angle = math.atan2(dy, dx)

                    # 尝试左右两侧避障
                    left_angle = angle + math.pi / 2
                    right_angle = angle - math.pi / 2

                    left_x = obs_x + safe_distance * math.cos(left_angle)
                    left_y = obs_y + safe_distance * math.sin(left_angle)
                    right_x = obs_x + safe_distance * math.cos(right_angle)
                    right_y = obs_y + safe_distance * math.sin(right_angle)

                    # 选择更优的避障方向
                    left_dist = float('inf')
                    right_dist = float('inf')
                    for (obs_x2, obs_y2) in obstacles:
                        left_dx = left_x - obs_x2
                        left_dy = left_y - obs_y2
                        left_dist = min(left_dist, math.sqrt(left_dx * left_dx + left_dy * left_dy))

                        right_dx = right_x - obs_x2
                        right_dy = right_y - obs_y2
                        right_dist = min(right_dist, math.sqrt(right_dx * right_dx + right_dy * right_dy))

                    if left_dist > right_dist:
                        new_target_x = left_x
                        new_target_y = left_y
                    else:
                        new_target_x = right_x
                        new_target_y = right_y

            # 如果新目标点没有改变，说明已经避开障碍物
            if new_target_x == target_x and new_target_y == target_y:
                break

            target_x = new_target_x
            target_y = new_target_y

        return target_x, target_y"""
    
    def moveto(self, tar_x, tar_y):  # pid跑位函数
        if self.tick == 1 or self.tick == 2 or self.tick == 3 or self.tick % 50 == 0 or self.reset is True: #100
            self.pid_moveto.reset_pid()

        # 检查目标位置有效性
        if not self.is_position_valid(tar_x, tar_y):
            tar_x, tar_y = self.adjust_to_boundary(tar_x, tar_y)

        v_max = GlobalVariable.MAX_WHEEL_SPEED

        dx = tar_x - self.PredictInformation[GlobalVariable.tick_delay].position.x
        dy = tar_y - self.PredictInformation[GlobalVariable.tick_delay].position.y
        angle_to = 180.0 / math.pi * np.arctan2(dy, dx)
        angle_diff = angle_to - self.PredictInformation[GlobalVariable.tick_delay].rotation

        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        if math.fabs(angle_diff) < 90:
            delta_v = self.pid_moveto.pid_cal(angle_diff)
            v_r = v_max + delta_v
            v_l = v_max - delta_v
        elif math.fabs(angle_diff) >= 90:
            angle_diff += 180
            if angle_diff > 180:
                angle_diff -= 360
            delta_v = self.pid_moveto.pid_cal(angle_diff)
            v_r = -v_max + delta_v
            v_l = -v_max - delta_v
        else:
            v_r = 120     #80
            v_l = -120   #-80

        self.set_wheel_velocity(v_l, v_r)

    def turntoangle(self, tar_angle):
        if self.tick == 1 or self.tick == 2 or self.tick == 3 or self.tick % 50 == 0 or self.reset is True: #100
            self.pid_moveto.reset_pid()
        v_max = 0 #将最大轮速 v_max 设置为 0，这意味着机器人在转向过程中不会有向前或向后的直线运动，只会进行旋转。
        angle_diff = tar_angle - self.PredictInformation[GlobalVariable.tick_delay].rotation

        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        if math.fabs(angle_diff) < 90:  #
            delta_v = self.pid_angle.pid_cal(angle_diff)
            v_r = v_max + delta_v
            v_l = v_max - delta_v
        elif math.fabs(angle_diff) >= 90:  # 背身跑过去
            angle_diff += 180
            if angle_diff > 180:
                angle_diff -= 360
            delta_v = self.pid_angle.pid_cal(angle_diff)
            v_r = -v_max + delta_v
            v_l = -v_max - delta_v
        else:
            v_r = 100  #80
            v_l = -100  #-80
        self.set_wheel_velocity(v_l, v_r)

    def move_with_angle(self, tarx, tary, tar_angle):
        dx = tarx - self.PredictInformation[GlobalVariable.tick_delay].position.x
        dy = tary - self.PredictInformation[GlobalVariable.tick_delay].position.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > 0.5:
            self.moveto(tarx, tary)
        else:
            self.turntoangle(tar_angle)

    def throw_ball(self, ballx, bally, dis):
        dx = ballx - self.PredictInformation[GlobalVariable.tick_delay].position.x
        dy = bally - self.PredictInformation[GlobalVariable.tick_delay].position.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > dis or dx > 0:  # 离球距离大于10 或在右面则绕到左面
            self.move_with_angle(ballx + 1, bally, 180)
        else:
            if bally < 0:
                self.set_wheel_velocity(130, -130)
            else:
                self.set_wheel_velocity(-130, 130)

    def shoot(self, ballx, bally, dis):  # 射门函数 球在-72.5右侧则追球进攻 -72.5左侧调用抛球函数射球

        if ballx > -72.5:
            self.moveto(ballx, bally)
        else:  # 球门区  射门
            self.throw_ball(ballx, bally, dis)

    """def breakthrough(self, ballx, bally, tarx, tary):
        # 动态参数设置
        if self.robot.position.y > 0:  # 上方机器人使用更激进的参数
            future_time = 14
            len_val = 5
            deltay = 4
        else:  # 下方机器人使用更保守的参数
            future_time = 10
            len_val = 4
            deltay = 3
        
        # 预测球的未来位置 (使用线性预测模型)
        dx = ballx - self.lastBallx if hasattr(self, 'lastBallx') else 0
        dy = bally - self.lastBally if hasattr(self, 'lastBally') else 0
        future_x = ballx + dx * future_time
        future_y = bally + dy * future_time
        
        # 初始移动
        self.moveto(tarx, tary)
        
        # 根据球的位置和预测位置调整
        if (self.robot.position.x < ballx < tarx
                or self.robot.position.x > ballx > tarx):
            if self.robot.position.y > 0:
                if future_y + len_val > self.robot.position.y:
                    # 调整目标位置并确保不越界 (使用场地宽度作为边界)
                    max_y = GlobalVariable.FIELD_WIDTH / 2 - 10  # 留出安全余量
                    new_tary = min(tary + deltay, max_y)
                    self.moveto(tarx, new_tary)
            if self.robot.position.y < 0:
                if future_y - len_val < self.robot.position.y:
                    # 调整目标位置并确保不越界 (使用场地宽度作为边界)
                    min_y = -GlobalVariable.FIELD_WIDTH / 2 + 10  # 留出安全余量
                    new_tary = max(tary - deltay, min_y)
                    self.moveto(tarx, new_tary)"""
    
    def breakthrough(self, ballx, bally, tarx, tary):
        self.moveto(tarx, tary)
        future_time = 14
        len = 5
        deltay = 4

        futureOppy = future_time * bally - (future_time - 1) * ballx
        if (self.robot.position.x < ballx < tarx
                or self.robot.position.x > ballx > tarx):
            if self.robot.position.y > 0:
                if futureOppy + len > self.robot.position.y:
                    self.moveto(tarx, tary + deltay)
            if self.robot.position.y < 0:
                if futureOppy - len < self.robot.position.y:
                    self.moveto(tarx, tary - deltay)

    def move_in_still_x(self, still_x, football_y):  # x不变
        self.moveto(still_x, football_y)

    def PredictRobotInformation(self, tick_delay):
        delta_t = 1
        self.PredictInformation[0] = self.robot.copy()

        self.AngularSpeed[0] = self.AngularSpeed[1]
        for i in range(1, tick_delay + 1):
            PredictedlastRotation = self.GetRobotInformation(i - 1).rotation
            PredictedLastPos = self.GetRobotInformation(i - 1).position.copy()
            # PredictedLineSpeed = (PredictedLastPos - self.GetRobotInformation(i - 2).position) / delta_t # i-1拍线速度
            # PredictedLineSpeed = math.sqrt(pow(PredictedLastPos.x - self.GetRobotInformation(i - 2).position.x, 2) + pow(PredictedLastPos.y - self.GetRobotInformation(i - 2).position.y, 2))

            dx = PredictedLastPos.x - self.GetRobotInformation(i - 2).position.x
            dy = PredictedLastPos.y - self.GetRobotInformation(i - 2).position.y

            PredictedLineSpeed = math.sqrt(dx * dx + dy * dy)

            angle_logic = math.atan2(dy, dx) * 180 / math.pi  # 逻辑角度
            if PredictedLastPos.x < self.GetRobotInformation(i - 2).position.x:
                if angle_logic > 0:
                    angle_logic = 180 - angle_logic
                else:
                    angle_logic = -180 - angle_logic
            delta_angle = angle_logic - self.GetRobotInformation(i - 1).rotation
            if delta_angle > 180:
                delta_angle -= 360
            if delta_angle < -180:
                delta_angle += 360
            if math.fabs(delta_angle) <= 90:  # 基于蓝方
                PredictedLineSpeed = -PredictedLineSpeed

            if (self.AngularSpeed[i - 1] > 0):
                if PredictedlastRotation > self.GetRobotInformation(i - 2).rotation:
                    PiedictedAngularSpeed = PredictedlastRotation - self.GetRobotInformation(i - 2).rotation
                else:
                    PiedictedAngularSpeed = (360 + PredictedlastRotation - self.GetRobotInformation(i - 2).rotation)
            else:
                if PredictedlastRotation > self.GetRobotInformation(i - 2).rotation:
                    PiedictedAngularSpeed = PredictedlastRotation - self.GetRobotInformation(i - 2).rotation - 360
                else:
                    PiedictedAngularSpeed = (PredictedlastRotation - self.GetRobotInformation(i - 2).rotation)

            settedWheelSpeed = self.HistoryInformation[tick_delay + 1 - i].wheel.copy()
            if settedWheelSpeed.left_speed > 130:
                settedWheelSpeed.left_speed = 130
            elif settedWheelSpeed.left_speed < -130:
                settedWheelSpeed.left_speed = -130
            if settedWheelSpeed.right_speed > 130:
                settedWheelSpeed.right_speed = 130
            elif settedWheelSpeed.right_speed < -130:
                settedWheelSpeed.right_speed = -130

            K1 = 0.002362192
            K2 = math.exp(-1 / 0.9231)
            K3 = 1 - math.exp(-1 / 3.096)
            K4 = 0.53461992

            if settedWheelSpeed.left_speed != 0 and settedWheelSpeed.right_speed != 0:
                nextLineSpeed = PredictedLineSpeed * 0.939534127623834133 + (
                            settedWheelSpeed.left_speed + settedWheelSpeed.right_speed) / 2 * K1
            if settedWheelSpeed.left_speed == 0 or settedWheelSpeed.right_speed == 0:
                nextLineSpeed = PredictedLineSpeed * K2

            nextAngularSpeed = PiedictedAngularSpeed + ((
                                                                    settedWheelSpeed.right_speed - settedWheelSpeed.left_speed) / 2 * K4 - PiedictedAngularSpeed) * K3
            self.AngularSpeed[i] = nextAngularSpeed

            newLineSpeed = nextLineSpeed  # 先算线速度然后根据线速度算出位置？
            newAngularSpeed = nextAngularSpeed  # 先算角速度然后根据角速度算出角度？（假设角速度ni时针为正）

            self.PredictInformation[i].position.x = self.PredictInformation[i - 1].position.x + newLineSpeed * math.cos(
                self.PredictInformation[i - 1].rotation / 180 * math.pi)
            self.PredictInformation[i].position.y = self.PredictInformation[i - 1].position.y + newLineSpeed * math.sin(
                self.PredictInformation[i - 1].rotation / 180 * math.pi)
            self.PredictInformation[i].rotation = self.PredictInformation[i - 1].rotation + newAngularSpeed

            # print(newLineSpeed)
            # print(math.cos(self.PredictInformation[i-1].rotation / 180 * math.pi))
            # print(self.PredictInformation[i].position.y)
            while self.PredictInformation[i].rotation > 180:
                self.PredictInformation[i].rotation -= 360
            while self.PredictInformation[i].rotation < -180:
                self.PredictInformation[i].rotation += 360

    def GetRobotInformation(self, time):
        if time >= 0:
            return self.PredictInformation[time]
        else:
            return self.HistoryInformation[-time]


class DataLoader:
    def get_event(self, tick):  # 获得tick时刻的比赛状态
        return self.event_states[tick]

    def set_tick_state(self, tick, event_state):  # 设置此时的信息
        self.tick = tick
        self.event_states[tick] = event_state

    tick = 0
    event_states = [-1 for n in range(100000)]
