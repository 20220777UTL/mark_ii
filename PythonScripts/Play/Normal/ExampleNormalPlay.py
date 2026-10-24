import math
from typing import override

import Global
import Utils
from Geometry import *
import time
from Global import debugEngine
from RoleMatch_LuaStyle import Task, State, declare_state_machine, StateMachine
from RoleMatch_LuaStyle.Skills import Skill, SimpleGoTo, Goalie, RushTo
from Strategy import *
from Utils import buffered_condition
from Vision import *
from Vision import Enemy
from WorldModel import Flags, Conditions, Params, Positions
from RoleMatch_LuaStyle.Skills.Skill import *
import random
from CppPackage import InTheirPenaltyArea



"""PARAMETERS BELOW"""
class parameters:
    ENEMY_GOALIE = 0
    GOALIE_NUMBER = 0
    ARRIVAL_DIST = 150
    ROBOT_MOUTH_RADIUS_MAX = 125 # mm 这个参数很重要！控制了KickOff脚本在多少的距离会从Rush转成PassToPos。
    ROBOT_MOUTH_RADIUS_MIN = 90 # mm 没什么用的参数
    ROBOT_MOUTH_HALF_WIDTH = 30 # mm，这个是整个的宽度，但是在边缘处的情况下射球效果可能不理想
    ROBOT_RADIUS = 75 #mm
    ROBOT_MAX_ACC = 1000 #mm/s^2
    LEAVE_THRESHOLD = 300 # 用来判断是否已经出球
    KICK_OFF_RESIDUAL_VEL = 500 # 用来避免Double Touch的，控制的到达踢球点时与球的相对速度。
    DEFAULT_POS = CGeoPoint(3000, 3000)
    ALL_POSSIBLE_NUMBERS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    DEBUG_MSG_POS = CGeoPoint(0, -4000)
    DEFAULT_PASS_MAX_BALL_VELOCITY = 3500
    DPMBV = DEFAULT_PASS_MAX_BALL_VELOCITY
    GOAL_WIDTH = 1000
    TOUCH_BALL_ACC_THRESHOLD = 1000 # 超过这个阈值，就代表球发生了剧烈的加速，意味着有人触球。
    MAX_ATTEMPTS = 8 # 每一个进攻阶段会在每个候选区域内进行这么多次采样尝试，如果采样失败就返回到更保守的候选区域。
    AREA_J = [
        (CGeoPoint(3000, -1200), CGeoPoint(4200, -2700)), # 右下角的传球区
        (CGeoPoint(3000, 1200), CGeoPoint(4200, 2700)) # 右上角的传球区
    ]
    AREA_K = [
        (CGeoPoint(2400, 1000), CGeoPoint(3400, -1000)) # 禁区前方的射门区
    ]
    AREA_G = [
        (CGeoPoint(4500, 500), CGeoPoint(4600, -500))
    ]
    POWER_RATIO = 2.5

"""PARAMETERS ABOVE"""
"""UTIL FUNCTIONS BELOW"""
class utils:
    ShouldReceiver = 0
    ShouldPasser = 1
    LastRefreshTime : float = 0
    LastBallVel : CVector = CVector(0, 0)
    BallAcc: CVector = CVector(0, 0)
    WeGotBall : bool = False
    LastBallToucher: tuple
    # We Got Ball 的定义：
    # 如果发生暂停，WeGotBall为False。
    # 如果当前在活球状态，那么已经开球，一定存在上一个持球者。上一个持球者如果不是我方球员，WeGotBall=False。
    # 更新逻辑：每个NormalPlay状态在transfunc中用refresh。每个RefPlay脚本在开球状态初始化的时候就修改。
    OurLastHolder : int = -1 
    # Our Last Holder 的定义：如果当前非我方控球，OurLastHolder = -1。
    # 如果当前为我方控球，一定存在上一次我方持球，记录上一次是谁持球。
    # 如果当前不是我方控球
    # Our Last Holder 的维护逻辑：
    # 当有我方球员接球，刷新OurLastHolder为上一个持球者。
    # 当丢球，OurLastHolder = -1
    JustKickedOff: bool = False # 用于从KickOff等脚本中继承上一次的发球车，防止任务匹配失败。
    # JustKickedOff 的定义：如果开球之后还没有其他球员触球，也没有发生暂停，那么JustKickedOff = True。否则为False.
    @staticmethod
    def dot2(a, b):
        return a[0] * b[0] + a[1] * b[1]
    @staticmethod
    def num_mult(num, vect):
        return (vect[0] * num, vect[1] * num)
    @staticmethod
    def unit(vect):
        len = math.sqrt(vect[0] ** 2 + vect[1] ** 2)
        if len == 0:
            return None
        return (vect[0] / len, vect[1] / len)
    @staticmethod
    def kick_ball_pos(ballpos: CGeoPoint, target: CGeoPoint, prep_length: float = parameters.ROBOT_MOUTH_RADIUS_MIN):
        """返回 (出球朝向, 准备位置)：沿球指向 target 的反方向后退 prep_length。

        prep_length 单位为 mm；准备位置是机器人中心的目标点。
        球与 target 重合时无法定义出球方向，沿用 point_angle 的异常处理。
        """
        angle = (target - ballpos).dir()
        ret_pos = CGeoPoint(
            ballpos.x() - math.cos(angle) * prep_length,
            ballpos.y() - math.sin(angle) * prep_length,
        )
        return angle, ret_pos

    @staticmethod
    def arrive_at(position: CGeoPoint, num: int, dist: float = parameters.ARRIVAL_DIST):
        playerpos = Player.pos(num)
        return (playerpos - position).mod() < dist
    @staticmethod
    def is_in_mouth(role: int, isEnemy: bool):
        if not isEnemy:
            rx = Ball.posX() - Player.posX(role)
            ry = Ball.posY() - Player.posY(role)
            angle = Player.direction(role)
        else:
            rx = Ball.posX() - Enemy.posX(role)
            ry = Ball.posY() - Enemy.posY(role)
            angle = Enemy.dir(role)
        
        dir = (math.cos(angle), math.sin(angle)) # 机器人方向向量
        norm = (-dir[1], dir[0]) # 机器人法向量
        para = u.dot2(dir, (rx, ry)) # 机器人朝向方向，球在哪个位置
        condit1 = (p.ROBOT_MOUTH_RADIUS_MIN <= para <= p.ROBOT_MOUTH_RADIUS_MAX)
        perpx = rx - para * dir[0] 
        perpy = ry - para * dir[1]
        perp = u.dot2(norm, (perpx, perpy)) # 机器人法向方向，球的位置
        condit2 = (-p.ROBOT_MOUTH_HALF_WIDTH <= perp <= p.ROBOT_MOUTH_HALF_WIDTH)
        return condit1 and condit2
    @staticmethod
    def got_ball(role: int, isEnemy: bool = False): # 用来检测有没有拿到球
        return u.is_in_mouth(role, isEnemy) # 调试用。实际测试用下面带有infra的
        if not isEnemy:
            return u.is_in_mouth(role, isEnemy) and Player.isInfraredOn(role) # 实车用
        else :
            return u.is_in_mouth(role, isEnemy) # 敌方的infrared无法访问
    @staticmethod
    def toVector(length: float, angle: float): # 用来把极坐标转换成直角坐标，返回CVector
        return CVector(length * math.cos(angle), length * math.sin(angle))
    @staticmethod
    def allPlayerStatus(): # 返回字典 {车号i:车号i是否正在控球} 并且刷新上次拿球者
        players = Player.getAllValidNumbers()
        statuses = {}
        for i in players:
            statuses[i] = u.got_ball(i, False)
            if statuses[i] == True:
                u.OurLastHolder = i
        
        return statuses
    @staticmethod
    def the_other_attacker(num: int):
        ps = u.allPlayerStatus()
        for i in ps:
            if i != p.GOALIE_NUMBER and i != num:
                return i
        return -1
    @staticmethod
    def allEnemyStatus(): # 返回字典 {敌方车号i:车号i是否正在控球}
        players = [i for i in p.ALL_POSSIBLE_NUMBERS if Enemy.valid(i)] # 能否不实时更改？
        statuses = {}
        for i in players:
            statuses[i] = u.got_ball(i, True)
        return statuses
    @classmethod
    def refreshHaveBallStatus(cls): # 必须每个State的transFunction都反复调用，否则就没用。实时刷新才能用！
        # 当前实现中会在NormalPlay的每个时刻都调取。
        
        ps = cls.allPlayerStatus()
        for i in ps:
            if ps[i] == True:
                cls.WeGotBall = True
                cls.OurLastHolder = i
        es = cls.allEnemyStatus() # 如果双方都有控球，那么说明在争夺球权，此时不认为WeGotBall
        for i in es:
            if es[i] == True:
                cls.WeGotBall = False
                cls.OurLastHolder = -1
            if InTheirPenaltyArea(Enemy.pos(i), 0):
                p.ENEMY_GOALIE = i

        if u.LastRefreshTime != 0: # 不是第一次
            u.BallAcc = (Ball.vel() - u.LastBallVel)/(time.monotonic() - u.LastRefreshTime)
            u.LastBallVel = Ball.vel()
            u.LastRefreshTime = time.monotonic()
            if u.BallAcc.mod() > p.TOUCH_BALL_ACC_THRESHOLD:
                o = []
                
                for i in ps:
                    o.append((i, (Ball.pos() - Player.pos(i)).mod(), False))
                for j in es:
                    o.append((j, (Ball.pos() - Enemy.pos(j)).mod(), True))
                mini = (0, float("inf"), False)
                for t in o:
                    if t[1] < mini[1]:
                        mini = t
                u.LastBallToucher = mini
                
                
        else:
            u.LastBallVel = Ball.vel()
            u.LastRefreshTime = time.monotonic()

        
                

        

        
    @staticmethod
    def whetherIntersect(start_segment_1: CGeoPoint, end_segment_1: CGeoPoint, start_segment_2: CGeoPoint, end_segement_2: CGeoPoint)->bool:
        # Detects parallel, mainly for avoiding dividing by zero.
        P, Q, R, S = start_segment_1, end_segment_1, start_segment_2, end_segement_2
        xP, yP = P.x(), P.y()
        xQ, yQ = Q.x(), Q.y()
        xR, yR = R.x(), R.y()
        xS, yS = S.x(), S.y()
        term1 = (xQ - xP) * (yR - yS)
        term2 = (xR - xS) * (yQ - yP)
        if math.isclose(term1, term2, rel_tol=1e-7, abs_tol=1e-4) :
            return False
        #计算参数方程(xR + t(xS - xR), yR + t(yS - yR))在直线PQ上的解。判断t是否在(0,1)之间，如果是则相交。
        t = (
            (xR - xP) * (yQ - yP) - (xQ - xP) * (yR - yP)
        ) / (
            (xS - xR) * (yQ - yP) - (yS - yR) * (xQ - xP)
        )
        if 0 < t < 1:
            return True
        else:
            return False

    @staticmethod
    def projectOntoLine(point: CGeoPoint, start_line: CGeoPoint, end_line: CGeoPoint):
        P, Q, A = start_line, end_line, point
        if (P - Q).mod() < 1:
            return None # 两个端点太近
        if (Q - A).mod() < 1: # 离端点太近
            return Q
        if (P - A).mod() < 1:
            return P
        if -1e-2 < (Q - A).dir() - (P - A).dir() < 1e-2:
            return A # 近似共线
        t = - (Q-P)*(P-A) / (P-Q).mod2()
        return P + (Q - P) * t

    @staticmethod
    def whetherBlockedByMovingEnemy(start:CGeoPoint, end:CGeoPoint, enemyPos: CGeoPoint, enemyVel: CVector, timeplus: float = 0): # timeplus 表示多久之后传出来
        # 先将敌人位置投影到start->end的直线上，计算留给敌人的反应时间。
        toEnemy = enemyPos - start
        toEnd = end - start
        if not (- math.pi / 3) < (toEnd.dir() - toEnemy.dir()) < (math.pi / 3):
            return False # 夹角大于60度，不会阻挡
        if toEnemy.mod() < 150:
            return True # 夹角小于等于60度，并且距离太小，阻挡
        proj = u.projectOntoLine(enemyPos, start, end)
        if proj is None:
            raise ValueError
        reactionTime = (proj - start).mod() / p.DPMBV + timeplus # seconds
        ddx = p.ROBOT_MAX_ACC * (reactionTime ** 2) / 2
        dx = reactionTime * enemyVel.mod()
        if (-0.5 < enemyVel.dir() - toEnd.dir() < 0.5): # 
            return (enemyPos-proj).mod() < ddx
        P1 = enemyPos + u.toVector(dx + ddx, enemyVel.dir())
        P2 = enemyPos + u.toVector(dx - ddx, enemyVel.dir())
        return u.whetherIntersect(
            P1, P2,
            start, end
        )
    @staticmethod
    def whetherBlockedByEnemy(start: CGeoPoint, end:CGeoPoint, enemyPos: CGeoPoint):
        toEnemy = enemyPos - start
        toEnd = end - start
        return -math.pi/18 < (toEnd.dir() - toEnemy.dir()) < math.pi/18

    @staticmethod
    def sample_in_rectangle(
        left_top: CGeoPoint,
        right_down: CGeoPoint,
    ) -> CGeoPoint:
        xmin, xmax = sorted((left_top.x(), right_down.x()))
        ymin, ymax = sorted((left_top.y(), right_down.y()))

        if xmin == xmax or ymin == ymax:
            raise ValueError("矩形不能退化成线或点")

        point = CGeoPoint(
            random.uniform(xmin, xmax),
            random.uniform(ymin, ymax),
        )
        return point

    @staticmethod
    def sample_in_rotated_rectangle(
        corner: CGeoPoint,
        neighbor_a: CGeoPoint,
        neighbor_b: CGeoPoint,
    ) -> CGeoPoint:
        """在三个顶点定义的斜矩形内均匀采样。平行四边形也可以."""
        ax = neighbor_a.x() - corner.x()
        ay = neighbor_a.y() - corner.y()
        bx = neighbor_b.x() - corner.x()
        by = neighbor_b.y() - corner.y()

        length_a = math.hypot(ax, ay)
        length_b = math.hypot(bx, by)
        if length_a == 0 or length_b == 0:
            raise ValueError("矩形的边长必须大于 0")

        u = random.random()
        v = random.random()
        return CGeoPoint(
            corner.x() + u * ax + v * bx,
            corner.y() + u * ay + v * by,
        )
    @staticmethod
    def area(rectangle):
        if rectangle.__len__() == 2:
            P = rectangle[0]
            Q = rectangle[1]
            return abs((Q.x() - P.x()) * ((Q.y() - P.y())))
        elif rectangle.__len__() == 3:
            C = rectangle[0]
            A = rectangle[1]
            B = rectangle[2]
            CA = A - C
            CB = B - C
            return abs(CA.mod() * CB.mod())
        else :
            raise ValueError("area failure.")
    @staticmethod
    def In(x, Range):
        return (Range[0] <= x <= Range[1]) or (Range[1] <= x <= Range[0])
    @staticmethod
    def sample_in_rectangles(set):
        D = []
        sumOfArea = 0
        for rect in set:
            area = u.area(rect)
            beforeArea = sumOfArea + 0
            sumOfArea += area
            D.append((rect, (beforeArea, sumOfArea))) # 用来采样
        sample = random.uniform(0, sumOfArea)
        sampled_rectangle = None
        for tup in D:
            if u.In(sample, tup[1]):
                sampled_rectangle = tup[0]
                break
        if sampled_rectangle is None:
            sampled_rectangle = p.AREA_K[0]
        if sampled_rectangle.__len__() == 2:
            return u.sample_in_rectangle(*sampled_rectangle)
        elif sampled_rectangle.__len__() == 3:
            return u.sample_in_rotated_rectangle(*sampled_rectangle)
        else :
            raise ValueError ("Invalid rectangle format")
    @staticmethod 
    def available_path(startpos: CGeoPoint, endpos: CGeoPoint, timeplus: float = 0) -> bool:
        enemies = [i for i in p.ALL_POSSIBLE_NUMBERS if Enemy.valid(i)] # 能否不实时更改？
        for enemy in enemies :
            if u.whetherBlockedByMovingEnemy(startpos, endpos, Enemy.pos(enemy), Enemy.vel(enemy), timeplus):
                return False
        return True
    @staticmethod
    def available_path_simple(startpos: CGeoPoint, endpos: CGeoPoint) -> bool:
        enemies = [i for i in p.ALL_POSSIBLE_NUMBERS if Enemy.valid(i)] # 能否不实时更改？
        for enemy in enemies :
            if u.whetherBlockedByEnemy(startpos, endpos, Enemy.pos(enemy)):
                return False
        return True
    @staticmethod
    def centre_of_rectangle(rect: tuple[CGeoPoint, CGeoPoint] | tuple[CGeoPoint, CGeoPoint, CGeoPoint]):
        if rect.__len__() == 2:
            return CGeoPoint(
                (rect[0].x() + rect[1].x()) / 2,
                (rect[0].y() + rect[1].y()) / 2
            )
        elif rect.__len__() == 3:
            return CGeoPoint(
                (rect[1].x() + rect[2].x()) / 2, # type: ignore
                (rect[1].y() + rect[2].y()) / 2 # type: ignore
            )
        else :
            raise ValueError

        

"""UTIL FUNCTIONS ABOVE"""
"""SELF-DEFINED SKILLS BELOW"""
class skills:
    pass
"""SELF-DEFINED SKILLS ABOVE"""
p = parameters
u = utils
s = skills

from Play.RefPlay.ExampleFreeKick import ReceivePos

class NormalStates :
    def __init__(self):
        pass
    class StartAttack(State):
        def getTasks(self):
            return {}
        def getMatchString(self) -> str:
            return ""
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            if not u.WeGotBall : # 进攻脚本一旦发现失去球权立即进入防守
                return "Defend"
            if u.OurLastHolder == p.GOALIE_NUMBER:
                return "AttackFromGoalie"
            else :
                return "AttackNormal"
            
    class Defend(State):
        INTERCEPT_OFFSET = 100       # mm，球背向持球对手一侧
        MARK_OFFSET = 200            # mm，另一对手朝球方向
        TEAMMATE_CLEARANCE = 2 * p.ROBOT_RADIUS + 60
        APPROACH_TOLERANCE = 60      # mm，到拦截点后开始断球
        CLEAR_POWER = 6000

        def run(self, isStateSwitched):
            if isStateSwitched or not hasattr(self, "enemy_for_role"):
                self.enemy_for_role = {"A": None, "B": None}
                self.defender_for_role = {"A": -1, "B": -1}
            return super().run(isStateSwitched)

        @staticmethod
        def _point_towards(start, target, distance):
            dx, dy = target.x() - start.x(), target.y() - start.y()
            length = math.hypot(dx, dy)
            if length < 1:
                dx, dy = Positions.ourGoal.x() - start.x(), Positions.ourGoal.y() - start.y()
                length = math.hypot(dx, dy)
                if length < 1:
                    dx, dy, length = 1, 0, 1
            return CGeoPoint(start.x() + distance * dx / length,
                             start.y() + distance * dy / length)

        @staticmethod
        def _legal_target(point):
            from CppPackage import MakeInField, InOurPenaltyArea, MakeOutOfOurPenaltyArea
            margin = p.ROBOT_RADIUS + 30
            point = MakeInField(point, margin)
            if InOurPenaltyArea(point, margin):
                point = MakeOutOfOurPenaltyArea(point, margin + 1)
                point = MakeInField(point, margin)
            return point

        @classmethod
        def _separate_target(cls, target, occupied):
            """目标不能与队友的当前位置或另一目标重叠；无安全点时停止。"""
            target = cls._legal_target(target)
            occupied = [point for point in occupied if point is not None]
            if all(target.dist(point) >= cls.TEAMMATE_CLEARANCE for point in occupied):
                return target
            for point in occupied:
                dx, dy = target.x() - point.x(), target.y() - point.y()
                length = math.hypot(dx, dy)
                if length < 1:
                    dx, dy, length = 0, 1, 1
                ux, uy = dx / length, dy / length
                for vx, vy in ((ux, uy), (-uy, ux), (uy, -ux)):
                    candidate = cls._legal_target(CGeoPoint(
                        point.x() + vx * cls.TEAMMATE_CLEARANCE,
                        point.y() + vy * cls.TEAMMATE_CLEARANCE))
                    if all(candidate.dist(other) >= cls.TEAMMATE_CLEARANCE - 5
                           for other in occupied):
                        return candidate
            return None

        def _attackers(self):
            # 3V3：排除留在其本方禁区的门将。门将出击时按离我方球门
            # 的距离选最有威胁的两人，不把对方车号 0 固定当门将。
            candidates = [number for number in p.ALL_POSSIBLE_NUMBERS
                          if Enemy.valid(number)
                          and not InTheirPenaltyArea(Enemy.pos(number), 0)]
            incumbents = [self.enemy_for_role[role] for role in ("A", "B")
                          if self.enemy_for_role[role] in candidates]
            candidates.sort(key=lambda number:
                            (Enemy.pos(number).dist(Positions.ourGoal), number))
            return (incumbents + [n for n in candidates if n not in incumbents])[:2]

        def _remember_attackers(self, attackers):
            # 同一 Defend 生命周期内，存活的对手始终由同一个角色盯防。
            for role in ("A", "B"):
                if self.enemy_for_role[role] not in attackers:
                    self.enemy_for_role[role] = None
            for enemy in attackers:
                if enemy not in self.enemy_for_role.values():
                    role = next((name for name in ("A", "B")
                                 if self.enemy_for_role[name] is None), None)
                    if role is not None:
                        self.enemy_for_role[role] = enemy

        @staticmethod
        def _get_ball_task(ball):
            return Task(GetBallV5(
                direction=(Positions.theirGoal - ball).dir(),
                flag=Flags.allow_dss | Flags.dodge_our_defense_box))

        @staticmethod
        def _move_task(target, ball):
            if target is None:
                return Task(Stop())
            return Task(SmartGoTo(
                target=target, dir=(ball - target).dir(),
                flag=Flags.quickly | Flags.allow_dss | Flags.dodge_our_defense_box))

        def _defender_position(self, role):
            number = self.defender_for_role.get(role, -1)
            return Player.pos(number) if number >= 0 and Player.valid(number) else None

        def _intercept(self, role, enemy, ball, other_role):
            enemy_pos = Enemy.pos(enemy)
            # 从对手经球继续向外延伸 100 mm；重合时退向我方球门。
            outward = (ball + (ball - enemy_pos)) if ball.dist(enemy_pos) >= 1 else Positions.ourGoal
            target = self._point_towards(ball, outward, self.INTERCEPT_OFFSET)
            target = self._separate_target(target, [self._defender_position(other_role)])
            if target is None:
                return Task(Stop()), None
            defender_pos = self._defender_position(role)
            if (defender_pos is not None
                    and defender_pos.dist(target) <= self.APPROACH_TOLERANCE):
                return Task(StaticGetBallV4(
                    pos=Positions.theirGoal, power=self.CLEAR_POWER, isChip=True,
                    flag=Flags.allow_dss | Flags.dodge_our_defense_box)), target
            return self._move_task(target, ball), target

        def getTasks(self):
            tasks = {"G": Task(Goalie(), fixedNumber=p.GOALIE_NUMBER)}
            if not Ball.valid():
                tasks.update({"A": Task(Stop()), "B": Task(Stop())})
                return tasks

            ball = Ball.pos()
            attackers = self._attackers()
            self._remember_attackers(attackers)
            holders = [enemy for enemy in attackers if u.got_ball(enemy, True)]
            holder = min(holders, key=lambda enemy: (Enemy.pos(enemy).dist(ball), enemy),
                         default=None)

            if not attackers:
                tasks["A"] = self._get_ball_task(ball)
                block = self._point_towards(ball, Positions.ourGoal, 550)
                tasks["B"] = self._move_task(self._legal_target(block), ball)
            elif len(attackers) == 1:
                enemy = attackers[0]
                chaser = next(role for role in ("A", "B")
                              if self.enemy_for_role[role] == enemy)
                blocker = "B" if chaser == "A" else "A"
                tasks[chaser] = self._get_ball_task(ball)
                intercept_target = ball
                block = self._point_towards(ball, Positions.ourGoal, 550)
                block = self._separate_target(
                    block, [intercept_target, self._defender_position(chaser)])
                tasks[blocker] = self._move_task(block, ball)
            elif holder is None:
                tasks["A"] = self._get_ball_task(ball)
                tasks["B"] = self._get_ball_task(ball)
            else:
                chaser = next(role for role in ("A", "B")
                              if self.enemy_for_role[role] == holder)
                blocker = "B" if chaser == "A" else "A"
                tasks[chaser], intercept_target = self._intercept(
                    chaser, holder, ball, blocker)
                other_enemy = self.enemy_for_role[blocker]
                guard = self._point_towards(
                    Enemy.pos(other_enemy), ball, self.MARK_OFFSET)
                guard = self._separate_target(
                    guard, [intercept_target, self._defender_position(chaser)])
                tasks[blocker] = self._move_task(guard, ball)
            return tasks

        def getMatchString(self) -> str:
            # 首帧匹配一次；后续保持 A/B 两辆实际车号与盯防目标不变。
            return "(AB)(G)"

        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            self.defender_for_role = {role: self.playerNum(role)
                                      for role in ("A", "B")}
            if u.WeGotBall and any(
                    u.got_ball(number) for number in Player.getAllValidNumbers()):
                return "StartAttack"
            return ""
    class AttackFromGoalie(State):
        def getTasks(self):
            receive_pos = u.sample_in_rectangles(p.AREA_J)
            attack_pos = u.sample_in_rectangles(p.AREA_K)

            
            return {
                "A": Task(SmartGoTo(target=receive_pos, dir = (Ball.pos() - receive_pos).dir(), flag=Flags.quickly)),
                "B": Task(SmartGoTo(target=attack_pos, dir=(Ball.pos() - attack_pos).dir(), flag = Flags.quickly)),
                "G": Task(StaticGetBallV4(pos = receive_pos, isChip=True), fixedNumber=p.GOALIE_NUMBER)
            }
        def getMatchString(self) -> str:
            return "(ABG)"
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            u.ShouldPasser = self.playerNum("A")
            u.ShouldReceiver = self.playerNum("B")
            if not u.WeGotBall :# 进攻脚本一旦发现失去球权立即进入防守
                return "Defend"
            if not u.arrive_at(Ball.pos(), p.GOALIE_NUMBER, dist = 300):
                return "AttackNormal"
            return ""
    class AttackNormal(State):
        def reset(self):
            self.AttackingStatus = "select"
            self.NextPoint = None
            self.Bstatus = "getball" # getball shoot pass
            self.Anum = -1
            self.Bnum = -1
            
                


        def run(self, isStateSwitched):
            if isStateSwitched:
                self.reset()
            return super().run(isStateSwitched)

        def getTasks(self):
            # 对下一个进攻点进行采样——我的方法。
            # 记号约定：A负责接球，B负责传球或者直接打门。每次A接到球，刷新成select状态。
            if u.JustKickedOff:
                self.Anum = u.OurLastHolder
                self.Bnum = u.the_other_attacker(u.OurLastHolder)
            # 选择点
            debugEngine.gui_debug_msg(Ball.pos()+CVector(0,-200), msgstr=f"OurLastholder={u.OurLastHolder} JustKickedOff={u.JustKickedOff}")
            if self.AttackingStatus == "select": # select状态表示接下来判断下一个进攻路线
                 # 清除固定车号，让官方munkres重新匹配。
                self.NextPoint = None
                self.Bstatus = "getball" # getball dribble shoot
                sample = None
                for _ in range(p.MAX_ATTEMPTS):
                    sample = u.sample_in_rectangles(p.AREA_G)
                    if u.available_path_simple(Ball.pos(), sample):
                        self.NextPoint = sample
                        self.AttackingStatus = "shoot"
                        break
                if self.AttackingStatus == "select": # 如果刚才没有选中说明不适合射门。看看能不能attack。
                    for _ in range(p.MAX_ATTEMPTS):
                        sample = u.sample_in_rectangles(p.AREA_K)
                        if u.available_path_simple(Ball.pos(), sample):
                            self.NextPoint = sample # 如果采样点中出现合适的进攻点位，就进行攻击。
                            self.AttackingStatus = "attack"
                            break
                

            #根据采样结果规划下一步的任务
            if self.AttackingStatus == "attack":
                if self.NextPoint is None:
                    raise ValueError("Invalid Nexpoint.")
                # 计算A的任务
                
                if self.Bstatus != "end":
                    Atask = Task(
                        SmartGoTo(
                            # target=self.NextPoint, # 去往选择好的进攻点
                            target=self.NextPoint, # 实时跑位
                            dir=(u.centre_of_rectangle(p.AREA_K[0]) - self.NextPoint).dir(), #朝向射门区K
                            flag = Flags.quickly
                        ),
                        fixedNumber=self.Anum
                    )
                    debugEngine.gui_debug_msg(p=Player.pos(self.Anum)+CVector(0,-150), msgstr="A:SmartGoto(NextPoint)")
                else: 
                    if self.Anum != -1:
                        debugEngine.gui_debug_msg(
                            p=Player.pos(self.Anum),
                            msgstr="going to ball!",
                        )
                    
                    Atask = Task(
                        GetBallV5(
                            direction = (Player.pos(self.Bnum)-Player.pos(self.Anum)).dir()
                        ),
                        fixedNumber=self.Anum
                    )
                    debugEngine.gui_debug_msg(p=Player.pos(self.Anum)+CVector(0,-150), msgstr="A:GetBallV5()")
                # 计算B的任务
                if self.Bstatus == "getball":
                    if self.Anum == -1:
                        Btask = Task(
                            GetBallV5(
                                direction=(Player.pos(self.Anum) - Player.pos(self.Bnum)).dir()
                            ),
                            fixedNumber=self.Bnum
                        )
                        debugEngine.gui_debug_msg(p=Player.pos(self.Bnum)+CVector(0,-150), msgstr="B:StaticGetBallV4(NextPoint,6000)")
                    else:
                        Btask = Task(
                            GetBallV5(
                                direction=(Player.pos(self.Anum) - Player.pos(self.Bnum)).dir()
                            ),
                            fixedNumber=self.Bnum
                        )
                        debugEngine.gui_debug_msg(p=Player.pos(self.Bnum)+CVector(0,-150), msgstr="B:getball:StaticGetBallV4(Apos,6000)")
                elif self.Bstatus == "dribble":
                    Bangle, Bpos = u.kick_ball_pos(Ball.pos(), Player.pos(self.Anum), prep_length=p.ROBOT_MOUTH_RADIUS_MIN - 60)
                    Btask = Task(
                        GetBallV5(
                            direction=(Player.pos(self.Anum)-Player.pos(self.Bnum)).dir()
                        ),
                        # SimpleGoTo(
                        #     Bpos, Bangle
                        # ),
                        fixedNumber=self.Bnum
                    )
                    debugEngine.gui_debug_msg(p=Player.pos(self.Bnum)+CVector(0,-150), msgstr="B:dribble:RushTo")

                elif self.Bstatus == "shoot":
                    can_shoot = False

                    for _ in range(p.MAX_ATTEMPTS):
                        sample = u.sample_in_rectangles(p.AREA_G)
                        if (u.available_path_simple(Ball.pos(), sample)):
                            can_shoot = True
                            Btask = Task(
                                NormalShoot(
                                    power=11500, isChip=False
                                )
                            )
                            debugEngine.gui_debug_msg(p=Player.pos(self.Bnum)+CVector(0,-150), msgstr="B:NormalShoot(11500)")



                    if not can_shoot:
                        Btask = Task(
                            TurnAndShoot(
                                target = Player.pos(self.Anum), power = (Player.pos(self.Anum) - Player.pos(self.Bnum)).mod() * p.POWER_RATIO
                            ),
                            fixedNumber=self.Bnum
                        )
                        debugEngine.gui_debug_msg(p=Player.pos(self.Bnum)+CVector(0,-150), msgstr="B:TurnAndShoot(Apos,6000)")
                        
                else:
                    # B 接下来应该去哪里？
                    Bpoint = messiDecision.receiverPos()
                    Btask = Task(
                        SmartGoTo(
                            target = Bpoint,
                            dir = (Ball.pos() - Bpoint).dir(),
                            flag=Flags.quickly
                        ),
                        fixedNumber=self.Bnum
                    )
                    debugEngine.gui_debug_msg(p=Player.pos(self.Bnum)+CVector(0,-150), msgstr="B:SmartGoTo(messiDecision)")
                Gtask = Task(
                    Goalie(),
                    fixedNumber=p.GOALIE_NUMBER
                )
            else: # AttackingStatus = "shoot"
                Atask = None
                Btask = Task(
                    NormalShoot(power = 11500, isChip=False, flag=Flags.do_not_stop),
                    fixedNumber=self.Bnum
                )
                debugEngine.gui_debug_msg(p=Player.pos(self.Bnum)+CVector(0,-150), msgstr="B:NormalShoot(11000)")
                Gtask = Task(Goalie(), fixedNumber=p.GOALIE_NUMBER)


            
            if Atask is None:
                Apoint = ReceivePos(shoot_pos=Enemy.pos(p.ENEMY_GOALIE))
                Atask = Task(
                    SmartGoTo(
                        target = Apoint,
                        dir = (Enemy.pos(p.ENEMY_GOALIE) - Apoint).dir()
                    ), fixedNumber=self.Anum
                )
                debugEngine.gui_debug_msg(p=Player.pos(self.Anum)+CVector(0,-150), msgstr="A:SmartGoTo(Apoint)")
            debugEngine.gui_debug_msg(
                p=CGeoPoint(0,4800),
                msgstr=f"AttackingStatus={self.AttackingStatus}  Bstatus:{self.Bstatus}",
                size = 150
            )
            return {
                "A": Atask,
                "B": Btask,
                "G": Gtask
            }
            
        # def getTasks(self):
        #     Atask = Task(SimpleGoTo(CGeoPoint(-3000,1000)), fixedNumber=self.Anum)
        #     Btask = Task(SimpleGoTo(CGeoPoint(3000,1000)), fixedNumber=self.Bnum)
        #     return{
        #         "A":Atask,
        #         "B":Btask,
        #         "G":Task(Goalie())
        #     }

        def getMatchString(self) -> str:
            return "[BA](G)"
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            if not u.WeGotBall :# 进攻脚本一旦发现失去球权立即进入防守
                return "Defend"
            if u.OurLastHolder == p.GOALIE_NUMBER:
                return "AttackFromGoalie"

            # if not self.Bgotball and u.got_ball(self.Bnum):
            #     self.Bgotball =  True
            # if self.Bgotball and not u.arrive_at(Ball.pos(), self.Bnum, dist=250):
            #     self.Bpassed = True
            if self.Bstatus == "getball":
                if u.arrive_at(Ball.pos(), self.Bnum, dist=500):
                    self.Bstatus = "dribble"
            elif self.Bstatus == "dribble":
                if u.got_ball(self.Bnum):
                    self.Bstatus = "shoot"
            elif self.Bstatus == "shoot":
                if (Ball.pos() - Player.pos(self.Anum)).mod() < (Ball.pos() - Player.pos(self.Bnum)).mod():
                    self.Bstatus = "end" # 如果距离A更近才切换到end
            
            
            self.Anum = self.playerNum("A") # 截取官方的任务分配结果，保持不变。
            self.Bnum = self.playerNum("B")

            if u.arrive_at(Ball.pos(), self.Anum, dist=250):# A方接到球就进行下一次选择，各种参数重置由getTasks进行。
                u.JustKickedOff = False
                self.AttackingStatus = "select"

            
            return ""
    class AttackDispicable(State):
        def getTasks(self):
            return {
                "G": Task(Goalie(), fixedNumber=p.GOALIE_NUMBER)
            }
        def getMatchString(self) -> str:
            return "(G)"
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            if not u.WeGotBall :# 进攻脚本一旦发现失去球权立即进入防守
                return "Defend"
            return ""




@declare_state_machine(
    NormalStates.Defend, 
    NormalStates.StartAttack, 
    NormalStates.AttackDispicable, 
    NormalStates.AttackFromGoalie, 
    NormalStates.AttackNormal
)
class ExampleNormalPlay(StateMachine):
    pass