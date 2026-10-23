import math
from typing import override

import Global
import Utils
from Geometry import *
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
    KICK_OFF_RESIDUAL_VEL = 200 # 用来避免Double Touch的，控制的到达踢球点时与球的相对速度。
    DEFAULT_POS = CGeoPoint(3000, 3000)
    ALL_POSSIBLE_NUMBERS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    DEBUG_MSG_POS = CGeoPoint(0, -4000)
    DEFAULT_PASS_MAX_BALL_VELOCITY = 3500
    DPMBV = DEFAULT_PASS_MAX_BALL_VELOCITY
    GOAL_WIDTH = 1000
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

"""PARAMETERS ABOVE"""
"""UTIL FUNCTIONS BELOW"""
class utils:
    WeGotBall : bool = False
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
    def kick_ball_pos(ballpos: CGeoPoint, target: CGeoPoint, prep_length: float = 100):
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
        else:
            rx = Ball.posX() - Enemy.posX(role)
            ry = Ball.posY() - Enemy.posY(role)
        angle = Player.direction(role)
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
        raise ValueError("The other attacker not found.")
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
            return ""
    class Defend(State):
        def getTasks(self):
            # TODO : WMarking 如何做到让两个车都去防守？现在只有一个车在防守，Goalie和B都呆在那里。
            return {
                "A": Task(WMarking(priority=0)),
                "B": Task(WMarking(priority=0)),
                "G": Task(Goalie(), fixedNumber=p.GOALIE_NUMBER)
            }
        def getMatchString(self) -> str:
            return "[AB](G)"
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            if u.WeGotBall : # 防守脚本一旦抢到球获得球权就进入进攻脚本。
                # TODO : 需要特殊处理双方抢球的情况
                return "StartAttack"
            return ""
    class AttackFromGoalie(State):
        def getTasks(self):
            return {
                "G": Task(Goalie(), fixedNumber=p.GOALIE_NUMBER)
            }
        def getMatchString(self) -> str:
            return ""
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            if not u.WeGotBall :# 进攻脚本一旦发现失去球权立即进入防守
                return "Defend"
            return ""
    class AttackNormal(State):
        def reset(self):
            self.AttackingStatus = "select"
            self.NextPoint = None
            self.Bgotball = False
            self.Bpassed = False
            self.Anum = -1
            self.Bnum = -1
            if u.OurLastHolder != -1 and u.JustKickedOff:
                self.Anum = u.OurLastHolder
                self.Bnum = u.the_other_attacker(u.OurLastHolder)
                


        def run(self, isStateSwitched):
            if isStateSwitched:
                self.reset()
            return super().run(isStateSwitched)

        def getTasks(self):
            # 对下一个进攻点进行采样——我的方法。
            # 记号约定：A负责接球，B负责传球或者直接打门。每次A接到球，刷新成select状态。

            # 选择点
            debugEngine.gui_debug_msg(Ball.pos()+CVector(0,-200,), msgstr=f"OurLastholder={u.OurLastHolder}")
            if self.AttackingStatus == "select":
                self.reset() # 清除固定车号，让官方munkres重新匹配。
                
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
                if self.NextPoint is None: # 否则回传
                    self.AttackingStatus = "goback"

            #根据采样结果规划下一步的任务
            if self.AttackingStatus == "goback" : # 如果采样失败，准备回传并重新传出。
                Apoint = u.sample_in_rectangle(*p.AREA_J[0]) # 进攻传球队员的准备点位
                Atask = Task(
                    SmartGoTo(
                        target=Apoint,
                        dir=(Player.pos(p.GOALIE_NUMBER) - Apoint).dir(),
                        flag=Flags.quickly
                    ),
                    fixedNumber=self.Anum
                )
                Btask = Task(
                    PassToPos(
                        Player.pos(p.GOALIE_NUMBER),
                        kickpower=7000
                    ),
                    fixedNumber=self.Bnum
                )
                Gtask = Task(
                    StaticGetBallV4(
                        pos=Apoint,
                        power=11000,
                        isChip=True
                    ), fixedNumber=p.GOALIE_NUMBER
                )
            elif self.AttackingStatus == "attack":
                if self.NextPoint is None:
                    raise ValueError("Invalid Nexpoint.")
                # 计算A的任务
                if self.Bpassed == False:
                # if False:
                    Atask = Task(
                        SmartGoTo(
                            target=self.NextPoint, # 去往选择好的进攻点
                            dir=(u.centre_of_rectangle(p.AREA_K[0]) - self.NextPoint).dir(), #朝向射门区K
                            flag = Flags.quickly
                        ),
                        fixedNumber=self.Anum
                    )
                else: 
                    if self.Anum != -1:
                        debugEngine.gui_debug_msg(
                            p=Player.pos(self.Anum),
                            msgstr="going to ball!",
                        )
                    Atask = Task(
                        # RushTo( # 动态拿球，下一步是射门或者传到下一个点
                        #     pos = Ball.pos(),
                        #     needDribble = True,
                        #     angle = (Ball.pos() - Player.pos(self.Anum)).dir(), #默认朝向射门区
                        # ),
                        GetBall(),
                        fixedNumber=self.Anum
                    )
                # 计算B的任务
                if not self.Bpassed:
                    if self.Anum == -1:
                        Btask = Task(
                            StaticGetBallV4(
                                pos = self.NextPoint,
                                power=6000
                            ),
                            fixedNumber=self.Bnum
                        )
                    else:
                        Btask = Task(
                            StaticGetBallV4(
                                pos = Player.pos(self.Anum),
                                power=6000
                            ), fixedNumber=self.Bnum
                        )
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
                Gtask = Task(
                    Goalie(),
                    fixedNumber=p.GOALIE_NUMBER
                )
            else: # AttackingStatus = "shoot"
                Atask = None
                Btask = Task(
                    NormalShoot(power = 11000, isChip=False, flag=Flags.do_not_stop),
                    fixedNumber=self.Bnum
                )
                Gtask = Task(Goalie(), fixedNumber=p.GOALIE_NUMBER)


            
            if Atask is None:
                Apoint = ReceivePos(shoot_pos=Enemy.pos(p.ENEMY_GOALIE))
                Atask = Task(
                    SmartGoTo(
                        target = Apoint,
                        dir = (Enemy.pos(p.ENEMY_GOALIE) - Apoint).dir()
                    ), fixedNumber=self.Anum
                )
            debugEngine.gui_debug_msg(
                p=Ball.pos(),
                msgstr=f"AttackingStatus={self.AttackingStatus}\nBgotball={self.Bgotball}\nBpassed={self.Bpassed}",
                size = 150
            )
            return {
                "A": Atask,
                "B": Btask,
                "G": Gtask
            }
            
        def getMatchString(self) -> str:
            return "[BA](G)"
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            u.JustKickedOff = False
            if not u.WeGotBall :# 进攻脚本一旦发现失去球权立即进入防守
                return "Defend"
            if u.OurLastHolder == p.GOALIE_NUMBER:
                return "AttackFromGoalie"

            if not self.Bgotball and u.got_ball(self.Bnum):
                self.Bgotball =  True
            if self.Bgotball and not u.arrive_at(Ball.pos(), self.Bnum, dist=250):
                self.Bpassed = True
            
            self.Anum = self.playerNum("A") # 截取官方的任务分配结果，保持不变。
            self.Bnum = self.playerNum("B")

            if u.got_ball(self.Anum):# A方接到球就进行下一次选择，各种参数重置由getTasks进行。
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