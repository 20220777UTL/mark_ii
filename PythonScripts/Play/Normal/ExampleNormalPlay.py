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



"""PARAMETERS BELOW"""
class parameters:
    GOALIE_NUMBER = 0
    ARRIVAL_DIST = 150
    ROBOT_MOUTH_RADIUS_MAX = 105 # mm 这个参数很重要！控制了KickOff脚本在多少的距离会从Rush转成PassToPos。
    ROBOT_MOUTH_RADIUS_MIN = 90 # mm 没什么用的参数
    ROBOT_MOUTH_HALF_WIDTH = 30 # mm，这个是整个的宽度，但是在边缘处的情况下射球效果可能不理想
    ROBOT_RADIUS = 75 #mm
    LEAVE_THRESHOLD = 300 # 用来判断是否已经出球
    KICK_OFF_RESIDUAL_VEL = 200 # 用来避免Double Touch的，控制的到达踢球点时与球的相对速度。
    DEFAULT_POS = CGeoPoint(3000, 3000)
    ALL_POSSIBLE_NUMBERS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    DEBUG_MSG_POS = CGeoPoint(0, -4000)
"""PARAMETERS ABOVE"""
"""UTIL FUNCTIONS BELOW"""
class utils:
    WeGotBall : bool = False
    OurLastHolder : int = -1
    JustKickedOff: bool = False # 用于从KickOff等脚本中继承上一次的发球车，防止任务匹配失败。
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
    def allEnemyStatus(): # 返回字典 {敌方车号i:车号i是否正在控球}
        players = [i for i in p.ALL_POSSIBLE_NUMBERS if Enemy.valid(i)]
        statuses = {}
        for i in players:
            statuses[i] = u.got_ball(i, True)
        return statuses
    @classmethod
    def refreshHaveBallStatus(cls): # 必须每个State的transFunction都反复调用，否则就没用。实时刷新才能用！
        ps = cls.allPlayerStatus()
        for i in ps:
            if ps[i] == True:
                cls.WeGotBall = True
        es = cls.allEnemyStatus() # 如果双方都有控球，那么说明在争夺球权，此时不认为WeGotBall
        for i in es:
            if es[i] == True:
                cls.WeGotBall = False
"""UTIL FUNCTIONS ABOVE"""
"""SELF-DEFINED SKILLS BELOW"""
class skills:
    pass
"""SELF-DEFINED SKILLS ABOVE"""
p = parameters
u = utils
s = skills

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
            return {
                "A": Task(WMarking(priority=0)),
                "B": Task(WMarking(priority=1)),
                "G": Task(Goalie())
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
            return {}
        def getMatchString(self) -> str:
            return ""
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            if not u.WeGotBall :# 进攻脚本一旦发现失去球权立即进入防守
                return "Defend"
            return ""
    class AttackNormal(State):
        def getTasks(self):
            Atask = Task(NormalShoot(power = 12000, isChip = False))
            return {
                "A": Atask
            }
        def getMatchString(self) -> str:
            return "(A)"
        def transFunction(self) -> str:
            u.refreshHaveBallStatus()
            if not u.WeGotBall :# 进攻脚本一旦发现失去球权立即进入防守
                return "Defend"
            return ""
    class AttackDispicable(State):
        def getTasks(self):
            return {}
        def getMatchString(self) -> str:
            return ""
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