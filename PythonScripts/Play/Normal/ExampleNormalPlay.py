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
    ROBOT_MOUTH_RADIUS_MAX = 105 # mm
    ROBOT_MOUTH_RADIUS_MIN = 90 # mm
    ROBOT_MOUTH_HALF_WIDTH = 30 # mm，这个是整个的宽度，但是在边缘处的情况下射球效果可能不理想
    ROBOT_RADIUS = 75 #mm
    LEAVE_THRESHOLD = 300 # 用来判断是否已经出球
"""PARAMETERS ABOVE"""
"""UTIL FUNCTIONS BELOW"""
class utils:
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
    def is_in_mouth(role: int):
        rx = Ball.posX() - Player.posX(role)
        ry = Ball.posY() - Player.posY(role)
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
    def got_ball(role: int):
        return u.is_in_mouth(role) # 调试用。实际测试用下面带有infra的
        return u.is_in_mouth(role) and Player.isInfraredOn(role)

"""UTIL FUNCTIONS ABOVE"""
"""SELF-DEFINED SKILLS BELOW"""
class skills:
    pass
"""SELF-DEFINED SKILLS ABOVE"""
p = parameters
u = utils
s = skills


class GO_FORWARD(State):

    @override
    def getMatchString(self) -> str:
        return "(AG)"

    @override
    def getTasks(self) -> "dict[str, Task]":
        return {
            # # "A": Task(SimpleGoTo(C.CGeoPoint(1000, -500), 0, 0)),
            # # "B": Task(SimpleGoTo(C.CGeoPoint(1000, 500), 0, 0)),
            # #"C": Task(SimpleGoTo(CGeoPoint(1000, 1000), 0, 0)),
            # "C": Task(Skill.RushTo(Ball.pos())),
            # "D": Task(lambda: SimpleGoTo(self.playerPos("E") + CVector(200,200), 0, 0)),
            # "E": Task(SimpleGoTo(CGeoPoint(1000, 1500), 0, 0)),
            # "G": Task(SimpleGoTo(CGeoPoint(-3518,-3012), 0, 0)),
            # "H": Task(SimpleGoTo(Player.Pos("E") + CVector(-200,-200), 0, 0)), 
            "A": Task(Skill.NormalShoot(12700, False)),
            "G": Task(Skill.Goalie()),
        }

    # 子类不覆写__init__方法也没问题，不会报错；如果覆写，一定要调用super().__init__()！！！
    def __init__(self):
        super().__init__()

    @override
    def transFunction(self):
        if buffered_condition(Global.ball().Pos().x() > 2000,10):
                    return "Attack"
        """状态转换函数：根据球的位置决定状态"""
        if Global.ball().Pos().x() > 0:
            return "GO_FORWARD"
        return "GO_BACKWARD"


class GO_BACKWARD(State):
    @override
    def getMatchString(self) -> str:
        return "(GAB)"

    @override
    def getTasks(self) -> "dict[str, Task]":
        return{
            "A": Task(SimpleGoTo(CGeoPoint(-1000, -500), 0, 0)),
            "B": Task(SimpleGoTo(CGeoPoint(-1000, 500), 0, 0)),
            # "C": Task(SimpleGoTo(CGeoPoint(-1000, 1000), 0, 0)),
            # "D": Task(lambda: SimpleGoTo(self.playerPos("E"), 0, 0)),
            # "E": Task(SimpleGoTo(CGeoPoint(-1000, 1500), 0, 0)),
            # "H": Task(RushTo(CGeoPoint(100,200), 3, 0)),
            # "I": Task(lambda: RushTo(self.playerPos("H"), 3, 0)),
            "G": Task(Skill.Goalie()), }


    @override
    def transFunction(self):
        """状态转换函数：根据球的位置决定状态"""
        if Global.ball().Pos().x() > 0:
            return "GO_FORWARD"
        return "GO_BACKWARD"




class Attack(State):

    def getMatchString(self) -> str:
        return "(GAB)"

    def getTasks(self) -> "dict[str, Task]":
        return {
            "A":Task(Skill.NormalShoot(12700, False)),
            "B":Task(SimpleGoTo(CGeoPoint(2500,1000))),
            "G":Task(Skill.Goalie()),
        }

    def transFunction(self) -> str:
        if buffered_condition(Global.ball().Pos().x() < 2000,10):
            return "GO_FORWARD"
        return "Attack"


# 第一个状态是最初状态（GO_FORWARD）
START_STATE = GO_FORWARD


@declare_state_machine(
    START_STATE,
    GO_BACKWARD,
    Attack
)
class ExampleNormalPlay(StateMachine):
    pass