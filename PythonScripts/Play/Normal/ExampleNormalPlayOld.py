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


# 第一个状态是最初状态（GO_FORWARD


@declare_state_machine()
class ExampleNormalPlay(StateMachine):
    pass