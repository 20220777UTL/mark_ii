"""校赛入口：选手填写五个 Example 模板，三个官方裁判策略统一提供。

不使用点球；人工摆球期间沿用 GameHalt，所有车辆停止并关闭吸球和踢球。
"""

import Global
from Play.Normal.ExampleNormalPlay import ExampleNormalPlay
from Play.RefPlay.ExampleKickOff import ExampleKickOff
from Play.RefPlay.ExampleKickOffDefend import ExampleKickOffDefend
from Play.RefPlay.ExampleFreeKick import ExampleFreeKick
from Play.RefPlay.ExampleFreeKickDefend import ExampleFreeKickDefend
from Play.RefPlay.GameHalt import GameHalt
from Play.RefPlay.GameStop import GameStop
from Play.RefPlay.OurTimeout import OurTimeout
from Play.Test.ExampleTest import ExampleTest


Global.isTestMode = False
Global.needDetailedRuntimeDebugInfo = False
# 比赛默认自动识别车号；本地固定三车调试可改成 (0, 1, 2)。
# NormalPlay 从视觉选择两名场上队员，守门员仍由 goalieNumber 固定。
Global.controlledRobotNumbers = (0, 1, 2)
Global.goalieNumber = 0
if (Global.controlledRobotNumbers is not None
        and Global.goalieNumber not in Global.controlledRobotNumbers):
    raise ValueError("The goalie must be one of controlledRobotNumbers")
Global.defaultRoleNumberStructTable["G"] = Global.RoleNumberStruct(
    Global.RoleNumberStruct.MatchType.Fixed, Global.goalieNumber
)


class GameStrategy:
    def __init__(self, blueTeam, yellowTeam=None):
        self.blueTeam = blueTeam
        self.yellowTeam = yellowTeam if yellowTeam is not None else blueTeam

    def runStrategy(self):
        if Global.isYellow:
            self.yellowTeam.planTasks()
        else:
            self.blueTeam.planTasks()


# key 由 GameControl/Referee.py 和 SelectPlay.py 使用；替换值时请保留 key。
Global.gameStrategies = {
    "NormalPlay": GameStrategy(ExampleNormalPlay()),
    "KickOff": GameStrategy(ExampleKickOff()),
    "KickOffDef": GameStrategy(ExampleKickOffDefend()),
    "FreeKick": GameStrategy(ExampleFreeKick()),
    "FreeKickDef": GameStrategy(ExampleFreeKickDefend()),
    "ourDirectKick": GameStrategy(ExampleFreeKick()),
    "theirDirectKick": GameStrategy(ExampleFreeKickDefend()),
    "TimeOut": GameStrategy(OurTimeout()),
    "GameStop": GameStrategy(GameStop()),
    "GameHalt": GameStrategy(GameHalt()),
    "PenaltyKick": GameStrategy(GameHalt()),
    "theirPenaltyKick": GameStrategy(GameHalt()),
    "BallPlace": GameStrategy(GameHalt()),
    "TheirBallPlace": GameStrategy(GameHalt()),
    "IfHalfField": False,
    "USE_ZPASS": False,
}

testStrategy = GameStrategy(ExampleTest())
