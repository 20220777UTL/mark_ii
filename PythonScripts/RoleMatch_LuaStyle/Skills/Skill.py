"""供 Task 使用的技能封装，每个技能返回 (skill_cpp, matchPos)。

skill_cpp(executor) 为实际车号下发任务；matchPos(executor) 提供角色匹配位置。
位置、距离使用毫米，朝向和角度容差使用弧度；flag 为任务标志，力度沿用底层标定值。
"""
# Goalie uses fdb37bdc (2025-10-18), with an optional clearing target.
# PassToPos uses ChaseKickV1 and refreshes its target, direction, and power each frame.
# WMarking uses the native marking skill and its target query for role matching.
import typing
# from numbers import Number
from typing import Any

import CppPackage
import Utils
import WorldModel
from Geometry import *
from Strategy import messiDecision
from Vision import Player
from Vision import ball, Ball
from Vision import vision
from WorldModel import Flags, Params
from WorldModel import Positions
from WorldModel import taskMediator, Directions, KickMode, Precision, kickStatus, \
    ChipPower
from WorldModel.PowerCalculator import evalPower, P, POWER_MAX as _CALC_MAX

"""
步骤：copy之前的定义，然后从CppPackage.pyi中复制函数签名（除了返回值签名），把第一个参数executor去掉，剩下的参数保持不变。

似乎可以使用from functools import partial，用partial进行部分绑定实现同样的效果。
调用过程类似于"A": Task((partial(CppPackage.makeItGoalie, pos=Positions.ourGoal, flag=0), matchPos), fixedNumber=0)

**WARNING: Python 的函数默认参数 只在函数定义时计算一次！！！不是cpp一样每次调用时重新计算。所以默认参数一定要在函数体内部重新计算！**
"""


def Goalie(pos: CGeoPoint = None, flag: typing.SupportsInt = 0):
    """执行 fdb37bdc（2025-10-18）版守门：站位、扑救和清球。

    pos 为清球目标，省略时由策略模块实时提供；flag 为任务标志。
    """
    def skill_cpp(executor: int) -> Any:
        """注册守门员角色，并向 executor 下发包含清球目标的守门任务。"""
        taskMediator.registerRole(executor, "goalie")
        target = messiDecision.goaliePassPos() if pos is None else pos
        return CppPackage.makeItGoalie(executor, target, flag)

    def matchPos(executor: int) -> CGeoPoint:
        """返回我方球门位置，供系统匹配守门员。"""
        return Positions.ourGoal

    return skill_cpp, matchPos


def SimpleGoTo(point: CppPackage.CGeoPoint, angle: float = 0, flag: int = 0):
    """使用基础移动技能前往 point，并转向 angle（弧度）；flag 为任务标志。"""

    def skill_cpp(executor: int) -> Any:
        """向 executor 下发目标点、目标朝向和任务标志。"""
        return CppPackage.makeItSimpleGoTo(executor, point, angle, flag)

    def matchPos(executor: int) -> CppPackage.CGeoPoint:
        """返回目标点 point，作为角色匹配位置。"""
        return point

    return skill_cpp, matchPos


def RushTo(pos: CGeoPoint, angle: typing.Union[typing.Callable, typing.SupportsFloat] = None, maxAcc: typing.Optional[typing.SupportsFloat]= None,
           flag: typing.SupportsInt = 0, *, needDribble: bool = False, vel: CVector = CVector(0, 0),
           registerRole: str = "", sender: typing.SupportsInt = 0):
    """快速移动到 pos，可设置朝向、加速度上限和到达时的速度。

    angle 可为弧度或接收 executor 的函数，省略时朝向对方球门；maxAcc=0 使用底层默认限制。
    needDribble 控制吸球，vel 为目标速度，sender 为出球者车号，flag 为任务标志。
    registerRole 当前未使用，执行时固定注册为 fetchBall 角色。
    """
    if not maxAcc:
        maxAcc = 0
    def skill_cpp(executor: int) -> Any:
        """解析朝向、按需开启吸球，并向 executor 下发快速移动任务。"""
        nonlocal angle
        if angle is None:
            angle = Directions.shoot(executor)
        elif callable(angle):
            angle = angle(executor)

        if needDribble:
            CppPackage.setDribbleCommand(executor, 3)
        CppPackage.TaskMediator.Instance().registerRole(executor, "fetchBall")
        return CppPackage.makeItRushTo(executor, pos, angle, flag, sender, maxAcc, needDribble, vel)

    def matchPos(executor: int) -> CGeoPoint:
        """返回移动目标 pos，作为角色匹配位置。"""
        return pos

    return skill_cpp, matchPos

def RushToV4(pos, mydir, flag = 0):

    """使用 SmartGotoV4 移动到 pos，mydir 为目标朝向，flag 为任务标志。

    当前实现中，mydir 为空或为 0 时使用朝向对方球门的方向。
    """
    def skill_cpp(executor: int) -> Any:
        """确定目标朝向，并向 executor 下发 SmartGotoV4 移动任务。"""
        if not mydir:
            idir = Player.toTheirGoalDir(executor)
        else:
            idir = mydir

        return CppPackage.makeItSmartGoToV4(executor, pos, idir, flag)

    def matchPos(executor: int) -> CGeoPoint:
        """返回移动目标 pos，作为角色匹配位置。"""
        return pos

    return skill_cpp, matchPos


def Stop():
    """让机器人执行底层停止技能，匹配位置保持在机器人当前位置。"""
    def skill_cpp(executor: int) -> Any:
        """向 executor 下发停止任务。"""
        return CppPackage.makeItStop(executor)

    def matchPos(executor: int) -> CGeoPoint:
        """返回 executor 的当前位置，减少停止任务的角色换车。"""
        return Player.Pos(executor)

    return skill_cpp, matchPos


def WMarking(priority: int, *, flag: int = 0, num: int = -1):
    """安排一辆我方车盯防对手。

    priority 从 0 开始，表示对手的防守排序；num 为对方实际车号，-1 表示自动选择。
    指定 num 时需保证对手视觉有效；flag 为任务标志，系统按盯防站位匹配我方车。
    """
    if not 0 <= priority < Params.maxPlayer:
        raise ValueError("WMarking 的 priority 必须在 0..maxPlayer-1 范围内")
    if not -1 <= num < Params.maxPlayer:
        raise ValueError("WMarking 的 num 必须为 -1 或有效车号")

    def skill_cpp(executor: int) -> Any:
        """注册盯防角色，并向 executor 下发按优先级或指定车号的盯防任务。"""
        if 0 <= executor < Params.maxPlayer:
            taskMediator.registerRole(executor, "marking")
        return CppPackage.makeItWMarking(executor, priority, flag, num)

    def matchPos(executor: int) -> CGeoPoint:
        """查询底层给出的盯防站位，作为角色匹配位置。"""
        return CppPackage.getWMarkingPos(executor, priority, flag, num)

    return skill_cpp, matchPos


def StaticGetBall(target: CGeoPoint, kickPower, chipPower):
    """调整到球后方拿球，并对准 target，满足底层条件后出球。

    kickPower、chipPower 分别为平踢和挑踢力度，踢球方式由底层策略选择。
    """
    def skill_cpp(executor: int) -> Any:
        """向 executor 下发目标方向及平踢、挑踢力度。"""
        return CppPackage.makeItStaticGetBall(executor, target, kickPower, chipPower)

    def matchPos(executor: int) -> CGeoPoint:
        """返回球背向 target 一侧、距球 90 mm 的准备位置。"""
        return Ball.pos() + Utils.Polar2Vector(90, (Ball.pos() - target).dir())

    return skill_cpp, matchPos


def JustKick(power, isChip: bool = False, needDribble: bool = True):
    """直接设置 power 力度并朝球移动，不等待朝向就绪。

    isChip 选择挑踢或平踢，needDribble 控制吸球。
    """
    def skill_cpp(executor: int) -> Any:
        """设置平踢或挑踢指令，并让 executor 朝球移动。"""
        if isChip:
            kickStatus.setChipKick(executor, power)
        else:
            kickStatus.setKick(executor, power)
        # return CppPackage.makeItStop(executor)
        return CppPackage.makeItRushTo(executor, ball().Pos(), Player.toBallDir(executor), needDribble=needDribble)

    def matchPos(executor: int) -> CGeoPoint:
        """返回当前球位，优先匹配靠近球的机器人。"""
        return Ball.pos()

    return skill_cpp, matchPos


def Shoot(target: CGeoPoint=None, direction: typing.SupportsFloat = None, *, isChip: bool=False, power: typing.SupportsFloat = None,
          flag=Flags.nothing, precision: typing.SupportsFloat = Precision.HIGH, needDribble: bool = False):
    """在朝向满足 precision、检测到红外持球或设置 force_kick 时下发踢球指令。

    target 默认对方球门，用于力度计算和角色匹配；direction 默认朝向对方球门。
    isChip 选择挑踢，省略 power 时自动计算力度，flag 为任务标志。
    当前实现始终开启吸球，needDribble 参数暂不控制开关。
    """
    if not target:
        target = Positions.theirGoal
    def skill_cpp(executor: int) -> Any:
        # 先原始处理一些参数，下发指令。对应原lua层的Play.lua中对后面几个参数的处理
        """检查踢球条件、开启吸球，并向 executor 下发踢球和移动指令。"""
        nonlocal direction, isChip, needDribble, power

        # handle direction
        if not direction:
            direction = Directions.shoot(executor)
        isDirOk = WorldModel.worldModel.KickDirArrived(vision.getCycle(), direction, precision, executor) or Player.isInfraredOn(executor)

        # handle dribble
        #needDribble = needDribble or (flag & Flags.dribble)
        needDribble = True
        if needDribble:
            CppPackage.setDribbleCommand(executor, 3)

        # handle kick/chip
        if isDirOk or (flag & Flags.force_kick):
            kickMode = KickMode.chip if isChip else KickMode.flat
            match kickMode:
                case KickMode.flat:
                    if not power:
                        power = evalPower(Ball.pos(), target, P.LOW)
                    kickStatus.setKick(executor, power)
                case KickMode.chip:
                    if not power:
                        power = evalPower(Ball.pos(), target, P.LOW, is_chip=True)
                    kickStatus.setChipKick(executor, power)
                case _:
                    raise ValueError(f"Unsupported kick mode: {kickMode}")
        gotoPoint = Ball.pos()
        if (Player.isInfraredOn(executor)):
            gotoPoint = Player.pos(executor)
        return CppPackage.makeItRushTo(executor, gotoPoint, direction, maxAcc=0, flag=flag)


    def matchPos(executor: int) -> CGeoPoint:
        """返回出球目标 target，作为当前实现的角色匹配位置。"""
        return target

    return skill_cpp, matchPos

def GoAndTurnKick(target:CGeoPoint = Positions.theirGoal, registerRole="leader", *, power, flag=0, useInter: bool = False):
    """调用底层转身踢球技能，拿球并调整朝向后向 target 出球。

    power 为必填力度，registerRole 默认注册为 leader，flag 为任务标志。
    useInter 原样传给底层的控制方式字段；匹配点使用底层估算的拿球位置。
    """

    def skill_cpp(executor: int) -> Any:
        """注册指定角色，并向 executor 下发目标、力度和控制参数。"""
        taskMediator.registerRole(executor, registerRole)
        return CppPackage.makeItGoAndTurnKick(executor, target, useInter, power, flag)

    def matchPos(executor: int) -> CGeoPoint:
        """返回 executor 的最佳拿球位置，供角色匹配使用。"""
        return CppPackage.WeBestGetBallPosition(executor)

    return skill_cpp, matchPos

def TurnAndShoot(target: CGeoPoint = Positions.theirGoal, registerRole="leader", *, power: typing.SupportsFloat, flag: typing.SupportsInt = 0):
    """拿球后转向 target，持球且朝向对准时以 power 力度平踢。

    power 为必填参数；registerRole 默认注册为 leader，flag 为任务标志。
    """
    def skill_cpp(executor: int) -> Any:
        """注册指定角色，并向 executor 下发转向目标后平踢的任务。"""
        taskMediator.registerRole(executor, registerRole)
        return CppPackage.makeItTurnAndShoot(executor, target, power, flag)

    def matchPos(executor: int) -> CGeoPoint:
        """返回 executor 的最佳拿球位置，供角色匹配使用。"""
        return CppPackage.WeBestGetBallPosition(executor)

    return skill_cpp, matchPos

def SmartGoTo(target: CGeoPoint, dir: typing.SupportsFloat, flag: typing.SupportsInt=0, *, sender: typing.SupportsInt=0, maxAcc: typing.SupportsInt=0, Velocity: CVector = CVector(0,0)) -> Any:
    """使用智能路径规划移动到 target，并转向 dir（弧度）。

    flag 控制运动标志，sender 为出球者车号，maxAcc 为加速度上限，Velocity 为目标速度。
    """
    def skill_cpp(executor: int) -> Any:
        """向 executor 下发智能移动的目标点、朝向、速度和限制参数。"""
        return CppPackage.makeItSmartGoTo(executor, target, dir, flag, sender, maxAcc, Velocity)

    def matchPos(executor: int) -> CGeoPoint:
        """返回目标点 target，作为角色匹配位置。"""
        return target

    return skill_cpp, matchPos


def FetchBall(target, power, mode):
    """按 mode 执行与 target 有关的拿球、推球、传球、接球或拖球动作。

    mode：1 拿球、2 推球、3 传球、4 接球、5 拖球；power 为传球时的踢球力度。
    """
    def skill_cpp(executor: int) -> Any:
        """向 executor 下发指定模式、目标点和力度的球处理任务。"""
        return CppPackage.makeItFetchBall(executor, target, power, mode)

    def matchPos(executor: int) -> CGeoPoint:
        """返回当前球位，优先匹配靠近球的机器人。"""
        return Ball.pos()

    return skill_cpp, matchPos


def GetBall(flag: typing.SupportsInt = 0):
    """调用基础拿球技能 naqiu，flag 为任务标志。

    当前实现以我方球门作为角色匹配点，而非当前球位。
    """
    def skill_cpp(executor: int) -> Any:
        """向 executor 下发基础拿球任务。"""
        return CppPackage.makeItGetBall(executor, flag)

    def matchPos(executor: int) -> CGeoPoint:
        """返回当前实现使用的我方球门匹配点。"""
        return Positions.ourGoal

    return skill_cpp, matchPos

def NormalShoot(power: typing.SupportsFloat, isChip: bool = False, flag: typing.SupportsInt = 0, type: int = 0):
    #type=0：kick type=1：breakshoot
    """调用常规射门模块，以 power 力度踢球，isChip 选择是否挑踢。

    type=0 使用直接拿球踢球逻辑，type=1 使用突破射门逻辑；flag 为任务标志。
    """
    def skill_cpp(executor: int) -> Any:
        """向 executor 下发指定力度、踢球方式和射门类型的任务。"""
        return CppPackage.makeItNormalShoot(executor, power, isChip, flag, type)

    def matchPos(executor: int) -> CGeoPoint:
        """返回当前球位，优先匹配靠近球的机器人。"""
        return Ball.pos()

    return skill_cpp, matchPos

def GetBallV5(direction: typing.SupportsFloat, ifIntercept: bool = False, flag :typing.SupportsInt=0):
    """调用 V5 拿球技能，direction 为拿球时的目标朝向（弧度）。

    ifIntercept 开启底层拦截处理，flag 为任务标志。
    """
    def skill_cpp(executor: int) -> Any:
        """向 executor 下发目标朝向及拦截开关的 V5 拿球任务。"""
        return CppPackage.makeItGetBallV5(executor, direction, ifIntercept, flag)

    def matchPos(executor: int) -> CGeoPoint:
        """返回当前球位，优先匹配靠近球的机器人。"""
        return ball().Pos()

    return skill_cpp, matchPos


def PassToPos(pos: CGeoPoint | typing.Callable[[], CGeoPoint],
              kickpower: typing.SupportsFloat | None = None):
    """追球并向 pos 平传，pos 可为坐标或无参数的目标点函数。

    省略 kickpower 时按当前球位和目标自动计算保守传球力度；也可手动指定力度。
    """
    def skill_cpp(executor: int) -> Any:
        """更新目标、传球方向和力度，并向 executor 下发追踢任务。"""
        target = pos() if callable(pos) else pos
        direction = Ball.toPointDir(target)
        power = evalPower(Ball.pos(), target, P.LOW) if kickpower is None else kickpower
        return CppPackage.makeItChaseKick(executor, direction, power, flags=0)

    def matchPos(executor: int) -> CGeoPoint:
        """返回当前球位，优先匹配靠近球的机器人。"""
        return Ball.pos()

    return skill_cpp, matchPos


def GetBallV4(pos):

    """调用 V4 拿球技能，并朝向目标点 pos；pos 用于计算朝向，不是移动终点。"""
    def skill_cpp(executor: int) -> Any:
        """计算 executor 指向 pos 的方向，并下发 V4 拿球任务。"""
        direction = Player.toPointDir(pos, executor)
        return CppPackage.makeItGetBallV4(executor, direction)

    def matchPos(executor: int) -> CGeoPoint:
        """返回当前球位，优先匹配靠近球的机器人。"""
        return Ball.pos()

    return skill_cpp, matchPos

def StaticGetBallV4(pos, anti = False, flag = 0, direction: typing.Union[typing.Callable, typing.SupportsFloat] = None, power : typing.SupportsFloat = 10000, isChip : bool = False):
    """调整拿球站位并朝目标方向出球，power 为力度，isChip 选择是否挑踢。

    默认由 pos 计算朝向，anti 控制按球所在半场镜像目标的 y 坐标。
    direction 可直接指定弧度或接收 executor 的函数；flag 为任务标志，执行时开启吸球。
    """
    def skill_cpp(executor: int) -> Any:
        """解析出球朝向、开启吸球，并向 executor 下发静态拿球及踢球参数。"""
        nonlocal direction
        if direction is None:
            direction = Ball.backDir(pos, anti)()
        elif callable(direction):
            direction = direction(executor)
        CppPackage.setDribbleCommand(executor, 3)
        return CppPackage.makeItStaticGetBallV4(executor, direction, power, isChip, flag)

    def matchPos(executor: int) -> CGeoPoint:
        #return Ball.pos()
        """原样返回 pos，作为当前实现的角色匹配位置。"""
        return pos
        # return Ball.pos()

    return skill_cpp, matchPos
