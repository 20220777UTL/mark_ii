"""对方任意球定位球策略：供 3V3 选手调用的查询、几何与退出判断接口。

文件末尾提供 ExampleFreeKickDefend 单状态占位，供 Config.py 导入；
可在 FreeKickDefendState.getTasks() 填写任务，退出条件默认复用 play_switch()。

3V3 共三辆车，包含一名门将和两名场上队员。本模块不分配角色，选手应在自己
的状态机中选择接口、分配实际可用的场上队员，并处理裁判状态和视觉丢失。
坐标、距离单位为毫米；使用视觉的我方坐标系，我方球门在负 X 方向。
以下函数必须在运行期间调用，不能在模块导入时缓存球或对手位置。

建议思路：先识别可能主罚的对手；一名场上队员封堵球到我方球门的路线，
另一名照看危险接应者或保护门前，门将守门。对方把球开出后结束定位球防守。
站位需要你根据规则计算，不要把查询到的对手位置直接当作我方要到达的点。

查询示例（放在运行期间执行的函数中）：
    from Play.RefPlay.ExampleFreeKickDefend import EnemyBestRobot, getFreeKickDefPos
    from Vision import Enemy
    number = EnemyBestRobot()
    opponent = Enemy.pos(number) if number >= 0 else None
    candidate = getFreeKickDefPos(0)
    if candidate is not None:
        pass  # 将候选点交给自己的站位逻辑，再检查规则和避障限制。
"""

import CppPackage
from Geometry import CGeoPoint
from Vision import Ball, Enemy
from WorldModel import Conditions, Params, Positions, Flags
from RoleMatch_LuaStyle import State, StateMachine, Task, declare_state_machine
from RoleMatch_LuaStyle.Skills import Skill


__all__ = ["ExampleFreeKickDefend", "getAttackerNum", "getFreeKickDefPos", "EnemyBestRobot", "nearest1", "play_switch"]

FreeKickDef_mode = 3  # 平台的任意球防守候选点模式；与上场人数无关。


def getAttackerNum(num):
    """查询任意球盯防优先序中的一个对手车号；AttackerNum 在此表示车号。

    参数：num 为从 0 开始的优先序，非负整数；无长度单位。
    返回：int，对手实际车号，可用于 Enemy.valid/Enemy.pos 等接口。
    含义：使用平台提供的危险程度排序；对方任意球时，优先序会
        排除距离球最近的有效对手（估计的主罚者）。此排序不是按车号或
        单一距离排序，且可能在一次定位球过程中保持相对稳定。
    局限：不保证有 num + 1 个不同候选人。平台在序号越界时
        重复最后一人，候选为空时返回 0，无法仅凭返回值区分空列表和 0 号。
        调用方必须检查 Enemy.valid，并自行去重、限制盯防人数；3V3 不能照搬
        多人盯防循环。需要在平台已经更新视觉与策略信息后查询。
        即使 Enemy.valid(0) 为 True，也不能证明 0 确实在候选序列里，可能
        误选 0 号主罚者。初次编写时可先用 nearest1 配合明确参考点自行选人，
        把本函数作为进阶参考，不把它当作保证可靠的完整名单。
    """
    if type(num) is not int:
        raise TypeError("num 必须是从 0 开始的整数优先序")
    if num < 0:
        raise ValueError("num 不能为负数")
    return CppPackage.DefenceSequence.Instance().getFreeDefNum(num)


def getFreeKickDefPos(index):
    """查询一个任意球防守候选点；候选点数量不代表需要的队员数量。

    参数：index 为整数 0..5，从 0 开始的候选编号，不是车号或角色名。
    返回：CGeoPoint（毫米）；球视觉无效时返回 None，调用方应保持等待。
    含义：0、2 是我方半场两侧，1 位于我方禁区前沿一侧，3 靠近中线，
        4 位于我方半场中路，5 位于我方半场侧翼；带侧向的点会随球所在
        正负 Y 一侧镜像；本接口返回平台维护的候选点。
    局限：必须在平台已更新当帧信息后查询；这不是已通过规则校验的移动目标，
        不保证避开球的禁入圆、禁区、其他机器人。3V3 只选择实际所需的点，
        不要按六个候选点生成六项任务。非法编号会抛出异常。
    """
    if type(index) is not int:
        raise TypeError("index 必须是 0..5 的整数候选编号")
    if not 0 <= index < 6:
        raise ValueError("任意球防守候选点只有 0..5")
    if not Ball.valid():
        return None
    return CppPackage.getPosModulePos(index, FreeKickDef_mode)


def EnemyBestRobot():
    """寻找当前距球最近的有效对手，作为可能主罚者的简单估计。

    参数：无；比较的距离单位为毫米。
    返回：int，对手实际车号；没有有效对手或球视觉无效时返回 -1。
    局限：名字中的 Best 只表示距球最近，不代表综合威胁最高或确定的主罚者。
        等距时取较小车号。扫描 Params.maxPlayer 是可识别车号范围，
        不是要求上场该数量的机器人。调用方必须先检查返回值 >= 0。
    """
    if not Ball.valid():
        return -1
    ball_pos = Ball.pos()
    best_number = -1
    best_distance = float("inf")
    for number in range(Params.maxPlayer):
        if not Enemy.valid(number):
            continue
        distance = Enemy.pos(number).dist(ball_pos)
        if distance < best_distance:
            best_number = number
            best_distance = distance
    return best_number


def nearest1(reference=None, rank=1):
    """按到参考点的距离，返回第 rank + 1 近的有效对手位置。

    参数：reference 为 CGeoPoint（毫米）或 None；None 时取我方异于球侧的
        门柱位置；球在 Y=0 时取正 Y 门柱。rank 为从 0 开始的非负整数名次，
        默认 1，表示第二近，而不是最近。可显式传 rank=0 查询最近对手。
    返回：CGeoPoint（毫米）；候选不足时返回 None。reference 为 None 且球
        视觉无效时也返回 None；显式提供参考点时不依赖球是否可见。
    局限：只按几何距离排序，不排除对方门将、主罚者或其他应忽略的对手；
        等距时取较小车号。返回的是对手位置，不能直接视作合法防守站位。
        名字中的 1 对应默认 rank=1，因此默认查询第二近的对手。
    """
    if type(rank) is not int:
        raise TypeError("rank 必须是从 0 开始的整数名次")
    if rank < 0:
        raise ValueError("rank 不能为负数")
    if reference is None:
        if not Ball.valid():
            return None
        side = -1 if Ball.posY() > 0 else 1
        reference = CGeoPoint(-Params.pitchLength / 2, side * Params.goalWidth / 2)
    candidates = sorted(
        (Enemy.pos(number).dist(reference), number)
        for number in range(Params.maxPlayer)
        if Enemy.valid(number)
    )
    if rank >= len(candidates):
        return None
    return Enemy.pos(candidates[rank][1])


def play_switch():
    """按原任意球防守的退出条件，判断何时结束定位球流程。

    先判断球已移动且当前不是 GameStop，再判断比赛是否进入进行中；
    任一成立时返回 "exit"，否则返回空字符串，继续当前状态。
    ballMoved() 表示球相对裁判基准位置移动超过 300 mm，并排除双方摆球。

    本函数只提供退出判断，不生成任务、不控制车辆。
    选手可以根据自己的战术补充其他结束条件。
    返回 "exit" 后，是否进入常规比赛仍由外层裁判调度决定。

    它不是开球许可判断。我方开球不能等待 isGameOn() 成立才第一次踢球，
    应按当前定位球策略的开球许可条件开始动作。

    调用示例：在自己的定位球策略中从本模块导入 play_switch 后：
        def transFunction(self):
            return play_switch()
    """
    if Conditions.ballMoved() and not Conditions.isGameStop():
        return "exit"
    if Conditions.isGameOn():
        return "exit"
    return ""


class FreeKickDefendState(State):
    """对方任意球时，两名场上队员挡住正面和一侧传球线路。"""

    def getTasks(self):
        tasks = {"G": Task(Skill.Goalie())}

        ball = Ball.pos()
        side, middle, other_side, side2, other_side2 = Positions.refStopAroundBall()

        # 只选场内、且不在两侧禁区附近的候选点。
        def legal(point):
            if abs(point.x()) > Params.pitchLength / 2 - 150:
                return False
            if abs(point.y()) > Params.pitchWidth / 2 - 150:
                return False
            if abs(point.y()) < Params.penaltyWidth / 2 + 200:
                if point.x() < -Params.pitchLength / 2 + Params.penaltyDepth + 200:
                    return False
                if point.x() > Params.pitchLength / 2 - Params.penaltyDepth - 200:
                    return False
            return True

        points = [p for p in (middle, side, other_side, side2, other_side2)
                  if legal(p)]
        flags = (Flags.dodge_ball | Flags.dodge_referee_area |
                 Flags.dodge_our_defense_box)

        for role, point in zip(("A", "B"), points):
            tasks[role] = Task(
                Skill.SmartGoTo(point, (ball - point).dir(), flag=flags)
            )

        for role in ("A", "B"):
            if role not in tasks:
                tasks[role] = Task(Skill.Stop())

        return tasks

    def getMatchString(self):
        return "(AB)"

    def transFunction(self):
        return play_switch()


@declare_state_machine(FreeKickDefendState)
class ExampleFreeKickDefend(StateMachine):
    pass
    """供裁判策略调度使用的我方任意球状态机。"""

