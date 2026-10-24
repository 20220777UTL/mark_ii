"""3V3 对方任意球防守：G 回门前，A 封主罚者射门，B 靠接应者截传球。

开球防守复用本模块的内部站位函数，只改变 B 的站位线和半场限制。
坐标、距离单位为毫米；我方球门在负 X 方向，球和对手位置每帧查询。
保留模板原有查询接口；查询得到的对手位置、候选点并不自动满足站位规则。
"""

import math

import CppPackage
import Global
from Geometry import CGeoPoint
from Vision import Ball, Enemy, Player
from WorldModel import Conditions, Params, Positions, Flags
from RoleMatch_LuaStyle import State, StateMachine, Task, declare_state_machine
from RoleMatch_LuaStyle.Skills import Skill
from .OfficialSupport import disable_ball_actions, stop_tasks


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
    """只按平台的活球判断退出，避免把球的视觉抖动当成开球完成。"""
    return "exit" if Conditions.isGameOn() else ""


def _kicker_block_pos(ball, kicker, distance):
    """默认挡球到门中心；主罚者贴球且面向门框时，改挡其朝向射线。"""
    direction = (Positions.ourGoal - ball).dir()
    if kicker >= 0 and Enemy.pos(kicker).dist(ball) < 300:
        angle = Enemy.dir(kicker)  # 视觉接口直接提供弧度，无需另写朝向识别。
        dx, dy = math.cos(angle), math.sin(angle)
        if dx < -0.01:
            t = (Positions.ourGoal.x() - ball.x()) / dx
            goal_y = ball.y() + t * dy
            if t > 0 and abs(goal_y) <= Params.goalWidth / 2:
                direction = angle
    return ball + CppPackage.Polar2Vector(distance, direction)


def _legal_defend_pos(point, ball, kickoff, *, goalie=False, occupied=()):
    """用底层几何接口修正目标；最终检查全部约束，失败则退到补防点。

    距离均为车中心距离：规则距离之外再加车半径和 60 mm 余量。
    多个投影可能互相影响，因此不能只调用一次避球/避禁区就直接下发。
    occupied 是本帧已经分配的站位，避免两名队员占同一点。
    """
    margin = Params.playerRadius + 60
    ball_gap = max(500, Params.freeKickAvoidBallDist) + margin
    separation = 2 * Params.playerRadius + 60
    enemies = [Enemy.pos(n) for n in range(Params.maxPlayer) if Enemy.valid(n)]
    # 门将只能在门前选点，不能套用场上队员的侧翼补防点。
    # 原先共用 backups，门前点被拒绝时 G 会直接跑到场地四分之一处。
    if goalie:
        post_limit = max(0, Params.goalWidth / 2 - margin)
        backups = [CGeoPoint(Positions.ourGoal.x() + margin, y)
                   for y in (0, -post_limit, post_limit)]
    else:
        backups = [CGeoPoint(-Params.pitchLength / 4, side * Params.pitchWidth / 4)
                   for side in (-1, 1)]
    for candidate in [point, *backups]:
        for _ in range(3):
            candidate = CppPackage.MakeInField(candidate, margin)
            if not goalie and CppPackage.InOurPenaltyArea(candidate, margin):
                candidate = CppPackage.MakeOutOfOurPenaltyArea(candidate, margin + 1)
            if CppPackage.InTheirPenaltyArea(candidate, 200 + margin):
                candidate = CppPackage.MakeOutOfTheirPenaltyArea(candidate, 200 + margin + 1)
            # 门前点若位于禁球圆内，尝试另一个门前点，不能把 G 沿圆推向场内。
            if goalie:
                break
            if kickoff:
                candidate = CGeoPoint(min(candidate.x(), -margin), candidate.y())
                candidate = CppPackage.MakeOutOfCircleAndInField(
                    CGeoPoint(0, 0), 500 + margin, candidate, 0)
            candidate = CppPackage.MakeOutOfCircleAndInField(ball, ball_gap, candidate, 0)

        # 球、禁区及边界约束必须同时成立；禁区修正后仍可能重新进入禁球圆。
        if not (math.isfinite(candidate.x()) and math.isfinite(candidate.y())):
            continue
        if (abs(candidate.x()) > Params.pitchLength / 2 - margin
                or abs(candidate.y()) > Params.pitchWidth / 2 - margin
                or candidate.dist(ball) < ball_gap - 0.01):
            continue
        if kickoff and (candidate.x() > -margin
                        or candidate.dist(CGeoPoint(0, 0)) < 500 + margin - 0.01):
            continue
        if not goalie and CppPackage.InOurPenaltyArea(candidate, margin):
            continue
        if CppPackage.InTheirPenaltyArea(candidate, 200 + margin):
            continue
        if any(candidate.dist(other) < separation for other in [*occupied, *enemies]):
            continue
        return candidate
    return None  # 没有合法目标时停止，不下发未检查的点。


def _defend_tasks(*, kickoff=False):
    """两种定位球的共用任务：G 守门、A 封射门、B 盯接应者。"""
    # 球权只由 NormalPlay 在重入后检测；这里不改它的内部状态。
    Global.hasFinishedRefPlay = False
    disable_ball_actions()
    if not Ball.valid():
        return stop_tasks()

    ball = Ball.pos()
    margin = Params.playerRadius + 60
    ball_gap = max(500, Params.freeKickAvoidBallDist) + margin
    kicker = EnemyBestRobot()
    # 3V3 中按当前视觉取离对方球门最近的车作为门将估计，允许门将在禁区边缘。
    # 不用 NormalPlay 的 ENEMY_GOALIE：它默认是 0，且只在常规比赛时刷新，
    # 会误删真正的 0 号接应者，并把非 0 号门将留在接应候选中。
    opponents = [n for n in range(Params.maxPlayer) if Enemy.valid(n)]
    enemy_goalie = min(opponents, key=lambda n: Enemy.pos(n).dist(Positions.theirGoal),
                       default=-1)
    receivers = [n for n in opponents
                 if n != kicker and n != enemy_goalie
                 and not CppPackage.InTheirPenaltyArea(Enemy.pos(n), 0)]
    preferred = getAttackerNum(0) if not kickoff and receivers else -1
    receiver = preferred if preferred in receivers else min(
        receivers, key=lambda n: Enemy.pos(n).dist(Positions.ourGoal), default=-1)
    # 在调试画面显示选人结果，方便直接对照场上车号定位视觉/选人问题。
    Global.debugEngine.gui_debug_msg(CGeoPoint(0, Params.pitchWidth / 2 - 200),
                                    f"DEF kicker={kicker} receiver={receiver} oppGK={enemy_goalie}")

    a_pos = _kicker_block_pos(ball, kicker, ball_gap)
    # 无接应者时，在球的另一侧、禁区前方补防。
    b_pos = CGeoPoint(Positions.ourGoal.x() + Params.penaltyDepth + 2 * margin,
                     (-1 if ball.y() >= 0 else 1) * Params.penaltyWidth / 2)
    if receiver >= 0:
        receive_pos = Enemy.pos(receiver)
        # 开球：接应者到我方球门；任意球：接应者到主罚者（丢失时用球位）。
        anchor = Positions.ourGoal if kickoff else (
            Enemy.pos(kicker) if kicker >= 0 else ball)
        length = receive_pos.dist(anchor)
        if length > 1:
            distance = min(350, length / 2)
            if kickoff and receive_pos.x() > -margin:
                # 沿同一条线退回我方半场，保留“接应者—球门”的遮挡关系。
                distance = max(distance, length * (receive_pos.x() + margin)
                               / (receive_pos.x() - anchor.x()))
            b_pos = receive_pos + CppPackage.Polar2Vector(
                distance, (anchor - receive_pos).dir())

    # 门将在球到门中心的线上守位，横向限制在两门柱内。
    # Goalie() 包含主动清球，死球阶段使用 SmartGoTo；活球后由 NormalPlay 接管。
    goalie_y = ball.y() * margin / max(1, ball.x() - Positions.ourGoal.x())
    post_limit = max(0, Params.goalWidth / 2 - margin)
    g_pos = CGeoPoint(Positions.ourGoal.x() + margin,
                      max(-post_limit, min(goalie_y, post_limit)))
    flags = (Flags.dodge_ball | Flags.avoid_stop_ball_circle
             | Flags.dodge_referee_area | Flags.allow_dss)
    if kickoff:
        flags |= Flags.avoid_half_field

    tasks, occupied = {}, []
    for role, target in (("G", g_pos), ("A", a_pos), ("B", b_pos)):
        is_goalie = role == "G"
        target = _legal_defend_pos(target, ball, kickoff,
                                   goalie=is_goalie, occupied=occupied)
        if target is None or (is_goalie and not Player.valid(Global.goalieNumber)):
            skill = Skill.Stop()
        else:
            occupied.append(target)
            if is_goalie:
                # 使用普通移动技能时，也向底层登记固定门将身份。
                CppPackage.TaskMediator.Instance().registerRole(Global.goalieNumber, "goalie")
            role_flags = flags if is_goalie else flags | Flags.dodge_our_defense_box
            skill = Skill.SmartGoTo(target, (ball - target).dir(), flag=role_flags)
        if is_goalie:
            # 标记实际下发目标；HOLD 表示门前各点均不合法或门将视觉丢失。
            Global.debugEngine.gui_debug_msg(
                target if target is not None else g_pos,
                "G HOLD" if target is None or not Player.valid(Global.goalieNumber) else "G target")
        tasks[role] = Task(skill, fixedNumber=Global.goalieNumber if is_goalie else -1)
    return tasks


class FreeKickDefendState(State):
    """A 封主罚者射门；B 贴近接应者，站在主罚者与接应者之间。"""

    def getTasks(self):
        return _defend_tasks()

    def getMatchString(self):
        # 本次定位球保持 A/B 分工，防止两车反复换人；目标仍每帧更新。
        return "(AB)"

    def transFunction(self):
        return play_switch()


@declare_state_machine(FreeKickDefendState)
class ExampleFreeKickDefend(StateMachine):
    """供裁判策略调度使用的对方任意球防守状态机。"""

