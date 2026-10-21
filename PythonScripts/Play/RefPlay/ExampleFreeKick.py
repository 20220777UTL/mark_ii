"""我方任意球定位球策略辅助接口，供 3V3 选手自行编写状态机。

文件末尾提供 ExampleFreeKick 单状态占位，供 Config.py 导入；
可在 FreeKickState.getTasks() 填写任务，退出条件默认复用 play_switch()。

这里提供位置、方向、力度计算及定位球流程的默认退出判断；
不分配角色、不生成 Task、不推进状态。
适用于我方任意球的准备、传球与射门计算。3V3 共三台车，包含一台门将；
一般由两台场上车配合，门将任务由选手的状态机另行安排。
角色参数可传选手已分配的角色名或实际车号，不要求使用特定角色名。

坐标系：场地中心为原点，我方球门在 -x，对方球门在 +x；距离单位 mm，
角度单位 rad。函数读取当前视觉帧，应在状态的计算函数内调用，勿在导入时缓存。
先用 Ball.valid() 确认当前看得见球；丢球时应由你的状态机决定等待或停止。
默认距离是战术经验值，不是比赛规则，应按本届场地和规则调整。
返回点不包含寻路或动态避障。计算结果也不代表裁判已允许触球或射门。

建议思路：准备站位 -> 取球并对准 -> 传给队友或选择射门 -> 确认踢出后退出。
第一次编写时可先用 ReceivePos、WaitBallPos2 和 KickPower 完成平传，
再按需要加入挑传、射门空当选择及传球线避让；不必一次使用全部接口。

两种接球点任选其一：ReceivePos() 按球的位置和你给的距离参数计算，容易
理解和调整；getFreeKickPos(index) 查询比赛程序自动计算的接应点，可作为跑位
参考。后者可能暂时没有可用结果，且没有前者的禁区缓冲修正，必须处理 None
并检查站位规则。它们都只给点，不分配车辆，也不保证一定能接到传球。

调用示例（在每帧计算任务时获取新结果，而不是放在文件顶部执行）：
    from Play.RefPlay.ExampleFreeKick import ReceivePos, KickPower
    target = ReceivePos()       # 获得一个接球候选点，尚未命令任何车移动
    power = KickPower(target)   # 获得传到此点的力度，尚未命令任何车踢球

使用比赛程序自动计算的接应点（在每帧运行的函数中查询）：
    from Vision import Ball
    from Play.RefPlay.ExampleFreeKick import (
        getFreeKickPos, ReceivePos, KickPower,
    )
    if Ball.valid():
        target = getFreeKickPos(0)
        if target is None:
            target = ReceivePos()  # 此处选择自行计算的点作为备用方案
        power = KickPower(target)
    # 看不见球时，由你自己的状态机处理等待或停止。
"""

import math

import CppPackage
from Geometry import CGeoPoint, CGeoSegment
from Global import debugEngine
from Utils import Normalize, Polar2Vector
from Vision import Ball, Enemy, Player
from WorldModel import Conditions, Params, Positions
from WorldModel.PowerCalculator import P, evalPower
from RoleMatch_LuaStyle import State, StateMachine, declare_state_machine

__all__ = [
    "ExampleFreeKick",
    "getChipPower_DirectKick", "ReceivePos", "getFreeKickPos",
    "WaitBallPos1", "WaitBallPos2",
    "IsTooCloseToTheirPenalty", "shouldUseChipKick", "KickPower", "canShoot",
    "getDirectShootDir", "getDirectShootDirOrFallback", "getAvoidPos",
    "DrawPenaltyAreaDebug", "play_switch",
]


# 后场挑球提示阈值，不是裁判规则或必须挑球的命令。
CHIP_X_THRESHOLD = -1000
BACKFIELD_X = -Params.pitchLength / 4


def _valid_player_number(role):
    """把角色名/车号转成有效我方车号；未分配、越界或视觉无效时返回 None。"""
    if not isinstance(role, (str, int)) or isinstance(role, bool):
        return None
    number = Player.getActualRoleNumber(role)
    if not 0 <= number < Params.maxPlayer:
        return None
    return number if Player.ourPlayer(number).Valid() else None


def _clamp_in_field(pos, margin):
    """把点 pos 裁剪到场内 margin mm；返回 CGeoPoint，不处理禁区或避障。"""
    if not 0 <= margin < min(Params.pitchLength, Params.pitchWidth) / 2:
        raise ValueError("场内边距必须非负，且小于场地短边的一半")
    return CGeoPoint(
        max(-Params.pitchLength / 2 + margin,
            min(Params.pitchLength / 2 - margin, pos.x())),
        max(-Params.pitchWidth / 2 + margin,
            min(Params.pitchWidth / 2 - margin, pos.y())),
    )


def _legal_receive_pos(pos, penalty_buffer, field_margin):
    """裁剪接球点并移出对方矩形禁区缓冲区；参数单位 mm，返回 CGeoPoint。

    仅修正进入缓冲矩形的点。选位移较短、且仍在场内的前沿或侧边；
    不处理我方禁区、机器人避障或球距。无可行边界时抛出 ValueError。
    """
    if penalty_buffer < 0:
        raise ValueError("禁区缓冲距离必须非负")
    result = _clamp_in_field(pos, field_margin)
    if not IsTooCloseToTheirPenalty(result, penalty_buffer):
        return result
    # 极小余量让结果严格位于包含边界的禁区缓冲矩形外。
    front_x = Params.pitchLength / 2 - Params.penaltyDepth - penalty_buffer - 1e-3
    side_y = Params.penaltyWidth / 2 + penalty_buffer + 1e-3
    candidates = []
    if front_x >= -Params.pitchLength / 2 + field_margin:
        candidates.append(CGeoPoint(front_x, result.y()))
    if side_y <= Params.pitchWidth / 2 - field_margin:
        candidates.append(CGeoPoint(result.x(), side_y if result.y() > 0 else -side_y))
    if not candidates:
        raise ValueError("当前禁区缓冲和场内边距没有可行接球位置")
    return min(candidates, key=lambda point: point.dist(result))


def getChipPower_DirectKick(role=None, *, target=None):
    """返回从球的原始视觉位置到目标的挑球力度，使用统一 Normal 挑球曲线。

    需要计算挑传力度时使用。role 可传接球者的角色名或车号。
    target：推荐显式传接球点（CGeoPoint），也支持角色名、车号或无参点函数；
    指定后优先于 role。推荐传明确的 CGeoPoint；角色/车号须先确认已分配且
    视觉有效，本函数不自动检查。None 或无法解析的目标可能使结果为 0，
    但不能用力度是否为 0 来判断一个角色是否有效。
    返回 float，为踢球力度指令值，不是 mm 或速度，受配置的力度上限限制。
    本函数不保证落点可达；需在实际场地标定挑球曲线。
    """
    return evalPower(Ball.rawPos(), target if target is not None else role,
                     P.NORMAL, is_chip=True)


def ReceivePos(shoot_pos=None, min_dist=1500, penalty_buffer=700, field_margin=200):
    """计算一个接球候选点，返回 CGeoPoint；所有距离参数单位为 mm。

    适合希望自己掌握接球布局、调整距离的选手；只根据球的位置和下列参数
    计算。若希望参考比赛程序自动计算的接应点，可用
    getFreeKickPos(index)，但该接口可能返回 None，且不做相同的禁区修正。

    shoot_pos：前场参考点，默认每帧按球的 y 镜像 (350/1200*场长,
    150/900*场宽)，放在球的异侧。显式给点时直接采用该点，不再镜像。
    min_dist：前/中段初始候选点与球过近时，沿球到该点方向扩距。
    penalty_buffer / field_margin：对方禁区外余量 / 场内边距。

    准备安排接球者跑位时使用：球在前/中场时让队友在前方接应，
    球在后场时让队友向中场同侧接应。三段布局：
    球 x > -场长/12 时在参考点外延 200；
    -场长/4 < x <= -场长/12 时再旋转 antiY*pi/4 后外延 200；
    更靠后时用 (-600, 球同侧*(场宽/2-1200)) 作为接球点。
    最后作禁区和场内修正，故最终点不保证 min_dist；后场段也不强制此球距。
    不检查传球线、队友或敌方占位，也不选择接球队员。
    """
    if min_dist < 0:
        raise ValueError("最小接球距离必须非负")
    ball_pos = Ball.pos()
    anti_y = Ball.antiY()
    reference = shoot_pos if shoot_pos is not None else CGeoPoint(
        350 / 1200 * Params.pitchLength,
        anti_y * 150 / 900 * Params.pitchWidth,
    )
    if ball_pos.x() > BACKFIELD_X:
        base_angle = (reference - ball_pos).dir()
        if ball_pos.x() <= -100 / 1200 * Params.pitchLength:
            base_angle += anti_y * math.pi / 4
        candidate = reference + Polar2Vector(200, base_angle)
        to_candidate = candidate - ball_pos
        if to_candidate.mod() < min_dist:
            candidate = ball_pos + Polar2Vector(min_dist, to_candidate.dir())
    else:
        candidate = CGeoPoint(-600, -anti_y * (Params.pitchWidth / 2 - 1200))
    return _legal_receive_pos(candidate, penalty_buffer, field_margin)


def getFreeKickPos(index=0, pass_line_start=None, pass_line_end=None):
    """查询比赛程序为我方任意球自动计算的接应点，返回 CGeoPoint（mm）或 None。

    比赛程序在 C++ 后台按球所在区域生成候选接球位置，并结合战术进行
    时间、传球可达性调整其中部分点。选手无需了解后台源码，可以直接
    查询这些点作为跑位参考。不是每个点都经过最优搜索，index 也不是
    优劣排名。与 ReceivePos 的公式计算不同，本函数读取后台已生成的
    结果；只调用它不会让后台重新计算，也不会命令车辆移动。

    index：0..5 的整数候选编号，不是车号。0 可先作为主要接球点参考，
    其余是可选点。3V3 仍只有两名场上队员，不要给六个点各分配一辆车。
    pass_line_start / pass_line_end：可选的传球线段起点和终点，均为
    CGeoPoint（mm），必须同时提供或同时省略。提供时，把查询到的候选点
    交给 getAvoidPos 做避让修正；不判断这条传球线是否真的可传。

    球无效、自动接应点尚未准备好、点坐标非有限数或超出当前场地时返回 None，
    你可以等待，或自行选择 ReceivePos 作为备用方案。非法编号、只给一个
    线段端点时抛出 ValueError。须在比赛程序已经更新本帧策略后调用。

    与 ReceivePos 不同，本函数没有自动移出禁区缓冲，也不保证球距、
    避障或裁判规则合法；得到点后仍要自己检查。它也无法判断一个场内点
    是否是本次任意球刚更新的结果。要固定一次传球的目标，请由自己的
    状态机保存；每帧重新查询时，候选点可能改变。不会选择接球者或下任务。
    """
    if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index <= 5:
        raise ValueError("任意球接应候选点 index 必须是 0..5 的整数")
    if (pass_line_start is None) != (pass_line_end is None):
        raise ValueError("传球线段的起点和终点必须同时提供")
    if not Ball.valid():
        return None
    base_pos = CppPackage.BeckhamDecision.Instance().get_waitPos(index)
    if (not math.isfinite(base_pos.x()) or not math.isfinite(base_pos.y())
            or abs(base_pos.x()) > Params.pitchLength / 2
            or abs(base_pos.y()) > Params.pitchWidth / 2):
        return None
    if pass_line_start is not None:
        return getAvoidPos(base_pos, pass_line_start, pass_line_end)
    return base_pos


def WaitBallPos1(offset=500):
    """第一段待球点：从球向我方且朝球同侧边线偏移 offset mm，返回 CGeoPoint。

    用于安排踢球者的第一段准备跑位；1 表示第一段，2 表示进一步对准接球者。
    默认方向为 -antiY*3*pi/4 rad；antiY 在球 y>0 时为 -1，否则为 1。
    不裁剪场地边界、不检查禁区；offset 非规则常数，需按本届避球距离设置。
    """
    if offset < 0:
        raise ValueError("待球偏移距离必须非负")
    return Ball.pos() + Polar2Vector(offset, -Ball.antiY() * math.pi * 3 / 4)


def WaitBallPos2(receiver, behind_dist=300):
    """第二段待球点：在球背离 receiver 的一侧 behind_dist mm，返回 CGeoPoint。

    用于让踢球者站到便于朝接球者传球的一侧。
    receiver 为有效我方角色名/车号，按其当前视觉位置计算。
    球与接球者重合时用 -x 为后退方向；无效接球者或负距离抛出 ValueError。
    此点帮助传球者对准接球者，不会自动满足裁判要求的避球距离或场地边界。
    """
    number = _valid_player_number(receiver)
    if number is None or behind_dist < 0:
        raise ValueError("接球者必须有效，待球距离必须非负")
    ball_pos = Ball.pos()
    away = ball_pos - Player.Pos(number)
    direction = away.dir() if away.mod() > 1e-6 else math.pi
    return ball_pos + Polar2Vector(behind_dist, direction)


def IsTooCloseToTheirPenalty(pos, buffer=250):
    """判断 pos 是否在对方禁区及 buffer mm 的矩形缓冲范围内，返回 bool。

    pos 为 CGeoPoint，buffer 必须非负；包含边界，沿 +x 方向不截断到端线。
    这是几何近距判断，未计入球员半径、裁判状态，也不判断点是否在场内。
    """
    if buffer < 0:
        raise ValueError("禁区缓冲距离必须非负")
    return (pos.x() >= Params.pitchLength / 2 - Params.penaltyDepth - buffer
            and abs(pos.y()) <= Params.penaltyWidth / 2 + buffer)


def shouldUseChipKick(x_threshold=CHIP_X_THRESHOLD):
    """球的当前 x < x_threshold mm 时返回 True；否则 False。

    选择平传或挑传前可参考此后场位置提示；它只考虑球位置，不检查传球线、
    力度、规则或落点。
    不会下发挑球指令，最终踢法由选手的状态机选择。
    """
    return Ball.posX() < x_threshold


def KickPower(pos):
    """返回球原始视觉位置到 pos 的 Normal 平传力度，使用统一曲线和上限。

    pos 支持 CGeoPoint、角色名、车号或无参点函数，推荐传明确的点。
    角色/车号须先确认已分配且视觉有效，本函数不自动检查；不能用力度
    是否为 0 来判断角色有效性。None 或无法解析的目标可能使结果为 0。
    返回 float 为力度指令值（不是距离或速度），不检查传球线或预测接球运动。
    """
    return evalPower(Ball.rawPos(), pos, P.NORMAL)


def canShoot(role, angle_limit=math.pi / 6, goal_dist=2500, ball_dist=150):
    """准备选择射门时，粗略判断 role 是否处于合适的位置，返回 bool。

    role 为我方角色名/车号；无效时返回 False。angle_limit 单位 rad，
    goal_dist / ball_dist 单位 mm，判断均为严格小于。
    条件：球员到球门中心的连线相对 +x 夹角小、距球门近、距球近。
    不是球员朝向判断，不检查红外控球、遮挡、裁判是否允许射门或二次触球。
    """
    number = _valid_player_number(role)
    if number is None:
        return False
    player_pos = Player.Pos(number)
    return (abs(Normalize((Positions.theirGoal - player_pos).dir())) < angle_limit
            and player_pos.dist(Positions.theirGoal) < goal_dist
            and Player.toBallDist(number) < ball_dist)


def _is_our_blocked(start, end, threshold_distance, exclude_num):
    """判断有效队友是否遮挡线段内部；距离单位 mm，排除 exclude_num，返回 bool。

    仅检测当前机器人中心到线段的垂距，不预测运动；不包含线段两个端点。
    """
    path = CGeoSegment(start, end)
    for number in range(Params.maxPlayer):
        if number == exclude_num or not Player.ourPlayer(number).Valid():
            continue
        player_pos = Player.Pos(number)
        projection = path.projection(player_pos)
        if (projection.dist(player_pos) < threshold_distance
                and path.IsPointOnLineOnSegment(projection)):
            return True
    return False


def getDirectShootDir(shooter, *, min_ball_x=2000, side_margin=120,
                      sample_count=10, block_threshold=200):
    """采样对方球门，返回可直射方向 float（rad）；找不到则返回 None。

    shooter 为有效我方角色名/车号；min_ball_x / side_margin / block_threshold
    单位 mm，分别为尝试直射的球 x 下限、离门柱的采样边距、对手遮挡半径。
    sample_count 为至少 2 的整数，均匀采样两门柱内侧区间。
    队友遮挡阈值为 block_threshold+100 mm，排除射手自己；只遍历有效机器人，
    所以不依赖 11V11 人数或支援角色。非法采样参数抛出 ValueError。

    准备决定是否直接射门时使用。优先选择射手当前转向最小的空当；
    所选方向还必须与 +x 的夹角小于 pi/3，否则返回 None，交给选手选择传球等动作。
    只测当前静态直线路径，线段端点不在遮挡测试内，不保证实际进球。
    本函数不修改全局变量，也不判断裁判是否允许直接射门。
    """
    if (isinstance(sample_count, bool) or not isinstance(sample_count, int)
            or sample_count < 2 or not 0 <= side_margin < Params.goalWidth / 2
            or block_threshold < 0):
        raise ValueError("球门采样数、门柱边距或遮挡半径无效")
    shooter_num = _valid_player_number(shooter)
    if shooter_num is None or not Ball.valid():
        return None
    ball_pos = Ball.rawPos()
    if ball_pos.x() < min_ball_x:
        return None
    shooter_dir = Player.direction(shooter_num)
    goal_y_min = -Params.goalWidth / 2 + side_margin
    goal_y_max = Params.goalWidth / 2 - side_margin
    best_dir = None
    best_turn = math.inf
    for index in range(sample_count):
        target_y = goal_y_min + (goal_y_max - goal_y_min) * index / (sample_count - 1)
        target = CGeoPoint(Params.pitchLength / 2, target_y)
        if (Enemy.isblocked(ball_pos, target, block_threshold)
                or _is_our_blocked(ball_pos, target, block_threshold + 100, shooter_num)):
            continue
        candidate_dir = (target - ball_pos).dir()
        turn = abs(Normalize(candidate_dir - shooter_dir))
        if turn < best_turn:
            best_turn, best_dir = turn, candidate_dir
    if best_dir is not None and abs(Normalize(best_dir)) < math.pi / 3:
        return best_dir
    return None


def getDirectShootDirOrFallback(shooter, receiver, **shoot_options):
    """返回直射方向；无空当时返回 shooter 指向 receiver 车头的方向（float，rad）。

    shooter / receiver 为我方角色名或实际车号，shoot_options 传给
    getDirectShootDir。回退车头点在接球者中心前 Params.playerFrontToCenter mm。
    需要一个“能射则射，否则对准接球者”的朝向时使用。
    回退时任一角色无效抛出 ValueError。始终返回数值角度，不修改全局开关；
    若需区分直射/传球，请直接用 getDirectShootDir，并处理它返回的 None。
    回退只给方向，不检查这条传球线是否被封堵。
    """
    direct_dir = getDirectShootDir(shooter, **shoot_options)
    if direct_dir is not None:
        return direct_dir
    shooter_num = _valid_player_number(shooter)
    receiver_num = _valid_player_number(receiver)
    if shooter_num is None or receiver_num is None:
        raise ValueError("回退传球方向需要有效的射手和接球者")
    receiver_head = Player.Pos(receiver_num) + Polar2Vector(
        Params.playerFrontToCenter, Player.direction(receiver_num))
    return (receiver_head - Player.Pos(shooter_num)).dir()


def getAvoidPos(teammate_pos, pass_line_start, pass_line_end, avoid_dist=1000,
                corridor_width=500, field_margin=200):
    """把挡住传球线的 teammate_pos 移到线段一侧，返回 CGeoPoint。

    三个位置参数均为 CGeoPoint；avoid_dist / corridor_width / field_margin
    单位 mm，分别为移到线段法向的距离、触发避让的距离阈值、场内边距。
    距有限线段小于 corridor_width 时，在最近点两侧各取 avoid_dist 的候选点，
    选择 |x| 较小的一侧（相等时选右法向），再裁剪到场内。
    线段长度接近零或无需避让时原样返回 teammate_pos。
    不改动传入点，不画图；未检查禁区/其它车，裁剪后也不保证避让净距。
    """
    if avoid_dist < 0 or corridor_width < 0:
        raise ValueError("避让距离和传球走廊宽度必须非负")
    pass_vector = pass_line_end - pass_line_start
    length = pass_vector.mod()
    if length < 1e-6:
        return teammate_pos
    relative = teammate_pos - pass_line_start
    projection_ratio = (relative.x() * pass_vector.x()
                        + relative.y() * pass_vector.y()) / (length * length)
    nearest = pass_line_start + pass_vector * max(0, min(1, projection_ratio))
    if nearest.dist(teammate_pos) >= corridor_width:
        return teammate_pos
    left = nearest + Polar2Vector(avoid_dist, pass_vector.dir() + math.pi / 2)
    right = nearest + Polar2Vector(avoid_dist, pass_vector.dir() - math.pi / 2)
    candidate = left if abs(left.x()) < abs(right.x()) else right
    return _clamp_in_field(candidate, field_margin)


def DrawPenaltyAreaDebug(buffer=200):
    """在调试图层画对方矩形禁区外 buffer mm 的三条参考边，返回 None。

    buffer 必须非负；这是本模块唯一主动绘图的接口，不生成运动任务。
    参考线不是判罚边界，也不表示球员已满足规则中的禁区限制。
    """
    if buffer < 0:
        raise ValueError("调试禁区边距必须非负")
    front_x = Params.pitchLength / 2 - Params.penaltyDepth - buffer
    back_x = Params.pitchLength / 2 + buffer
    side_y = Params.penaltyWidth / 2 + buffer
    left = CGeoPoint(front_x, side_y)
    right = CGeoPoint(front_x, -side_y)
    debugEngine.gui_debug_line(left, right, debug_color=3)
    debugEngine.gui_debug_line(left, CGeoPoint(back_x, side_y), debug_color=3)
    debugEngine.gui_debug_line(right, CGeoPoint(back_x, -side_y), debug_color=3)


def play_switch():
    """判断本次定位球流程是否可以结束。

    比赛进入进行中时返回 "exit"，通知框架本次定位球流程已完成；
    否则返回空字符串，继续当前状态。

    本函数只提供默认退出判断，不生成任务、不控制车辆。
    选手可以根据自己的战术补充踢球检测、球速或超时等结束条件，
    在自己的 transFunction 中组合判断，无需使用原策略的状态名称。
    返回 "exit" 后，是否进入常规比赛仍由外层裁判调度决定。

    它不是开球许可判断。我方开球不能等待 isGameOn() 成立才第一次踢球，
    应按当前定位球策略的开球许可条件开始动作。

    调用示例：在自己的定位球策略中从本模块导入 play_switch 后：
        def transFunction(self):
            return play_switch()
    """
    return "exit" if Conditions.isGameOn() else ""


class FreeKickState(State):
    """我方任意球占位状态，在 getTasks 中填写具体战术。"""

    def getTasks(self):
        return {}

    def getMatchString(self):
        return ""

    def transFunction(self):
        return play_switch()


@declare_state_machine(FreeKickState)
class ExampleFreeKick(StateMachine):
    pass
    """供裁判策略调度使用的我方任意球状态机。"""
