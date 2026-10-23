"""3V3 对方开球防守：G 回门前，A 封主罚者射门，B 站在接应者与球门之间。

复用任意球防守的内部站位函数，并增加我方半场和中圈限制。
坐标、距离单位为毫米；我方球门在负 X 方向，Left/Right 指负 Y / 正 Y。
保留模板原有查询接口；对手位置和参考点须经规则检查后才能作为移动目标。
"""

from Geometry import CGeoPoint
from Vision import Ball, Enemy
from WorldModel import Conditions, Params
from RoleMatch_LuaStyle import State, StateMachine, declare_state_machine
from Play.RefPlay.ExampleFreeKickDefend import _defend_tasks
from Play.Normal.ExampleNormalPlay import parameters as p
from Play.Normal.ExampleNormalPlay import utils as u
from Play.Normal.ExampleNormalPlay import skills as s
from Global import debugEngine

__all__ = ["ExampleKickOffDefend", "Leftpos", "Rightpos", "OTHER_POS", "EnemyBestRobot", "play_switch"]


def Leftpos(x_limit=500, y_limit=1000):
    """查找负 Y 一侧、靠近中线或进入我方半场的一个对手位置。

    参数：x_limit 为 X 上界（毫米，默认 500）；y_limit 为非负侧向阈值
        （毫米，默认 1000），候选须满足 x < x_limit 且 y < -y_limit。
    返回：CGeoPoint（毫米），取符合条件的最小有效车号，不是最近/最危险者；
        无候选时返回 (-150, -pitchWidth/4-500) 的候选点。
    局限：候选包含对方门将等所有可见对手，不保证有对象可盯防。
        返回的对手可能仍在对方半场、禁区或禁入圆内，不能直接当作合法的
        开球防守移动目标；无候选时的兜底点也需要结合当前场地与规则检查。
    """
    if y_limit < 0:
        raise ValueError("y_limit 必须是非负距离（毫米）")
    for number in range(Params.maxPlayer):
        if not Enemy.valid(number):
            continue
        if Enemy.posX(number) < x_limit and Enemy.posY(number) < -y_limit:
            return Enemy.pos(number)
    return CGeoPoint(-150, -Params.pitchWidth / 4 - 500)


def Rightpos(x_limit=500, y_limit=1000):
    """查找正 Y 一侧、靠近中线或进入我方半场的一个对手位置。

    参数：x_limit 为 X 上界（毫米，默认 500）；y_limit 为非负侧向阈值
        （毫米，默认 1000），候选须满足 x < x_limit 且 y > y_limit。
    返回：CGeoPoint（毫米），取符合条件的最小有效车号，不是最近/最危险者；
        无候选时返回 (-150, pitchWidth/4+500) 的候选点。
    局限：候选包含对方门将等所有可见对手，不保证有对象可盯防。
        返回的对手可能仍在对方半场、禁区或禁入圆内，不能直接当作合法的
        开球防守移动目标；无候选时的兜底点也需要结合当前场地与规则检查。
    """
    if y_limit < 0:
        raise ValueError("y_limit 必须是非负距离（毫米）")
    for number in range(Params.maxPlayer):
        if not Enemy.valid(number):
            continue
        if Enemy.posX(number) < x_limit and Enemy.posY(number) > y_limit:
            return Enemy.pos(number)
    return CGeoPoint(-150, Params.pitchWidth / 4 + 500)


def OTHER_POS(x=-500):
    """在可见对手人数较多的一侧给出一个我方半场候选点。

    参数：x 为候选点 X 坐标（毫米，默认 -500），选手可按场地调整。
    返回：CGeoPoint（毫米），正 Y 一侧人数严格更多时为 (x, pitchWidth/4)，
        否则为 (x, -pitchWidth/4)。Y=0 的对手计入负 Y 一侧；平局、没有
        有效对手时都选择负 Y 侧。
    局限：统计所有有效对手，不区分进攻威胁、门将或主罚者，也不排除
        另一个接口已选中的对手。此点只表达补防侧向，不负责验证半场、
        中圈、场内边界或避障规则，不决定由哪名队员前往。
    """
    negative_y_count = 0
    positive_y_count = 0
    for number in range(Params.maxPlayer):
        if not Enemy.valid(number):
            continue
        if Enemy.posY(number) > 0:
            positive_y_count += 1
        else:
            negative_y_count += 1
    side = 1 if positive_y_count > negative_y_count else -1
    return CGeoPoint(x, side * Params.pitchWidth / 4)


def EnemyBestRobot():
    """寻找当前距球最近的有效对手，作为可能开球者的简单估计。

    参数：无；比较的距离单位为毫米。
    返回：int，对手实际车号；没有有效对手或球视觉无效时返回 -1。
    局限：Best 只表示距球最近，不是完整威胁评估，也不确定其有开球权；
        等距时取较小车号。Params.maxPlayer 是可识别车号范围，和 3V3 的
        上场人数无关。调用方必须检查返回值 >= 0 后再查询该对手的位置。
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


def play_switch():
    """按原开球防守的退出条件，判断何时结束定位球流程。

    比赛进入进行中时返回 "exit"，通知框架本次定位球流程已完成；
    否则返回空字符串，继续当前状态。

    本函数保留原 play_switch() 的退出条件，不生成任务、不控制车辆。
    选手可以根据自己的战术补充其他结束条件。
    返回 "exit" 后，是否进入常规比赛仍由外层裁判调度决定。

    它不是开球许可判断。我方开球不能等待 isGameOn() 成立才第一次踢球，
    应按当前定位球策略的开球许可条件开始动作。

    调用示例：在自己的定位球策略中从本模块导入 play_switch 后：
        def transFunction(self):
            return play_switch()
    """
    if Conditions.isGameOn():
        return "exit"
    return ""


class KickOffDefendState(State):
    """A 封主罚者射门；B 尽量贴近接应者，站位目标均留在我方半场。"""

    def getTasks(self):
        u.WeGotBall = False
        u.OurLastHolder = -1
        return _defend_tasks(kickoff=True)

    def getMatchString(self):
        # 固定本次定位球的两名场上角色，G 由共用任务固定为配置的门将车号。
        return "(AB)"

    def transFunction(self):
        return play_switch()


@declare_state_machine(KickOffDefendState)
class ExampleKickOffDefend(StateMachine):
    """供裁判策略调度使用的对方开球状态机。"""
