"""我方开球定位球策略的辅助接口，供选手自行编写 3V3 状态机。

文件末尾提供 ExampleKickOff 单状态占位，供 Config.py 导入；
可在 KickOffState.getTasks() 填写任务，退出条件默认复用 play_switch()。

本文件提供点位、力度计算及定位球流程的默认退出判断；
你在自己的状态机中决定哪辆车做什么、何时切换。
3V3 按总共三辆车（含守门员）考虑：选手只需给开球车、接球车和守门员
安排任务；候选点索引不是车号，不要求把所有候选点都用上。

坐标和距离单位为 mm，角度为 rad；战术坐标中我方球门在负 X 方向。
函数应在每帧需要时调用；候选点由比赛平台提供。
点位不包含路径规划或完整裁判规则检查。使用前须统一 Python/C++ 场地参数，
等待开球时须由选手保证半场、中心圆及其他规则约束。

建议思路：准备站位 -> 获准开球后对准接球点 -> 传球 -> 确认踢出后退出。
这只是思路说明；具体状态、任务、条件及超时处理由选手实现。
我方开球时可参考 Conditions.isNormalStart() 判断放行；isGameOn() 通常在
球已经开出后才为 True，不能等待它变 True 才第一次踢球。

使用例子（在比赛进程中，而非模块导入时运行）：
    from Play.RefPlay.ExampleKickOff import getKickOffPos, getAlignPos, KickPower
    receiver_pos = getKickOffPos(0)  # 为接球车选一个等待点
    kicker_pos = getAlignPos(receiver_pos, distance=500)  # 计算球后的准备点
    power = KickPower(receiver_pos)  # 传给单车踢球技能的力度
上面三行只得到计算结果，还需要你在状态机中给车辆安排移动或传球任务。
"""

import math

import CppPackage
from Geometry import CGeoPoint, CVector
from Vision import Ball
from WorldModel import Conditions
from WorldModel.PowerCalculator import P, evalPower
from RoleMatch_LuaStyle import State, StateMachine, declare_state_machine

__all__ = ["ExampleKickOff", "getKickOffPos", "getAlignPos", "receiveBias", "KickPower", "play_switch"]

KickOff_mode = 4
DirBias = math.pi / 7


def getKickOffPos(index=0):
    """读取我方开球的一个候选跑位点，返回 CGeoPoint（mm）。

    index 为 0..5 的整数，是候选点索引，不是角色或车号。
    0 可供主要接球车使用；1..5 是其他候选点。3V3 可只选其中一个，
    不能据此生成六名队员的任务。后端可能选择不同点位模板，应在同一帧
    取值并按需要在状态入口保存；不能假定它是跨次开球不变的常量。

    必须在比赛进程开始接收视觉数据后调用，不能单独运行此文件获取点位。
    场地尺寸由主办方配置；点位仍须按当前比赛规则检查。
    越界索引抛出 ValueError，不会被自动截断到有效范围。
    """
    if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index <= 5:
        raise ValueError("开球候选点 index 必须是 0..5 的整数")
    return CppPackage.getPosModulePos(index, KickOff_mode)


def getAlignPos(receiver_pos=None, distance=250):
    """计算开球车在球后方的对准点，返回 CGeoPoint（mm）。

    receiver_pos 为接球目标点；省略时读取 getKickOffPos(0)。distance 是
    沿“目标点 -> 球”方向从球继续后退的距离，必须非负。准备站位时可取
    500 mm，获准开球后靠近对准时可取 250 mm；具体距离由你决定。
    目标和球重合时方向无定义，抛出 ValueError，调用方应重新选目标。

    本函数只给出几何点，不控制车的朝向、避球、到位判断或开球许可；
    输出也不保证在场内，须由选手结合实际规则选择可用点位。
    """
    if distance < 0:
        raise ValueError("distance 不能为负数")
    if receiver_pos is None:
        receiver_pos = getKickOffPos(0)
    ball_pos = Ball.pos()
    direction = ball_pos - receiver_pos
    if direction.mod() < 1e-6:
        raise ValueError("接球目标不能与球的位置重合")
    return ball_pos + direction.unit() * distance


def receiveBias(On: bool, pd: float, dist):
    """估算接球点沿 X 方向的偏移量，始终返回 CVector（mm，可选功能）。

    On 控制是否启用；pd 是开球车朝向（rad），dist 是传球车与接球车
    距离（mm，非负）。关闭时返回零向量；开启时只沿 X 方向偏移：
    pd > pi 返回负 X，否则返回正 X，大小为 dist * (pi/7) * 3。

    这是简单的经验公式，pd 不会被自动归一化。当朝向在 [-pi, pi]
    内时，开启后总是偏向正 X；它不是通用的接球预测或物理模型，且
    偏移可能很大。选手可不用它；使用时须检查偏移后目标是否合法。
    用法：target = receiver_pos + receiveBias(False, heading, distance)。
    初次编写状态机时建议关闭，先完成不带偏移的基本传球流程。
    """
    if dist < 0:
        raise ValueError("dist 不能为负数")
    if not On:
        return CVector(0, 0)
    offset = dist * DirBias * 3
    return CVector(-offset if pd > math.pi else offset, 0)


def KickPower(pos):
    """计算球传到 pos 的常规平传力度，返回底层技能使用的力度值。

    pos 推荐为 CGeoPoint（mm），也支持 evalPower 接受的角色名、车号
    或无参点位函数。起点是当前滤波球位置 Ball.pos()，等级为 P.NORMAL。
    结果受统一力度配置限制；不是距离、球速，也不代表一定能传到。
    若传角色/车号，须先确认角色已分配、视觉有效；本函数不自动验证，
    不能用力度是否为 0 来判断角色有效性。
    本函数不踢球，选手须把结果传给自己选用的单车 Skill。
    """
    return evalPower(Ball.pos(), pos, P.NORMAL)


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


class KickOffState(State):
    """我方开球占位状态，在 getTasks 中填写具体战术。"""

    def getTasks(self):
        return {}

    def getMatchString(self):
        return ""

    def transFunction(self):
        return play_switch()


@declare_state_machine(KickOffState)
class ExampleKickOff(StateMachine):
    pass
    """供裁判策略调度使用的我方开球状态机。"""
