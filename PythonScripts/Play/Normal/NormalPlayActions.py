"""可独立更换的动作层：pass_ball / receive_ball / shoot_ball。

输入是当帧快照、实际车号、目标和参数，输出标准 Skill 二元组。
这里不切换战术状态、不选择接应点，也不通过红外强行触发踢球。
以后更换接球控制器或力度标定，只需修改对应函数。
"""
import math

import CppPackage as C
from Geometry import CVector
from RoleMatch_LuaStyle.Skills import Skill
from WorldModel import Flags, Params, kickStatus
from WorldModel.PowerCalculator import evalPower
from . import NormalPlayGeometry as geo


MOVE_FLAGS = Flags.allow_dss | Flags.dodge_our_defense_box


def move_to(robot, target, facing, cfg, *, avoid_ball=True):
    target = geo.legalize(target, cfg)
    if target is None:
        return Skill.Stop()
    flags = MOVE_FLAGS | (Flags.dodge_ball if avoid_ball else 0)
    return Skill.SmartGoTo(target, facing, flag=flags, maxAcc=cfg.MOVE_ACC)


def hold_ball(robot, frame, cfg):
    """保持当前朝向、低加速度吸球，不主动踢球；不保证能抵抗贴身抢断。"""
    target = geo.legalize(frame.ball + geo.vector(-cfg.MOUTH_OFFSET, robot.heading), cfg)
    if target is None:
        return Skill.Stop()
    return Skill.RushTo(target, robot.heading, maxAcc=cfg.BALL_ACC,
                        flag=MOVE_FLAGS, needDribble=True)


def kick_ready(robot, frame, target, cfg, tolerance):
    """只认可当前几何持球、低速和对准；时间稳定性由上层连续帧确认。"""
    return (geo.geometric_holder(robot, frame, cfg)
            and robot.vel.mod() <= cfg.KICK_ROBOT_SPEED
            and geo.angle_error(robot.heading, (target - frame.ball).dir()) <= tolerance
            and geo.legal(robot.pos, cfg))


def pass_tolerance(frame, target, cfg):
    """角度误差随距离放大；远传不能只使用固定的 6° 容差。"""
    return min(cfg.PASS_ANGLE, math.atan2(cfg.PASS_AIM_ERROR, max(1, frame.ball.dist(target))))


def shot_tolerance(frame, target, cfg):
    """按目标离门柱的剩余余量收紧容差，尤其避免远距离角点射门打出门框。"""
    clearance = max(1, Params.goalWidth / 2 - abs(target.y()) - 21.5 - cfg.SHOT_AIM_MARGIN)
    return min(cfg.SHOT_ANGLE, math.atan2(clearance, max(1, frame.ball.dist(target))))


def _aim_and_kick(robot, frame, target, cfg, *, chip, level, tolerance,
                  release, on_fire):
    """共用出球执行器：先绕到球后对准；每次授权只发一帧，随后上层等离脚。

    evalPower 是唯一距离→力度入口；vt 已弃用，不能拿它限制到球速度。
    不直接采用 Shoot 的“角度满足或红外亮就踢”，以免提前踢出。
    """
    direction = (target - frame.ball).dir()
    error = math.atan2(math.sin(direction - robot.heading), math.cos(direction - robot.heading))
    aim = robot.heading + max(-cfg.AIM_STEP_ANGLE, min(cfg.AIM_STEP_ANGLE, error))
    # 分小角度绕球调整车中心和朝向，避免一次要求转 180° 时径直穿过球心。
    center = frame.ball + geo.vector(-cfg.MOUTH_OFFSET, aim)
    if not geo.legal(center, cfg):
        return hold_ball(robot, frame, cfg)
    movement, match_pos = Skill.RushTo(center, aim, maxAcc=cfg.BALL_ACC,
                                      flag=MOVE_FLAGS, needDribble=True)

    def execute(number):
        movement(number)
        if not release or not kick_ready(robot, frame, target, cfg, tolerance):
            return
        power = evalPower(frame.ball, target, level, is_chip=chip)
        if not math.isfinite(power) or power <= 0:
            return  # 无有效标定输出时保留球，不把“请求出球”误记成已经出球。
        if chip:
            kickStatus.setChipKick(number, power)
        else:
            kickStatus.setKick(number, power)
        if on_fire is not None:
            on_fire()

    return execute, match_pos


def pass_ball(robot, frame, target, cfg, *, release=False, on_fire=None):
    """配合传球唯一入口。是否挑传仅由开头 PASS_IS_CHIP 管理。"""
    return _aim_and_kick(robot, frame, target, cfg, chip=cfg.PASS_IS_CHIP,
                         level=cfg.PASS_POWER_LEVEL, tolerance=pass_tolerance(frame, target, cfg),
                         release=release, on_fire=on_fire)


def shoot_ball(robot, frame, target, cfg, *, release=False, on_fire=None):
    """射门唯一入口；释放当帧再检查双方车体，队友尚未让开就绝不出球。"""
    blocked = (geo.lane_blockers(frame.ball, target, frame.enemies, cfg.LANE_WIDTH)
               or geo.lane_blockers(frame.ball, target, frame.ours, cfg.LANE_WIDTH,
                                    exclude=(robot.number,)))
    return _aim_and_kick(robot, frame, target, cfg, chip=False,
                         level=cfg.SHOT_POWER_LEVEL, tolerance=shot_tolerance(frame, target, cfg),
                         release=release and not blocked, on_fire=on_fire)


def receive_target(robot, frame, contact, origin, cfg, *, in_flight):
    """返回车中心、迎球朝向、缓冲速度；准备阶段绝不追逐传球者脚下的球。"""
    velocity = CVector(0, 0)
    if in_flight:
        if frame.airborne:
            # 挑球只跟可信的落点；无预测时留在约定接球点，不追空中球的投影。
            if (frame.landing is not None
                    and frame.landing.dist(contact) <= cfg.CHIP_PREDICT_TOLERANCE
                    and geo.legal(frame.landing, cfg)):
                contact = frame.landing
        elif frame.velocity.mod() > cfg.RECEIVE_ROLL_SPEED:
            speed2 = frame.velocity.mod2()
            delta = robot.pos - frame.ball
            time = max(0.0, min(cfg.RECEIVE_LOOKAHEAD,
                       (delta.x() * frame.velocity.x() + delta.y() * frame.velocity.y()) / speed2))
            contact = frame.ball + frame.velocity * time
            origin = contact + geo.vector(-1000, frame.velocity.dir())  # 朝向来球。
            if time < cfg.CUSHION_TIME and robot.pos.dist(frame.ball) < cfg.RECEIVE_NEAR:
                # 顺着来球退让，减小车嘴与球的相对碰撞速度；不是迎球加速冲撞。
                speed = min(cfg.CUSHION_SPEED, frame.velocity.mod() * cfg.CUSHION_RATIO)
                velocity = geo.vector(speed, frame.velocity.dir())
        else:
            contact = frame.ball
            # 将车位关于球镜像后作为朝向参考，中心仍落在车当前这一侧。
            # 直接用 robot.pos 当 origin 会把车中心放到球的另一边，导致穿球。
            origin = contact + (contact - robot.pos)
    center, facing = geo.receive_pose(contact, origin, cfg)
    return center, facing, velocity


def receiver_ready(robot, frame, contact, origin, cfg):
    center, facing, _ = receive_target(robot, frame, contact, origin, cfg, in_flight=False)
    return (geo.legal(center, cfg) and robot.pos.dist(center) <= cfg.RECEIVE_READY_DIST
            and geo.angle_error(robot.heading, facing) <= cfg.RECEIVE_READY_ANGLE
            and robot.vel.mod() <= cfg.RECEIVE_READY_SPEED)


def receive_ball(robot, frame, contact, origin, cfg, *, in_flight=False):
    """接球唯一入口：迎球→吸球→近距离小步缓冲；本函数永不调用踢球接口。

    RushTo.vel 是到达目标的速度，并非最高速度；maxAcc 也只是加速度。
    因此近球时缩短目标步长，同时降低加速度，而不是假设 vel=0 就会慢跑。
    """
    if in_flight and geo.geometric_holder(robot, frame, cfg):
        return hold_ball(robot, frame, cfg)
    center, facing, velocity = receive_target(robot, frame, contact, origin, cfg,
                                              in_flight=in_flight)
    center = geo.legalize(center, cfg)
    if center is None:
        return Skill.Stop()
    close = robot.pos.dist(frame.ball) < cfg.RECEIVE_NEAR and not frame.airborne
    delta = center - robot.pos
    if close and delta.mod() > cfg.RECEIVE_STEP:
        center = robot.pos + geo.vector(cfg.RECEIVE_STEP, delta.dir())
    return Skill.RushTo(center, facing,
                        maxAcc=cfg.RECEIVE_ACC if close else cfg.MOVE_ACC,
                        flag=MOVE_FLAGS | (Flags.dodge_ball if not in_flight else 0),
                        needDribble=True, vel=velocity)


def get_ball(robot, frame, cfg):
    """远处交给原生拦截；近处/挑球复用接球控制，避免直接冲向球中心。"""
    if not geo.legal(frame.ball, cfg):
        # 禁区中的球让门将处理；场上队员在合法边界等候。
        return move_to(robot, frame.ball, (frame.ball - robot.pos).dir(), cfg)
    if frame.airborne or robot.pos.dist(frame.ball) < cfg.RECEIVE_NEAR:
        contact = frame.landing if frame.airborne and frame.landing is not None else frame.ball
        origin = (contact + geo.vector(-1000, frame.velocity.dir())
                  if frame.velocity.mod() > cfg.RECEIVE_ROLL_SPEED
                  else contact + (contact - robot.pos))
        return receive_ball(robot, frame, contact, origin, cfg, in_flight=True)
    movement, match_pos = Skill.GetBallV5((frame.ball - robot.pos).dir(),
                                         ifIntercept=True, flag=MOVE_FLAGS)

    def execute(number):
        C.setDribbleCommand(number, 3)
        movement(number)

    return execute, match_pos
