"""独立动作入口：pass_ball / receive_ball / shoot_ball；不选择战术或换车。"""
import math

import CppPackage as C
from Geometry import CVector
from RoleMatch_LuaStyle.Skills import Skill
from WorldModel import Flags, kickStatus
from WorldModel.PowerCalculator import evalPower
from . import NormalPlayGeometry as geo


MOVE_FLAGS = Flags.allow_dss | Flags.dodge_our_defense_box


def move_to(robot, target, facing, cfg, *, avoid_ball=True):
    flags = MOVE_FLAGS | (Flags.dodge_ball if avoid_ball else 0)
    return Skill.SmartGoTo(target, facing, flag=flags)


def hold_ball(robot, frame, cfg):
    target = frame.ball + geo.vector(-cfg.MOUTH_OFFSET, robot.heading)
    return Skill.RushTo(target, robot.heading, flag=MOVE_FLAGS, needDribble=True)


def kick_ready(robot, frame, target, cfg, label):
    """只检查车嘴位置和统一的 2° 朝向误差；不等待接应者到位或车辆降速。"""
    mouth = geo.geometric_holder(robot, frame, cfg)
    error = geo.angle_error(robot.heading, (target - frame.ball).dir())
    if not mouth:
        forward, lateral = geo.mouth_coordinates(robot, frame.ball)
        geo.note(frame, label + "_MOUTH",
                 f"R{robot.number} forward={forward:.0f} [{cfg.MOUTH_MIN},{cfg.MOUTH_MAX}] "
                 f"lateral={lateral:.0f} +/-{cfg.MOUTH_HALF_WIDTH}")
    if error > cfg.KICK_ANGLE:
        geo.note(frame, label + "_ANGLE",
                 f"R{robot.number} angle={math.degrees(error):.2f} > 2deg")
    return mouth and error <= cfg.KICK_ANGLE


def _aim_and_kick(robot, frame, target, cfg, *, chip, level, label, release, on_fire):
    direction = (target - frame.ball).dir()
    center = frame.ball + geo.vector(-cfg.MOUTH_OFFSET, direction)
    movement, match_pos = Skill.RushTo(center, direction, flag=MOVE_FLAGS, needDribble=True)

    def execute(number):
        movement(number)
        ready = kick_ready(robot, frame, target, cfg, label)
        if not release or not ready:
            return
        # 按要求删除本动作层对力度的拒绝条件，直接使用统一力度接口的输出。
        power = evalPower(frame.ball, target, level, is_chip=chip)
        geo.note(frame, label + "_FIRE", f"R{number} power={power} chip={int(chip)}")
        if chip:
            kickStatus.setChipKick(number, power)
        else:
            kickStatus.setKick(number, power)
        if on_fire is not None:
            on_fire()

    return execute, match_pos


def pass_ball(robot, frame, target, cfg, *, release=True, on_fire=None):
    return _aim_and_kick(robot, frame, target, cfg, chip=cfg.PASS_IS_CHIP,
                         level=cfg.PASS_POWER_LEVEL, label="PASS",
                         release=release, on_fire=on_fire)


def shoot_ball(robot, frame, target, cfg, *, release=True, on_fire=None):
    enemy = geo.lane_blockers(frame.ball, target, frame.enemies, cfg.LANE_WIDTH)
    own = geo.lane_blockers(frame.ball, target, frame.ours, cfg.LANE_WIDTH,
                           exclude=(robot.number,))
    if enemy or own:
        geo.note(frame, "SHOT_LINE", f"blocked enemy={enemy} own={own}")
    return _aim_and_kick(robot, frame, target, cfg, chip=False,
                         level=cfg.SHOT_POWER_LEVEL, label="SHOT",
                         release=release and not enemy and not own, on_fire=on_fire)


def receive_target(robot, frame, contact, origin, cfg, *, in_flight):
    velocity = CVector(0, 0)
    if in_flight:
        if frame.airborne:
            contact = frame.landing
            geo.note(frame, "RECEIVE_MODE", "chip: follow native landing without distance filter")
        elif frame.velocity.mod() > cfg.RECEIVE_ROLL_SPEED:
            delta = robot.pos - frame.ball
            time = max(0.0, min(cfg.RECEIVE_LOOKAHEAD,
                       (delta.x() * frame.velocity.x() + delta.y() * frame.velocity.y())
                       / frame.velocity.mod2()))
            contact = frame.ball + frame.velocity * time
            origin = contact + geo.vector(-1000, frame.velocity.dir())
            near = robot.pos.dist(frame.ball) < cfg.RECEIVE_NEAR
            cushion = time < cfg.CUSHION_TIME and near
            geo.note(frame, "RECEIVE_MODE",
                     f"roll v={frame.velocity.mod():.0f} > {cfg.RECEIVE_ROLL_SPEED}; "
                     f"t={time:.2f} cushion={int(cushion)} (t<{cfg.CUSHION_TIME},d<{cfg.RECEIVE_NEAR})")
            if cushion:
                velocity = geo.vector(min(cfg.CUSHION_SPEED, frame.velocity.mod() * cfg.CUSHION_RATIO),
                                      frame.velocity.dir())
        else:
            contact = frame.ball
            origin = contact + (contact - robot.pos)
            geo.note(frame, "RECEIVE_MODE",
                     f"slow ball v={frame.velocity.mod():.0f} <= {cfg.RECEIVE_ROLL_SPEED}")
    center, facing = geo.receive_pose(contact, origin, cfg)
    return center, facing, velocity


def receive_ball(robot, frame, contact, origin, cfg, *, in_flight=False):
    if in_flight and geo.geometric_holder(robot, frame, cfg):
        return hold_ball(robot, frame, cfg)
    center, facing, velocity = receive_target(robot, frame, contact, origin, cfg,
                                              in_flight=in_flight)
    distance = robot.pos.dist(frame.ball)
    delta = center - robot.pos
    small_step = distance < cfg.RECEIVE_NEAR and delta.mod() > cfg.RECEIVE_STEP
    geo.note(frame, f"RECEIVE_R{robot.number}",
             f"d={distance:.0f} near<{cfg.RECEIVE_NEAR} step={int(small_step)} "
             f"target_delta={delta.mod():.0f}")
    if small_step:
        center = robot.pos + geo.vector(cfg.RECEIVE_STEP, delta.dir())
    return Skill.RushTo(center, facing,
                        flag=MOVE_FLAGS | (Flags.dodge_ball if not in_flight else 0),
                        needDribble=True, vel=velocity)


def get_ball(robot, frame, cfg):
    distance = robot.pos.dist(frame.ball)
    geo.note(frame, f"GET_R{robot.number}",
             f"d={distance:.0f}; near<{cfg.RECEIVE_NEAR}; airborne={int(frame.airborne)}")
    if frame.airborne or distance < cfg.RECEIVE_NEAR:
        contact = frame.landing if frame.airborne else frame.ball
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
