"""独立动作入口：pass_ball / receive_ball / shoot_ball；不选择战术或换车。"""
import math
from dataclasses import dataclass

import CppPackage as C
from Geometry import CGeoPoint, CVector
from RoleMatch_LuaStyle.Skills import Skill
from WorldModel import Flags, kickStatus
from WorldModel.PowerCalculator import evalPower
from . import NormalPlayGeometry as geo


MOVE_FLAGS = Flags.allow_dss | Flags.dodge_our_defense_box


def move_to(robot, target, facing, cfg, *, avoid_ball=True):
    flags = MOVE_FLAGS | (Flags.dodge_ball if avoid_ball else 0)
    return Skill.SmartGoTo(target, facing, flag=flags)


@dataclass
class ContactPreparation:
    """每辆场上车独立的跨帧贴球过程，生命周期由 NormalPlay 管理。"""
    phase: str = "align"
    heading: object = None
    origin: object = None
    started_cycle: int = -1

    def reset(self):
        self.phase = "align"
        self.heading = self.origin = None
        self.started_cycle = -1

    def plan(self, robot, frame, cfg):
        if self.heading is None:
            self.heading = robot.heading
        direction = self.heading
        delta = frame.ball - robot.pos
        forward = delta.x() * math.cos(direction) + delta.y() * math.sin(direction)
        lateral = -delta.x() * math.sin(direction) + delta.y() * math.cos(direction)
        heading_error = geo.angle_error(robot.heading, direction)
        # 已确认贴球后允许原逻辑转身；丢球/出球/Play 重入时由调用者重置。
        if self.phase == "ready":
            geo.note(frame, f"CONTACT_R{robot.number}", "ready")
            return None
        aligned = (abs(lateral) <= cfg.CONTACT_LATERAL_TOLERANCE
                   and heading_error <= cfg.KICK_ANGLE)
        if not aligned:
            self.phase, self.origin, self.started_cycle = "align", None, -1
            # 只矫正横向，保留当前纵向距离，避免尚未对中就撞向球侧面。
            target = robot.pos + geo.vector(lateral, direction + math.pi / 2)
            geo.note(frame, f"CONTACT_R{robot.number}", f"align lateral={lateral:.0f}")
        else:
            if self.phase == "align":
                self.phase = "press"
                self.origin = CGeoPoint(robot.pos.x(), robot.pos.y())
                self.started_cycle = frame.cycle
            travel = robot.pos - self.origin
            advance = travel.x() * math.cos(direction) + travel.y() * math.sin(direction)
            near_mouth = cfg.MOUTH_MIN <= forward <= cfg.MOUTH_OFFSET + cfg.CONTACT_NEAR_MARGIN
            if (frame.cycle > self.started_cycle
                    and advance >= cfg.CONTACT_MIN_ADVANCE
                    and near_mouth and geo.geometric_holder(robot, frame, cfg)):
                self.phase = "ready"
                geo.note(frame, f"CONTACT_R{robot.number}", f"ready moved={advance:.0f}")
                return None
            # 目标持续位于车前，避免球已很近时固定接触点落到车后。
            # 不要求抵达穿透目标；观察到实际前移并保持球在嘴前后结束。
            step = max(cfg.CONTACT_PRESS_DEPTH,
                       forward - (cfg.MOUTH_OFFSET - cfg.CONTACT_PRESS_DEPTH))
            target = (robot.pos + geo.vector(step, direction)
                      + geo.vector(lateral, direction + math.pi / 2))
            geo.note(frame, f"CONTACT_R{robot.number}",
                     f"press moved={advance:.0f}/{cfg.CONTACT_MIN_ADVANCE:g} fwd={forward:.0f}")
        movement, match_pos = Skill.SimpleGoTo(target, direction, flag=MOVE_FLAGS)

        def execute(number):
            C.setDribbleCommand(number, 10)
            movement(number)

        return execute, match_pos


def hold_ball(robot, frame, cfg, *, capture=None):
    if capture is not None:
        preparing = capture.plan(robot, frame, cfg)
        if preparing is not None:
            return preparing
    target = frame.ball + geo.vector(-cfg.MOUTH_OFFSET, robot.heading)
    return Skill.RushTo(target, robot.heading, flag=MOVE_FLAGS, needDribble=True)


def kick_ready(robot, frame, target, cfg, label):
    """只检查车嘴位置和配置的朝向误差；不等待接应者到位或车辆降速。"""
    mouth = geo.geometric_holder(robot, frame, cfg)
    error = geo.angle_error(robot.heading, (target - frame.ball).dir())
    if not mouth:
        forward, lateral = geo.mouth_coordinates(robot, frame.ball)
        geo.note(frame, label + "_MOUTH",
                 f"R{robot.number} forward={forward:.0f} [{cfg.MOUTH_MIN},{cfg.MOUTH_MAX}] "
                 f"lateral={lateral:.0f} +/-{cfg.MOUTH_HALF_WIDTH}")
    if error > cfg.KICK_ANGLE:
        geo.note(frame, label + "_ANGLE",
                 f"R{robot.number} angle={math.degrees(error):.2f} > {math.degrees(cfg.KICK_ANGLE):g}deg")
    return mouth and error <= cfg.KICK_ANGLE


def contact_velocity(ball_velocity, direction, cfg):
    """RushTo 接收场地坐标速度向量；减去球速后，朝出球方向为 600 mm/s。"""
    return ball_velocity + geo.vector(cfg.KICK_CONTACT_SPEED, direction)


def _aim_and_kick(robot, frame, target, cfg, *, chip, level, label, release, on_fire,
                  capture=None):
    if capture is not None:
        preparing = capture.plan(robot, frame, cfg)
        if preparing is not None:
            return preparing  # 本帧只贴球，不下发踢球、不调用 on_fire。
    direction = (target - frame.ball).dir()
    center = frame.ball + geo.vector(-cfg.MOUTH_OFFSET, direction)
    movement, match_pos = Skill.RushTo(
        center, direction, flag=MOVE_FLAGS, needDribble=True,
        vel=contact_velocity(frame.velocity, direction, cfg))

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


def pass_ball(robot, frame, target, cfg, *, release=True, on_fire=None, capture=None):
    return _aim_and_kick(robot, frame, target, cfg, chip=cfg.PASS_IS_CHIP,
                         level=cfg.PASS_POWER_LEVEL, label="PASS",
                         release=release, on_fire=on_fire, capture=capture)


def shoot_ball(robot, frame, target, cfg, *, release=True, on_fire=None, capture=None):
    enemy = geo.lane_blockers(frame.ball, target, frame.enemies, cfg.LANE_WIDTH)
    if enemy:
        geo.note(frame, "SHOT_LINE", f"blocked enemy={enemy}")
    return _aim_and_kick(robot, frame, target, cfg, chip=False,
                         level=cfg.SHOT_POWER_LEVEL, label="SHOT",
                         release=release and not enemy, on_fire=on_fire, capture=capture)


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


def receive_ball(robot, frame, contact, origin, cfg, *, in_flight=False, capture=None):
    if geo.geometric_holder(robot, frame, cfg):
        return hold_ball(robot, frame, cfg, capture=capture)
    if capture is not None:
        capture.reset()
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
    # prepare、confirm、receive 都是接球任务，不能用 dodge_ball 阻止靠近球。
    return Skill.RushTo(center, facing,
                        flag=MOVE_FLAGS,
                        needDribble=True, vel=velocity)


def get_ball(robot, frame, cfg, *, capture=None):
    if geo.geometric_holder(robot, frame, cfg):
        return hold_ball(robot, frame, cfg, capture=capture)
    if capture is not None:
        capture.reset()
    distance = robot.pos.dist(frame.ball)
    geo.note(frame, f"GET_R{robot.number}",
             f"d={distance:.0f}; near<{cfg.RECEIVE_NEAR}; airborne={int(frame.airborne)}")
    if frame.airborne or distance < cfg.RECEIVE_NEAR:
        contact = frame.landing if frame.airborne else frame.ball
        origin = (contact + geo.vector(-1000, frame.velocity.dir())
                  if frame.velocity.mod() > cfg.RECEIVE_ROLL_SPEED
                  else contact + (contact - robot.pos))
        return receive_ball(robot, frame, contact, origin, cfg, in_flight=True, capture=capture)
    movement, match_pos = Skill.GetBallV5((frame.ball - robot.pos).dir(),
                                         ifIntercept=True, flag=MOVE_FLAGS)

    def execute(number):
        C.setDribbleCommand(number, 6)
        movement(number)

    return execute, match_pos
