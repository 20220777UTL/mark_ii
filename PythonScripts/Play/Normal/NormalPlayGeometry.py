"""NormalPlay 的视觉快照、几何判断；不下发运动或踢球命令。

距离 mm，速度 mm/s，角度 rad。视觉坐标已经归一化为向 +X 进攻。
"""
from dataclasses import dataclass
import math

import CppPackage as C
from Geometry import CGeoPoint, CVector
from Vision import Ball, Player, Enemy, vision
from WorldModel import Params, Positions


def vector(length, angle):
    return CVector(length * math.cos(angle), length * math.sin(angle))


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def finite(point):
    return math.isfinite(point.x()) and math.isfinite(point.y())


@dataclass
class Robot:
    number: int
    pos: CGeoPoint
    heading: float
    vel: CVector


@dataclass
class Frame:
    cycle: int
    ball: CGeoPoint
    velocity: CVector
    ours: dict
    enemies: dict
    airborne: bool
    landing: object = None


def read_frame():
    """同一帧统一读取；不缓存导入时的 native ball 对象。"""
    def robots(api, heading):
        result = {}
        for number in range(Params.maxPlayer):
            if not api.valid(number):
                continue
            pos, vel, direction = api.pos(number), api.vel(number), heading(number)
            if finite(pos) and finite(vel) and math.isfinite(direction):
                result[number] = Robot(number, CGeoPoint(pos.x(), pos.y()), direction,
                                       CVector(vel.x(), vel.y()))
        return result

    ball = Ball.ball()
    native_pos, native_vel = ball.Pos(), ball.Vel()
    # 显式复制，传球 origin 等跨帧记录不依赖 C++ 视图是否复用底层内存。
    pos = CGeoPoint(native_pos.x(), native_pos.y())
    velocity = CVector(native_vel.x(), native_vel.y())
    # 使用底层连续提供的球估计；不把 Valid 标志作为停止规划的开关。
    airborne = bool(ball.ChipBallState())
    landing = None
    if airborne:
        predicted = ball.BestChipPredictPos()
        # 飞行状态有效才用落点；无效预测不等于球已经落地。
        if (finite(predicted) and abs(predicted.x()) < Params.pitchLength / 2
                and abs(predicted.y()) < Params.pitchWidth / 2):
            landing = CGeoPoint(predicted.x(), predicted.y())
    return Frame(vision.getCycle(), pos, velocity,
                 robots(Player, Player.direction), robots(Enemy, Enemy.dir),
                 airborne, landing)


def in_mouth(robot, ball, cfg):
    delta = ball - robot.pos
    forward = delta.x() * math.cos(robot.heading) + delta.y() * math.sin(robot.heading)
    lateral = -delta.x() * math.sin(robot.heading) + delta.y() * math.cos(robot.heading)
    return (cfg.MOUTH_MIN <= forward <= cfg.MOUTH_MAX
            and abs(lateral) <= cfg.MOUTH_HALF_WIDTH)


def geometric_holder(robot, frame, cfg):
    """口前几何 + 相对速度；飞越车顶/快速擦过不算稳定持球。"""
    return (not frame.airborne and in_mouth(robot, frame.ball, cfg)
            and (frame.velocity - robot.vel).mod() <= cfg.CONTROL_REL_SPEED)


def segment_distance(point, start, end):
    """返回到有限线段的距离和未截断的投影比例，排除车后方的假挡线。"""
    dx, dy = end.x() - start.x(), end.y() - start.y()
    length2 = dx * dx + dy * dy
    if length2 < 1:
        return point.dist(start), 0.0
    t = ((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) / length2
    clamped = max(0.0, min(1.0, t))
    projection = CGeoPoint(start.x() + clamped * dx, start.y() + clamped * dy)
    return point.dist(projection), t


def lane_blockers(start, end, robots, width, *, exclude=()):
    blockers = []
    for robot in robots.values():
        distance, t = segment_distance(robot.pos, start, end)
        if robot.number not in exclude and distance < width and 0 < t <= 1.05:
            blockers.append(robot.number)
    return blockers


def legal(point, cfg):
    margin = Params.playerRadius + cfg.FIELD_MARGIN
    return (finite(point) and abs(point.x()) <= Params.pitchLength / 2 - margin
            and abs(point.y()) <= Params.pitchWidth / 2 - margin
            and not C.InOurPenaltyArea(point, margin)
            and not C.InTheirPenaltyArea(point, margin))


def legalize(point, cfg):
    """只修正运动目标；传球落点若非法则直接拒绝，不能悄悄改落点。"""
    margin = Params.playerRadius + cfg.FIELD_MARGIN
    for _ in range(3):
        point = C.MakeInField(point, margin)
        if C.InOurPenaltyArea(point, margin):
            point = C.MakeOutOfOurPenaltyArea(point, margin + 1)
        if C.InTheirPenaltyArea(point, margin):
            point = C.MakeOutOfTheirPenaltyArea(point, margin + 1)
    return point if legal(point, cfg) else None


def receive_pose(contact, origin, cfg):
    """contact 是球的接触点，不是车中心；车在球行进方向的后面迎球。"""
    facing = (origin - contact).dir()
    # 绑定只支持 点+向量、点-点，不支持 点-向量。
    return contact + vector(-cfg.MOUTH_OFFSET, facing), facing


def receiving_points(cfg, side_y):
    """先中央，再近侧中点、远侧中点、近侧边点、远侧边点；五点全部评分。

    半径覆盖矩形禁区最远角，并加车体和接球余量。仅用 penaltyDepth
    会使 30/150 度附近的车中心落入禁区。最终仍逐点检查合法性。
    """
    margin = Params.playerRadius + cfg.FIELD_MARGIN + cfg.MOUTH_OFFSET
    radius = math.hypot(Params.penaltyDepth + margin,
                        Params.penaltyWidth / 2 + margin) + cfg.ARC_EXTRA
    order = (90, 60, 120, 30, 150) if side_y >= 0 else (90, 120, 60, 150, 30)
    for degrees in order:
        angle = math.radians(degrees)
        yield CGeoPoint(Positions.theirGoal.x() - radius * math.sin(angle),
                        radius * math.cos(angle))


def goal_targets(cfg):
    y = max(0, Params.goalWidth / 2 - cfg.GOAL_INSET)
    return [CGeoPoint(Positions.theirGoal.x(), value) for value in (0, y, -y)]


def enemy_goalie(frame):
    return min(frame.enemies, key=lambda n: frame.enemies[n].pos.dist(Positions.theirGoal),
               default=-1)
