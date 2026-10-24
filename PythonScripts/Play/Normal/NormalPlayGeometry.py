"""NormalPlay 几何与快照：位置 mm，速度 mm/s，朝向 rad；向 +X 进攻。"""
from dataclasses import dataclass, field
import math

from Geometry import CGeoPoint, CVector
from Vision import Ball, Player, Enemy, vision
from WorldModel import Params, Positions


def vector(length, angle):
    return CVector(length * math.cos(angle), length * math.sin(angle))


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


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
    messages: dict = field(default_factory=dict)


def note(frame, key, text):
    """同一帧同一条件只显示一行；键和值均用 ASCII，兼容场地图字体。"""
    frame.messages[key] = text


def read_frame(our_numbers, enemy_numbers):
    """我方固定三辆车始终读取；已发现的对手不会因一帧 Valid 变化被剔除。

    Enemy.valid 仅用于发现尚未登记的对手车号，避免把 16 个槽位全当成上场车辆。
    没有球 Valid/None 判断，没有缺帧停车、配合取消或坐标有限性淘汰。
    """
    enemy_numbers.update(n for n in range(Params.maxPlayer) if Enemy.valid(n))

    def robots(api, heading, numbers):
        result = {}
        for number in sorted(numbers):
            pos, vel = api.pos(number), api.vel(number)
            result[number] = Robot(number, CGeoPoint(pos.x(), pos.y()), heading(number),
                                   CVector(vel.x(), vel.y()))
        return result

    ball = Ball.ball()
    pos, vel = ball.Pos(), ball.Vel()
    airborne = bool(ball.ChipBallState())
    # 空中时直接采用底层预测落点，不加偏差距离或区域筛选。
    landing = ball.BestChipPredictPos() if airborne else None
    return Frame(vision.getCycle(), CGeoPoint(pos.x(), pos.y()), CVector(vel.x(), vel.y()),
                 robots(Player, Player.direction, our_numbers),
                 robots(Enemy, Enemy.dir, enemy_numbers), airborne, landing)


def mouth_coordinates(robot, ball):
    delta = ball - robot.pos
    return (delta.x() * math.cos(robot.heading) + delta.y() * math.sin(robot.heading),
            -delta.x() * math.sin(robot.heading) + delta.y() * math.cos(robot.heading))


def in_mouth(robot, ball, cfg):
    forward, lateral = mouth_coordinates(robot, ball)
    return cfg.MOUTH_MIN <= forward <= cfg.MOUTH_MAX and abs(lateral) <= cfg.MOUTH_HALF_WIDTH


def geometric_holder(robot, frame, cfg):
    """持球只看球相对车嘴的位置，不看速度、红外或空中标志。"""
    return in_mouth(robot, frame.ball, cfg)


def segment_distance(point, start, end):
    dx, dy = end.x() - start.x(), end.y() - start.y()
    length2 = dx * dx + dy * dy
    if length2 < 1:
        return point.dist(start), 0.0
    t = ((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) / length2
    ratio = max(0.0, min(1.0, t))
    return point.dist(CGeoPoint(start.x() + ratio * dx, start.y() + ratio * dy)), t


def lane_blockers(start, end, robots, width, *, exclude=()):
    blockers = []
    for robot in robots.values():
        distance, t = segment_distance(robot.pos, start, end)
        if robot.number not in exclude and distance < width and 0 < t <= 1.05:
            blockers.append(robot.number)
    return blockers


def receive_pose(contact, origin, cfg):
    facing = (origin - contact).dir()
    return contact + vector(-cfg.MOUTH_OFFSET, facing), facing


def receiving_points(cfg, side_y):
    """固定五个半圆点；返回顺序仅用于完全同分时稳定选择。"""
    radius = math.hypot(Params.penaltyDepth, Params.penaltyWidth / 2) + cfg.ARC_EXTRA
    order = (90, 60, 120, 30, 150) if side_y >= 0 else (90, 120, 60, 150, 30)
    for degrees in order:
        angle = math.radians(degrees)
        yield CGeoPoint(Positions.theirGoal.x() - radius * math.sin(angle),
                        radius * math.cos(angle))


def goal_targets(cfg):
    y = Params.goalWidth / 2 - cfg.GOAL_INSET
    return [CGeoPoint(Positions.theirGoal.x(), value) for value in (0, y, -y)]


def enemy_goalie(frame):
    return min(frame.enemies, key=lambda n: frame.enemies[n].pos.dist(Positions.theirGoal),
               default=-1)
