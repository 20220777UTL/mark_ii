"""两个场上队员 + 固定 Goalie；每次调度都明确安排两人的动作。

Get/Defend：我方持球→Attack，对方持球→Defend，无人持球→Get。
Attack：对方持球才转 Defend；自由球在 Attack 内回收；射门后下一次调度直接 Get。
传/接/射动作见 NormalPlayActions；几何与当帧调试消息见 NormalPlayGeometry。
"""
from dataclasses import dataclass
import math

import CppPackage as C
import Global
from Geometry import CGeoPoint
from RoleMatch_LuaStyle import State, StateMachine, Task
from RoleMatch_LuaStyle.Skills import Skill
from Vision import Ball, Player, Enemy
from WorldModel import Params, Positions, kickStatus, worldModel
from WorldModel.PowerCalculator import P
from . import NormalPlayActions as actions
from . import NormalPlayGeometry as geo


class parameters:
    # 默认仿真。SIMULATION 只选择策略参数，不修改 zss.ini 的底层运行模式。
    # 真车先标定 flat/chip 距离-力度曲线，再量车嘴几何和调整接球缓冲。
    # 当前已取消额外运动加速度限制，使用底层默认值；真车的吸球、摩擦和
    # 延迟不同，应根据实际轨迹调缓冲，而不是恢复已经删除的停车/等待条件。
    SIMULATION = True
    PASS_IS_CHIP = True
    PASS_POWER_LEVEL = P.NORMAL
    SHOT_POWER_LEVEL = P.HIGH
    # 力度只经 evalPower 获取；Low/Normal/High 是曲线名，不保证单调关系。
    MOUTH_MIN = 70
    MOUTH_MAX = 135
    MOUTH_HALF_WIDTH = 38
    MOUTH_OFFSET = Params.playerFrontToCenter + 21.5
    KICK_ANGLE = math.radians(2)  # 传球、射门统一 2°，不再按距离收紧。
    CONTROL_FRAMES = 3 if SIMULATION else 5  # 保留的球权确认次数；非 cycle 连续性检测。
    FREE_FRAMES = 2 if SIMULATION else 3

    LANE_WIDTH = Params.playerRadius + 21.5 + 50
    GOAL_INSET = 130
    ARC_EXTRA = 200
    LAUNCH_CLEARANCE = 350       # 保留 13c：起球近端对手阻挡筛选。
    MAX_RECEIVER_TRAVEL = 5000  # 保留 13d 的距离门槛，已脱离准备超时参数。
    SUPPORT_SEPARATION = 450
    SUPPORT_ENEMY_GAP = 450
    SUPPORT_REFRESH = 0.6       # 保留支援点刷新间隔；无合适点仍会跑位。
    MARK_DISTANCE = 350
    DEFEND_BALL_DISTANCE = 550
    BALL_DEPART_DISTANCE = 160 # 保留离脚双距离确认，无超时取消。

    RECEIVE_NEAR = 120
    RECEIVE_STEP = 60
    RECEIVE_ROLL_SPEED = 250
    RECEIVE_LOOKAHEAD = 0.8
    CUSHION_TIME = 0.20 if SIMULATION else 0.28
    CUSHION_SPEED = 650 if SIMULATION else 450
    CUSHION_RATIO = 0.35

    # 保留 ExampleKickOff 依赖的接口参数。
    ARRIVAL_DIST = 150
    ROBOT_MOUTH_RADIUS_MIN = 90
    ROBOT_MOUTH_RADIUS_MAX = MOUTH_MAX
    ROBOT_MOUTH_HALF_WIDTH = MOUTH_HALF_WIDTH
    KICK_OFF_RESIDUAL_VEL = 200
    LEAVE_THRESHOLD = 300
    DEFAULT_POS = CGeoPoint(3000, 3000)
    DEBUG_MSG_POS = CGeoPoint(-Params.pitchLength / 2 + 100, -200)


p = parameters
ball_control = -1


class utils:
    """开球兼容入口；几何持球不检查 Valid、速度、空中状态。"""
    WeGotBall = False
    OurLastHolder = -1
    JustKickedOff = False

    @staticmethod
    def toVector(length, angle):
        return geo.vector(length, angle)

    @staticmethod
    def kick_ball_pos(ballpos, target, prep_length=100):
        angle = (target - ballpos).dir()
        return angle, ballpos + geo.vector(-prep_length, angle)

    @staticmethod
    def arrive_at(position, num, dist=p.ARRIVAL_DIST):
        return Player.pos(num).dist(position) < dist

    @staticmethod
    def got_ball(role, isEnemy=False):
        api = Enemy if isEnemy else Player
        heading = Enemy.dir(role) if isEnemy else Player.direction(role)
        robot = geo.Robot(role, api.pos(role), heading, api.vel(role))
        result = geo.in_mouth(robot, Ball.pos(), p)
        # 真车更严格的可选判断；仿真按要求保持注释。
        # if not isEnemy:
        #     result = result and Player.isInfraredOn(role)
        return result


u, skills = utils, actions


@dataclass
class PassPlan:
    passer: int
    receiver: int
    target: CGeoPoint
    origin: CGeoPoint
    phase: str = "prepare"  # prepare → confirm → receive；不设超时。
    run_target: object = None


class _NormalState(State):
    def __init__(self, name, play, planner):
        super().__init__(name)
        self.play, self.planner = play, planner

    def getTasks(self):
        orders = self.planner()
        # 每个分支必须显式安排两人；没有默认 Stop，也不吞掉遗漏任务的问题。
        tasks = {role: Task(orders[number], fixedNumber=number)
                 for role, number in zip(("A", "B"), self.play.field_numbers)}
        tasks["G"] = Task(Skill.Goalie(), fixedNumber=self.play.goalie)
        for role, number in zip(("A", "B"), self.play.field_numbers):
            geo.note(self.play.frame, role, f"R{number} target={orders[number][1](number)}")
        return tasks

    def getMatchString(self):
        return ""

    def transFunction(self):
        return ""


class ExampleNormalPlay(StateMachine):
    def __init__(self):
        super().__init__()
        self.goalie = Global.goalieNumber
        configured = Global.controlledRobotNumbers
        if configured is None or len(configured) != 3 or self.goalie not in configured:
            raise ValueError("ExampleNormalPlay requires two field players and one goalie")
        self.field_numbers = tuple(n for n in configured if n != self.goalie)
        self.our_numbers = tuple(configured)
        self.enemy_numbers = set()
        self.states = {name: _NormalState(name, self, planner)
                       for name, planner in (("Get", self.get), ("Attack", self.attack),
                                             ("Defend", self.defend))}
        self.start_state = self.states["Get"]
        self.frame = None
        self.reset()

    def reset(self):
        global ball_control
        ball_control = -1
        self.holder = -1
        self.candidate = (-1, -1)
        self.candidate_frames = 0
        self.current_state = self.start_state
        self.isStateSwitched = True
        self.pass_plan = None
        self.after_shot_get = False
        self.support_target, self.support_cycle = None, -100000
        self.shot_target = None
        self.stage = "recover"

    @staticmethod
    def next_state(current, control):
        if control == 0:
            return "Defend"
        if control == 1:
            return "Attack"
        return "Attack" if current == "Attack" else "Get"

    def elapsed(self, cycle):
        return (self.frame.cycle - cycle) / Params.frameRate

    def update_ball_control(self):
        global ball_control
        f = self.frame
        candidates = []
        for team, robots in ((1, f.ours), (0, f.enemies)):
            for number, robot in robots.items():
                if geo.geometric_holder(robot, f, p):
                    candidates.append((robot.pos.dist(f.ball), team, number))
                elif team == 1 and number in self.field_numbers:
                    forward, lateral = geo.mouth_coordinates(robot, f.ball)
                    geo.note(f, f"MOUTH_R{number}",
                             f"forward={forward:.0f} [{p.MOUTH_MIN},{p.MOUTH_MAX}] "
                             f"lateral={lateral:.0f} +/-{p.MOUTH_HALF_WIDTH}")
        # 双方都在嘴前时，按球到车中心的几何距离选人，不再优先判敌方。
        # 真车严格红外可在我方候选加入前加 Player.isInfraredOn(number)，先保持注释。
        winner = min(candidates, key=lambda item: item[0], default=None)
        candidate = (-1, -1) if winner is None else (winner[1], winner[2])
        self.candidate_frames = self.candidate_frames + 1 if candidate == self.candidate else 1
        self.candidate = candidate
        threshold = p.FREE_FRAMES if candidate[0] == -1 else p.CONTROL_FRAMES
        if self.candidate_frames >= threshold:
            ball_control, self.holder = candidate
        else:
            geo.note(f, "CONTROL_CONFIRM",
                     f"candidate={candidate} count={self.candidate_frames}/{threshold}")
        u.WeGotBall = ball_control == 1
        u.OurLastHolder = self.holder if ball_control == 1 else -1

    def draw_debug(self):
        f = self.frame
        Global.debugEngine.gui_debug_msg(p.DEBUG_MSG_POS,
            f"Normal {self.current_state.state_name} control={ball_control} "
            f"holder={self.holder} {self.stage}", size=75)
        for index, (key, value) in enumerate(f.messages.items()):
            column, row = divmod(index, 24)
            pos = CGeoPoint(p.DEBUG_MSG_POS.x() + column * Params.pitchLength / 2,
                            p.DEBUG_MSG_POS.y() - (row + 1) * 105)
            Global.debugEngine.gui_debug_msg(pos, f"{key}: {value}", size=65)

    def planTasks(self):
        if Global.isPlaySwitched:
            self.reset()
        self.frame = geo.read_frame(self.our_numbers, self.enemy_numbers)
        geo.note(self.frame, "NATIVE_FLAGS", "DSS=on our_box=on; movement may dodge_ball")
        self.update_ball_control()
        if self.after_shot_get:
            desired = "Get"  # 发出射门指令后的下一次调度直接抢球，没有 Stop/确认等待。
            self.after_shot_get = False
        else:
            desired = self.next_state(self.current_state.state_name, ball_control)
        changed = self.isStateSwitched or desired != self.current_state.state_name
        if desired != "Attack":
            self.pass_plan, self.shot_target = None, None
        self.current_state = self.states[desired]
        if changed:
            worldModel.SPlayFSMSwitchClearAll(True)
        kickStatus.clearAll()
        for number in self.field_numbers:
            C.setDribbleCommand(number, 0)  # 保留的先清再由本帧技能设置机制。
        self.current_state.run(changed)
        self.isStateSwitched = False
        self.draw_debug()

    def five_points(self, number):
        return list(geo.receiving_points(p, self.frame.ours[number].pos.y()))

    def run_position(self, number):
        """传球后跑位：候选点→球门中心挡线对手最少，同人数取离本车最近。"""
        f, robot = self.frame, self.frame.ours[number]
        ranked = [(len(geo.lane_blockers(point, Positions.theirGoal, f.enemies, p.LANE_WIDTH)),
                   point.dist(robot.pos), point) for point in self.five_points(number)]
        count, distance, target = min(ranked, key=lambda item: item[:2])
        geo.note(f, f"RUN_R{number}", f"goal-line opponents={count} travel={distance:.0f}")
        return target

    def move_runner(self, number, target, *, avoid_ball=True):
        robot = self.frame.ours[number]
        return actions.move_to(robot, target, (self.frame.ball - robot.pos).dir(), p,
                               avoid_ball=avoid_ball)

    def marker(self, number, *, exclude=()):
        f, robot = self.frame, self.frame.ours[number]
        goalie = geo.enemy_goalie(f)
        candidates = [n for n in f.enemies if n != goalie and n not in exclude]
        target_enemy = min(candidates, key=lambda n: f.enemies[n].pos.dist(robot.pos), default=-1)
        if target_enemy >= 0:
            enemy = f.enemies[target_enemy].pos
            anchor = f.ball if enemy.dist(f.ball) > 100 else Positions.ourGoal
            target = enemy + geo.vector(min(p.MARK_DISTANCE, enemy.dist(anchor) / 2),
                                         (anchor - enemy).dir())
        else:
            geo.note(f, f"MARK_R{number}", "no non-goalie target; cover own goal")
            target = CGeoPoint(Positions.ourGoal.x() + Params.penaltyDepth + 500,
                              -800 if f.ball.y() >= 0 else 800)
        return self.move_runner(number, target)

    def get(self):
        self.stage = "getball"
        f = self.frame
        closest = min(self.field_numbers, key=lambda n: f.ours[n].pos.dist(f.ball))
        other = next(n for n in self.field_numbers if n != closest)
        return {closest: actions.get_ball(f.ours[closest], f, p), other: self.marker(other)}

    def defend(self):
        from Play.RefPlay.ExampleFreeKickDefend import _kicker_block_pos
        self.stage = "mark"
        f = self.frame
        kicker = self.holder if ball_control == 0 else min(
            f.enemies, key=lambda n: f.enemies[n].pos.dist(f.ball), default=-1)
        block = _kicker_block_pos(f.ball, kicker, p.DEFEND_BALL_DISTANCE)
        defender = min(self.field_numbers, key=lambda n: f.ours[n].pos.dist(block))
        other = next(n for n in self.field_numbers if n != defender)
        return {defender: self.move_runner(defender, block), other: self.marker(other, exclude=(kicker,))}

    def clearance_position(self, teammate, target):
        """五点中选最近的让路点。保留实际射线检查，避免跑位目标本身仍占射线。"""
        f = self.frame
        points = self.five_points(teammate.number)
        def score(point):
            distance, t = geo.segment_distance(point, f.ball, target)
            blocks = distance < p.LANE_WIDTH and 0 < t <= 1.05
            return blocks, point.dist(teammate.pos)
        return min(points, key=score)

    def choose_shot(self, shooter):
        f, robot = self.frame, self.frame.ours[shooter]
        candidates = []
        for index, target in enumerate(geo.goal_targets(p)):
            enemy = geo.lane_blockers(f.ball, target, f.enemies, p.LANE_WIDTH)
            own = geo.lane_blockers(f.ball, target, f.ours, p.LANE_WIDTH, exclude=(shooter,))
            if enemy:
                geo.note(f, f"SHOT_P{index}", f"enemy blockers={enemy}")
                continue
            if self.goalie in own:
                geo.note(f, f"SHOT_P{index}", "goalie on line; G remains Goalie")
                continue
            locked = self.shot_target is not None and self.shot_target[0] == shooter and self.shot_target[1].dist(target) < 1
            turn = geo.angle_error(robot.heading, (target - f.ball).dir())
            candidates.append(((len(own), not locked, turn), target))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    def shoot(self, shooter, target):
        f = self.frame
        self.stage = "shoot"
        self.shot_target = (shooter, target)
        other = next(n for n in self.field_numbers if n != shooter)
        clear = self.clearance_position(f.ours[other], target)

        def fired():
            self.after_shot_get = True
            self.stage = "shot->Get"

        return {
            # 明确给无球队员 dodge_ball，实际站位尚挡射线时 shoot_ball 显示原因并等待清线。
            other: self.move_runner(other, clear, avoid_ball=True),
            shooter: actions.shoot_ball(f.ours[shooter], f, target, p, on_fire=fired),
        }

    def pass_candidate(self, receiver):
        """仅保留未要求删除的近端350mm、接应路程5000mm和平传挡线条件。"""
        f, robot = self.frame, self.frame.ours[receiver]
        scored = []
        for index, target in enumerate(self.five_points(receiver)):
            center, _ = geo.receive_pose(target, f.ball, p)
            travel = robot.pos.dist(center)
            blockers = geo.lane_blockers(f.ball, target, f.enemies, p.LANE_WIDTH)
            reasons = []
            if travel > p.MAX_RECEIVER_TRAVEL:
                reasons.append(f"travel={travel:.0f}>{p.MAX_RECEIVER_TRAVEL}")
            if not p.PASS_IS_CHIP and blockers:
                reasons.append(f"flat blockers={blockers}")
            near = [(n, f.enemies[n].pos.dist(f.ball)) for n in blockers
                    if f.enemies[n].pos.dist(f.ball) < p.LAUNCH_CLEARANCE]
            if near:
                reasons.append(f"launch<{p.LAUNCH_CLEARANCE}: " +
                               ",".join(f"R{n}={distance:.0f}" for n, distance in near))
            if reasons:
                geo.note(f, f"PASS_P{index}", "; ".join(reasons))
                continue
            scored.append(((len(blockers), travel, index), target))
        return min(scored, key=lambda item: item[0])[1] if scored else None

    def support(self, number, holder):
        f, robot = self.frame, self.frame.ours[number]
        elapsed = self.elapsed(self.support_cycle)
        if self.support_target is None or elapsed >= p.SUPPORT_REFRESH:
            candidates = []
            for index, point in enumerate(self.five_points(number)):
                own_gap = point.dist(f.ours[holder].pos)
                enemy_gap = min((point.dist(r.pos) for r in f.enemies.values()), default=float("inf"))
                if own_gap < p.SUPPORT_SEPARATION or enemy_gap < p.SUPPORT_ENEMY_GAP:
                    geo.note(f, f"SUPPORT_P{index}",
                             f"holder_gap={own_gap:.0f}/{p.SUPPORT_SEPARATION} "
                             f"enemy_gap={enemy_gap:.0f}/{p.SUPPORT_ENEMY_GAP}")
                else:
                    candidates.append(point)
            # 没有合格支援点也跑位，不再返回 Stop。
            self.support_target = (min(candidates, key=lambda point: point.dist(robot.pos))
                                   if candidates else self.run_position(number))
            self.support_cycle = f.cycle
        else:
            geo.note(f, "SUPPORT_REFRESH", f"keep target elapsed={elapsed:.2f} < {p.SUPPORT_REFRESH}")
        return self.move_runner(number, self.support_target)

    def hold_and_support(self, holder):
        self.stage = "hold/reassess"
        other = next(n for n in self.field_numbers if n != holder)
        return {holder: actions.hold_ball(self.frame.ours[holder], self.frame, p),
                other: self.support(other, holder)}

    def ball_departed(self, plan):
        f = self.frame
        moved = plan.origin.dist(f.ball)
        gap = f.ours[plan.passer].pos.dist(f.ball)
        departed = moved > p.BALL_DEPART_DISTANCE and gap > p.MOUTH_MAX + 40
        if not departed:
            geo.note(f, "PASS_CONFIRM",
                     f"moved={moved:.0f}>{p.BALL_DEPART_DISTANCE}; gap={gap:.0f}>{p.MOUTH_MAX + 40}")
        return departed

    def run_pass(self):
        plan, f = self.pass_plan, self.frame
        if ball_control == 1 and self.holder == plan.receiver:
            self.pass_plan = None
            return self.attack()
        if plan.phase == "prepare":
            if not geo.geometric_holder(f.ours[plan.passer], f, p):
                self.stage = "pass/reget"
                geo.note(f, "PASS_REGET", "mouth condition false; keep plan and get ball")
                return {
                    plan.passer: actions.get_ball(f.ours[plan.passer], f, p),
                    plan.receiver: actions.receive_ball(f.ours[plan.receiver], f, plan.target, f.ball, p),
                }
            # TODO：出球前路线恶化可重新选点；按约定仅注释，不加入执行条件。
            def fired():
                plan.phase, plan.origin = "confirm", f.ball
                plan.run_target = self.run_position(plan.passer)

            self.stage = "pass/prepare"
            return {
                plan.passer: actions.pass_ball(f.ours[plan.passer], f, plan.target, p, on_fire=fired),
                plan.receiver: actions.receive_ball(f.ours[plan.receiver], f, plan.target, f.ball, p),
            }
        if plan.phase == "confirm" and self.ball_departed(plan):
            plan.phase = "receive"
        self.stage = "pass/" + plan.phase
        return {
            plan.passer: self.move_runner(plan.passer, plan.run_target),
            plan.receiver: actions.receive_ball(f.ours[plan.receiver], f, plan.target, plan.origin,
                                                 p, in_flight=plan.phase == "receive"),
        }

    def attack(self):
        f = self.frame
        if self.pass_plan is not None:
            if self.pass_plan.phase == "prepare" and ball_control == 1 and self.holder == self.pass_plan.passer:
                target = self.choose_shot(self.holder)
                if target is not None and geo.geometric_holder(f.ours[self.holder], f, p):
                    self.pass_plan = None
                    return self.shoot(self.holder, target)
            return self.run_pass()
        holder = self.holder if ball_control == 1 and self.holder in self.field_numbers else -1
        if holder < 0 or not geo.geometric_holder(f.ours[holder], f, p):
            geo.note(f, "ATTACK_REGET", "no field holder at mouth; get ball inside Attack")
            return self.get()
        target = self.choose_shot(holder)
        if target is not None:
            return self.shoot(holder, target)
        self.shot_target = None
        receiver = next(n for n in self.field_numbers if n != holder)
        target = self.pass_candidate(receiver)
        if target is not None:
            self.pass_plan = PassPlan(holder, receiver, target, f.ball)
            return self.run_pass()
        geo.note(f, "PASS_NONE", "all five failed retained conditions; hold + run support")
        return self.hold_and_support(holder)
