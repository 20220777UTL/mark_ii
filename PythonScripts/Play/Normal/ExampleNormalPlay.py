"""3V3：两个场上队员执行 Attack/Defend/Get，固定门将始终执行 Goalie。

阅读顺序：parameters → update_ball_control/next_state → attack/get/defend。
传/接/射动作独立封装在 NormalPlayActions.py，几何在 NormalPlayGeometry.py。
球权是事实，Attack 是战术记忆；传球中无人持球不会退出 Attack。

转换表       我方(1)    对方(0)    无人(-1)
Get          Attack     Defend     Get
Defend       Attack     Defend     Get
Attack       Attack     Defend     Attack（接球/内部重新抢球）
每次调度都使用底层提供的球位置继续规划，不因球有效标志或周期跳号暂停。
"""
from dataclasses import dataclass, replace
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
    # ================= 调参入口：默认仿真 =================
    # 这里只选择本策略控制参数，不修改 zss.ini 或底层仿真/真车开关。
    # 真车：先标定每辆车的 flat/chip 距离→力度，再量车嘴几何，接着低速
    # 调整加速度、缓冲提前量、稳定帧数，最后提高节奏。通信延迟、打滑、
    # 吸球、电池及反弹都与仿真有差别，下面真车分支只是待实测的保守起点。
    SIMULATION = True
    PASS_IS_CHIP = True          # 全局管理配合：True 全挑传；False 全平传。
    PASS_POWER_LEVEL = P.NORMAL  # active_curve.json 对应 chip/flat 曲线。
    SHOT_POWER_LEVEL = P.HIGH    # 射门独立使用 flat 曲线。
    # Low/Normal/High 只是曲线名，不保证 Low 在所有距离下力度更小。
    # 传过头/落点偏差先改标定，不靠接球队员撞停过强来球；evalPower.vt 无效。

    MOUTH_MIN = 70               # mm，球在车嘴前方的纵向范围。
    MOUTH_MAX = 135
    MOUTH_HALF_WIDTH = 38
    MOUTH_OFFSET = Params.playerFrontToCenter + 21.5  # 车中心到接触球心。
    CONTROL_REL_SPEED = 350 if SIMULATION else 250  # mm/s，球与车相对速度。
    CONTROL_FRAMES = 3 if SIMULATION else 5
    FREE_FRAMES = 2 if SIMULATION else 3
    # 严格红外在 utils.got_ball/update_ball_control 中保留为注释；仿真不启用。
    # 真车也要确认传感器正常，再取消相应注释。

    FIELD_MARGIN = 45
    LANE_WIDTH = Params.playerRadius + 21.5 + 50  # 车半径+球半径+余量。
    GOAL_INSET = 130             # 两个射门角点距门柱的内缩距离。
    ARC_EXTRA = 200              # 覆盖矩形禁区的半径之外，再加接应余量。
    LANDING_CLEARANCE = 450      # 接触点及接球车中心与对手的最小距离。
    LAUNCH_CLEARANCE = 350       # 挑球刚起飞时不能贴着对手强挑。
    PASS_MIN_DISTANCE = 600
    PASS_MAX_DISTANCE = 5500
    SUPPORT_SEPARATION = 450
    SUPPORT_EST_SPEED = 1000    # mm/s，仅用于淘汰准备超时内难以到达的点。
    HOLD_LOST_DISTANCE = 280    # 准备传球时球嘴短暂脱离，在此范围内先贴回球。
    MARK_DISTANCE = 350
    DEFEND_BALL_DISTANCE = 550   # 活球站位距离，不套定位球禁球规则。

    MOVE_ACC = 1800 if SIMULATION else 1200       # mm/s²
    BALL_ACC = 850 if SIMULATION else 600
    RECEIVE_ACC = 650 if SIMULATION else 450
    KICK_ROBOT_SPEED = 250 if SIMULATION else 180  # 出球前车速上限 mm/s
    PASS_ANGLE = math.radians(6)
    SHOT_ANGLE = math.radians(4)
    PASS_AIM_ERROR = 80         # mm，远距离传球按可接受横向误差收紧角度。
    SHOT_AIM_MARGIN = 40        # mm，角点射门额外保留门柱误差余量。
    AIM_STEP_ANGLE = math.radians(8 if SIMULATION else 5)  # 持球绕球转向的单步角。
    READY_FRAMES = 3 if SIMULATION else 5
    RECEIVE_READY_DIST = 100 if SIMULATION else 80
    RECEIVE_READY_ANGLE = math.radians(15)
    RECEIVE_READY_SPEED = 250 if SIMULATION else 180
    RECEIVE_NEAR = 650
    RECEIVE_STEP = 60 if SIMULATION else 40  # 近球目标步长，不是速度上限。
    RECEIVE_ROLL_SPEED = 250
    RECEIVE_LOOKAHEAD = 0.8      # 秒，地滚球拦截预测上限，真车按延迟调。
    CUSHION_TIME = 0.20 if SIMULATION else 0.28
    CUSHION_SPEED = 650 if SIMULATION else 450
    CUSHION_RATIO = 0.35         # 近球时顺着来球速度退让的比例。
    CHIP_PREDICT_TOLERANCE = 900 # 原生落点偏差太大时，仍等在约定点。
    CHIP_MIN_RECEIVE_TIME = 0.35 # 秒，补足仿真可能没有空中标志的初始等待。
    # 此时间不是完整飞行时间，优先服从 native ChipBallState，按实测再调。

    PASS_PREPARE_TIMEOUT = 5.0  # 秒，等不到接应者则护球后重新评估。
    KICK_CONFIRM_TIMEOUT = 0.6  # 下发指令≠已出球；等待视觉确认离脚。
    RECEIVE_TIMEOUT = 4.0       # 接球超时仍留在 Attack 内抢球。
    RETRY_DELAY = 0.4
    BALL_DEPART_DISTANCE = 160
    SUPPORT_REFRESH = 0.6       # 护球期间调整支援的间隔，避免每帧摆动。

    # 旧开球脚本依赖的兼容名字。
    ARRIVAL_DIST = 150
    ROBOT_MOUTH_RADIUS_MIN = 90
    ROBOT_MOUTH_RADIUS_MAX = MOUTH_MAX
    ROBOT_MOUTH_HALF_WIDTH = MOUTH_HALF_WIDTH
    KICK_OFF_RESIDUAL_VEL = 200
    LEAVE_THRESHOLD = 300
    DEFAULT_POS = CGeoPoint(3000, 3000)
    DEBUG_MSG_POS = CGeoPoint(0, -Params.pitchWidth / 2 + 100)


p = parameters
ball_control = -1


class utils:
    """保留定位球脚本的导入；旧 WeGotBall 不再决定新状态机。"""
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
        return num >= 0 and Player.valid(num) and Player.pos(num).dist(position) < dist

    @staticmethod
    def got_ball(role, isEnemy=False):
        api = Enemy if isEnemy else Player
        if role < 0 or not api.valid(role):
            return False
        heading = Enemy.dir(role) if isEnemy else Player.direction(role)
        robot = geo.Robot(role, api.pos(role), heading, api.vel(role))
        result = (not Ball.ball().ChipBallState() and geo.in_mouth(robot, Ball.pos(), p)
                  and (Ball.vel() - robot.vel).mod() <= p.CONTROL_REL_SPEED)
        # 真车严格判断（仿真先注释）：
        # if not isEnemy:
        #     result = result and Player.isInfraredOn(role)
        return result


u, skills = utils, actions  # ExampleKickOff 的兼容导入。


@dataclass
class PassPlan:
    passer: int
    receiver: int
    target: CGeoPoint          # 球的接触点，不是车中心。
    origin: CGeoPoint
    created: int
    phase: str = "prepare"    # prepare → confirm（发指令）→ receive（确认离脚）。
    sent: object = None
    departed: object = None


class _NormalState(State):
    """使用框架 Task/角色注册，A/B 固定实际车号，配合中不会重新匹配换车。"""
    def __init__(self, name, play, planner):
        super().__init__(name)
        self.play, self.planner = play, planner

    def getTasks(self):
        orders = self.planner()
        tasks = {role: Task(orders.get(number, Skill.Stop()), fixedNumber=number)
                 for role, number in zip(("A", "B"), self.play.field_numbers)}
        tasks["G"] = Task(Skill.Goalie(), fixedNumber=self.play.goalie)
        return tasks

    def getMatchString(self):
        return ""

    def transFunction(self):
        return ""  # 在下发任务之前统一转换。


class ExampleNormalPlay(StateMachine):
    def __init__(self):
        super().__init__()
        self.goalie = Global.goalieNumber
        configured = Global.controlledRobotNumbers
        if configured is None or len(configured) != 3 or self.goalie not in configured:
            raise ValueError("ExampleNormalPlay 需要配置三辆车（含固定门将）")
        self.field_numbers = tuple(n for n in configured if n != self.goalie)
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
        self.shot_pending = None
        self.ready_key, self.ready_frames = None, 0
        self.retry_after = 0
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
        """敌我使用各自朝向；按连续调用的判断结果累计，不检查视觉周期连续性。"""
        global ball_control
        f = self.frame
        ours = [n for n in (*self.field_numbers, self.goalie)
                if n in f.ours and geo.geometric_holder(f.ours[n], f, p)]
        if self.chip_settling():
            # 只延后本次接球队员的成功判定；不屏蔽对手抢到球的几何证据。
            ours = [n for n in ours if n != self.pass_plan.receiver]
        # 真车可选更严格条件（仿真不启用）：
        # ours = [n for n in ours if Player.isInfraredOn(n)]
        enemies = [n for n, robot in f.enemies.items() if geo.geometric_holder(robot, f, p)]
        # 同时贴球按争抢保守判对方，但仍须满足连续帧数。
        if enemies:
            candidate = (0, min(enemies, key=lambda n: f.enemies[n].pos.dist(f.ball)))
        elif ours:
            candidate = (1, min(ours, key=lambda n: f.ours[n].pos.dist(f.ball)))
        else:
            candidate = (-1, -1)
        self.candidate_frames = self.candidate_frames + 1 if candidate == self.candidate else 1
        self.candidate = candidate
        threshold = p.FREE_FRAMES if candidate[0] == -1 else p.CONTROL_FRAMES
        if self.candidate_frames >= threshold:
            ball_control, self.holder = candidate
        u.WeGotBall = ball_control == 1
        u.OurLastHolder = self.holder if ball_control == 1 else -1

    def planTasks(self):
        """先转换后执行：确认对方拿球的本帧就取消配合。

        沿用基类清踢球和 State.run 协议；不修改公共框架或其他裁判策略。
        """
        if Global.isPlaySwitched:
            self.reset()
        # 重复周期、跳号、球有效标志变化均不打断规划或清除配合。
        self.frame = geo.read_frame()
        self.update_ball_control()
        desired = self.next_state(self.current_state.state_name, ball_control)
        changed = self.isStateSwitched or desired != self.current_state.state_name
        if desired != "Attack":
            self.pass_plan, self.shot_pending, self.shot_target = None, None, None
            self.ready_key, self.ready_frames = None, 0
        self.current_state = self.states[desired]
        if changed:
            worldModel.SPlayFSMSwitchClearAll(True)
        kickStatus.clearAll()
        for number in self.field_numbers:
            C.setDribbleCommand(number, 0)  # 仅本帧持球/接球动作明确开吸球。
        self.current_state.run(changed)
        self.isStateSwitched = False
        Global.debugEngine.gui_debug_msg(p.DEBUG_MSG_POS,
            f"Normal {desired} control={ball_control} holder={self.holder} {self.stage}")

    def field_players(self):
        return [n for n in self.field_numbers if n in self.frame.ours]

    def stable_ready(self, key, ready):
        # 这里只看同一动作的就绪条件；周期跳号不会额外清零。
        consecutive = self.ready_key == key
        self.ready_frames = self.ready_frames + 1 if ready and consecutive else int(ready)
        self.ready_key = key
        return self.ready_frames >= p.READY_FRAMES

    def chip_settling(self):
        return (self.pass_plan is not None and self.pass_plan.sent is not None
                and p.PASS_IS_CHIP and self.elapsed(self.pass_plan.sent) < p.CHIP_MIN_RECEIVE_TIME)

    def marker(self, number, *, exclude=()):
        """盯离本车最近的非门将，站到其与球之间；无候选则在禁区前补防。"""
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
            target = CGeoPoint(Positions.ourGoal.x() + Params.penaltyDepth + 500,
                              -800 if f.ball.y() >= 0 else 800)
        return actions.move_to(robot, target, (f.ball - robot.pos).dir(), p)

    def get(self):
        """Get 和 Attack 内自由球回收共用；只有最近一人抢球。"""
        self.stage = "recover"
        players = self.field_players()
        if not players:
            return {}
        closest = min(players, key=lambda n: self.frame.ours[n].pos.dist(self.frame.ball))
        orders = {closest: actions.get_ball(self.frame.ours[closest], self.frame, p)}
        for n in players:
            if n != closest:
                orders[n] = self.marker(n)
        return orders

    def defend(self):
        """借用 ExampleFreeKickDefend 封射门几何，另一人盯非持球接应者。

        不直接调用 _defend_tasks：其中含定位球距离、禁踢球和门将静止逻辑。
        """
        from Play.RefPlay.ExampleFreeKickDefend import _kicker_block_pos
        self.stage = "mark"
        players, f = self.field_players(), self.frame
        if not players:
            return {}
        kicker = self.holder if ball_control == 0 and self.holder in f.enemies else min(
            f.enemies, key=lambda n: f.enemies[n].pos.dist(f.ball), default=-1)
        block = _kicker_block_pos(f.ball, kicker, p.DEFEND_BALL_DISTANCE)
        defender = min(players, key=lambda n: f.ours[n].pos.dist(block))
        orders = {defender: actions.move_to(f.ours[defender], block,
                                           (f.ball - f.ours[defender].pos).dir(), p)}
        for n in players:
            if n != defender:
                orders[n] = self.marker(n, exclude=(kicker,))
        return orders

    def clearance_position(self, teammate, target, shooter):
        """在射线两侧找合法让路点；按实际车位清线，不按目标点假定已让开。"""
        f = self.frame
        direction = (target - f.ball).dir()
        distance, t = geo.segment_distance(teammate.pos, f.ball, target)
        if distance >= p.LANE_WIDTH or t <= 0 or t > 1.05:
            return teammate.pos
        projection = f.ball + (target - f.ball) * max(0, min(1, t))
        candidates = [projection + geo.vector(p.LANE_WIDTH + p.SUPPORT_SEPARATION / 2,
                                               direction + sign * math.pi / 2)
                      for sign in (-1, 1)]
        obstacles = [r.pos for n, r in f.ours.items() if n != teammate.number]
        obstacles += [r.pos for r in f.enemies.values()]
        candidates = [point for point in candidates if geo.legal(point, p)
                      and all(point.dist(other) >= 2 * Params.playerRadius + p.FIELD_MARGIN
                              for other in obstacles)
                      and point.dist(f.ours[shooter].pos) >= p.SUPPORT_SEPARATION]
        return min(candidates, key=lambda point: point.dist(teammate.pos), default=None)

    def choose_shot(self, shooter):
        """检查中心和两角：先能立即射的，再可让队友清线的；保留可用瞄准点。"""
        f, robot = self.frame, self.frame.ours[shooter]
        candidates = []
        for target in geo.goal_targets(p):
            if geo.lane_blockers(f.ball, target, f.enemies, p.LANE_WIDTH):
                continue
            own = geo.lane_blockers(f.ball, target, f.ours, p.LANE_WIDTH, exclude=(shooter,))
            if any(n not in self.field_numbers for n in own):
                continue  # 门将始终 Goalie，不为射门改派其他动作。
            if any(self.clearance_position(f.ours[n], target, shooter) is None for n in own):
                continue
            clearance = min((geo.segment_distance(r.pos, f.ball, target)[0]
                             for r in f.enemies.values()), default=10000)
            turn = geo.angle_error(robot.heading, (target - f.ball).dir())
            # 无障碍优先；同等级保留上帧目标，避免队友让路过程中反复换方向。
            locked = (self.shot_target is not None and self.shot_target[0] == shooter
                      and self.shot_target[1].dist(target) < 1)
            candidates.append(((len(own), not locked, turn, -clearance), target))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    def shoot(self, shooter, target):
        """调度射门：队友让路 → 连续对准/清线 → 单次出球 → 等视觉离脚。"""
        f = self.frame
        self.stage = "shoot/clear"
        self.shot_target = (shooter, target)
        orders = {}
        for n in self.field_players():
            if n == shooter:
                continue
            robot = f.ours[n]
            clear = self.clearance_position(robot, target, shooter)
            orders[n] = (actions.move_to(robot, clear, (f.ball - robot.pos).dir(), p)
                         if clear is not None else Skill.Stop())
        clear = (not geo.lane_blockers(f.ball, target, f.enemies, p.LANE_WIDTH)
                 and not geo.lane_blockers(f.ball, target, f.ours, p.LANE_WIDTH,
                                          exclude=(shooter,)))
        ready = clear and actions.kick_ready(f.ours[shooter], f, target, p,
                                             actions.shot_tolerance(f, target, p))
        release = self.stable_ready(("shoot", shooter, target.x(), target.y()), ready)

        def fired():
            self.shot_pending = (shooter, f.ball, f.cycle)
            self.ready_frames = 0

        orders[shooter] = actions.shoot_ball(f.ours[shooter], f, target, p,
                                             release=release, on_fire=fired)
        return orders

    def pass_candidate(self, passer, receiver):
        """五点全部评估：先排除不能安全落地/接应的点，再按挡线人数排序。

        同人数按接应距离、接球后可射门通道数排序，最后用确定的角度顺序。
        挑球中段可以越人，但近端和落点必须有余量；平传则整条线必须畅通。
        """
        f, robot = self.frame, self.frame.ours[receiver]
        scored = []
        for index, target in enumerate(geo.receiving_points(p, robot.pos.y())):
            center, _ = geo.receive_pose(target, f.ball, p)
            distance = target.dist(f.ball)
            if (not geo.legal(target, p) or not geo.legal(center, p)
                    or not p.PASS_MIN_DISTANCE <= distance <= p.PASS_MAX_DISTANCE):
                continue
            if any(min(r.pos.dist(target), r.pos.dist(center)) < p.LANDING_CLEARANCE
                   for r in f.enemies.values()):
                continue
            # 用距离/期望跑速粗估能否在准备超时内到位；实际出球仍等到位。
            travel = robot.pos.dist(center)
            if travel > p.PASS_PREPARE_TIMEOUT * p.SUPPORT_EST_SPEED:
                continue
            blockers = geo.lane_blockers(f.ball, target, f.enemies, p.LANE_WIDTH)
            if not p.PASS_IS_CHIP and blockers:
                continue
            if any(f.enemies[n].pos.dist(f.ball) < p.LAUNCH_CLEARANCE for n in blockers):
                continue
            own = geo.lane_blockers(f.ball, target, f.ours, p.LANE_WIDTH,
                                    exclude=(passer, receiver))
            if own:
                continue
            openings = sum(not geo.lane_blockers(target, goal, f.enemies, p.LANE_WIDTH)
                           for goal in geo.goal_targets(p))
            scored.append(((len(blockers), travel, -openings, index), target))
        return min(scored, key=lambda item: item[0])[1] if scored else None

    def support(self, number, holder):
        """暂无安全传球点：护球者保持球，队友调整支援位置，然后重评。

        TODO（待用户复议）：全部候选不适合时，这种等待是否会拖慢进攻？
        后续可比较持球推进或退传；本版先按约定保留护球等待。
        """
        f, robot = self.frame, self.frame.ours[number]
        if self.support_target is None or self.elapsed(self.support_cycle) >= p.SUPPORT_REFRESH:
            candidates = list(geo.receiving_points(p, robot.pos.y()))
            # 禁区前五点都被占时，增加球侧支援位置，不将其当作本轮传球落点。
            candidates += [f.ball + geo.vector(1000, angle)
                           for angle in (-math.pi / 2, math.pi / 2, math.pi)]
            candidates = [point for point in candidates if geo.legal(point, p)
                          and point.dist(f.ours[holder].pos) >= p.SUPPORT_SEPARATION
                          and all(point.dist(r.pos) >= p.LANDING_CLEARANCE
                                  for r in f.enemies.values())]
            self.support_target = min(candidates, key=lambda point: point.dist(robot.pos), default=None)
            self.support_cycle = f.cycle
        if self.support_target is None:
            return Skill.Stop()
        return actions.move_to(robot, self.support_target, (f.ball - robot.pos).dir(), p)

    def hold_and_support(self, holder):
        self.stage = "hold/reassess"
        orders = {holder: actions.hold_ball(self.frame.ours[holder], self.frame, p)}
        for number in self.field_players():
            if number != holder:
                orders[number] = self.support(number, holder)
        return orders

    def ball_departed(self, passer, origin):
        """只检查视觉位移/离开车嘴，不使用只代表下发指令的 IsBallKicked。"""
        f = self.frame
        return (origin.dist(f.ball) > p.BALL_DEPART_DISTANCE
                and f.ours[passer].pos.dist(f.ball) > p.MOUTH_MAX + 40)

    def cancel_pass(self, *, completed=False):
        self.pass_plan = None
        self.ready_key, self.ready_frames = None, 0
        # 接球成功后可立即寻找射门；只有失败重试才需要冷却。
        self.retry_after = self.frame.cycle + (0 if completed else round(p.RETRY_DELAY * Params.frameRate))

    def run_pass(self):
        """一次配合锁定实际车号和接触点；None 表示结束/失败，回上层重新决策。"""
        plan, f = self.pass_plan, self.frame
        if plan.passer not in f.ours or plan.receiver not in f.ours:
            self.cancel_pass()
            return None
        if ball_control == 1 and self.holder == plan.receiver:
            self.cancel_pass(completed=True)
            return None
        if plan.phase == "prepare":
            if self.elapsed(plan.created) > p.PASS_PREPARE_TIMEOUT:
                self.cancel_pass()
                return None
            if not geo.geometric_holder(f.ours[plan.passer], f, p):
                if f.ours[plan.passer].pos.dist(f.ball) > p.HOLD_LOST_DISTANCE:
                    self.cancel_pass()
                    return None
                # 一两帧丢失车嘴几何不重选落点；贴回球，重新累计出球稳定帧。
                self.ready_frames = 0
                self.stage = "pass/settle"
                return {
                    plan.passer: actions.get_ball(f.ours[plan.passer], f, p),
                    plan.receiver: actions.receive_ball(f.ours[plan.receiver], f, plan.target,
                                                         f.ball, p),
                }
            # TODO：出球前若路线明显恶化，可重新选点。本版按要求只记注释，
            # 不实现路线恶化重选；本次准备期间始终保持选定的五点之一。
            ready = (actions.receiver_ready(f.ours[plan.receiver], f, plan.target, f.ball, p)
                     and actions.kick_ready(f.ours[plan.passer], f, plan.target, p,
                                             actions.pass_tolerance(f, plan.target, p)))
            release = self.stable_ready(("pass", plan.created), ready)

            def fired():
                plan.phase, plan.sent, plan.origin = "confirm", f.cycle, f.ball
                self.ready_frames = 0

            self.stage = "pass/prepare"
            return {
                plan.passer: actions.pass_ball(f.ours[plan.passer], f, plan.target, p,
                                                release=release, on_fire=fired),
                plan.receiver: actions.receive_ball(f.ours[plan.receiver], f, plan.target,
                                                     f.ball, p),
            }
        if plan.phase == "confirm":
            if self.ball_departed(plan.passer, plan.origin):
                plan.phase, plan.departed = "receive", f.cycle
            elif self.elapsed(plan.sent) >= p.KICK_CONFIRM_TIMEOUT:
                self.cancel_pass()
                return None
        if plan.phase == "receive" and self.elapsed(plan.departed) >= p.RECEIVE_TIMEOUT:
            self.cancel_pass()
            return None
        self.stage = "pass/" + plan.phase
        receiving_frame = replace(f, airborne=True) if self.chip_settling() else f
        # 传球者停止且关闭吸球，避免追着球二次触碰；接球者保持实际身份。
        # 确认离脚前仍只准备；确认后才按落点/地滚球轨迹接球。
        return {
            plan.passer: Skill.Stop(),
            plan.receiver: actions.receive_ball(f.ours[plan.receiver], receiving_frame, plan.target,
                                                 plan.origin, p,
                                                 in_flight=plan.phase == "receive"),
        }

    def attack(self):
        """优先级：已出球的配合 → 射门 → 准备传球 → 护球；自由球内部回收。"""
        f = self.frame
        if self.pass_plan is not None and self.pass_plan.phase != "prepare":
            orders = self.run_pass()
            if orders is not None:
                return orders
        if self.shot_pending is not None:
            shooter, origin, sent = self.shot_pending
            departed = shooter not in f.ours or self.ball_departed(shooter, origin)
            if not departed and self.elapsed(sent) < p.KICK_CONFIRM_TIMEOUT:
                self.stage = "shoot/confirm"
                return {}  # 等真实离脚，不重复发射；G 仍由统一任务执行 Goalie。
            self.shot_pending = None
            self.retry_after = f.cycle + round(p.RETRY_DELAY * Params.frameRate)
        players = self.field_players()
        holder = self.holder if ball_control == 1 and self.holder in players else -1
        if holder < 0 or not geo.geometric_holder(f.ours[holder], f, p):
            if self.pass_plan is not None:
                orders = self.run_pass()
                if orders is not None:
                    return orders
            self.pass_plan = None
            self.ready_key, self.ready_frames = None, 0
            return self.get()
        target = self.choose_shot(holder)
        if target is not None and f.cycle >= self.retry_after:
            self.pass_plan = None
            return self.shoot(holder, target)
        self.shot_target = None
        if self.pass_plan is not None:
            if self.pass_plan.passer != holder:
                self.cancel_pass()
            else:
                orders = self.run_pass()
                if orders is not None:
                    return orders
        receivers = [n for n in players if n != holder]
        if receivers and f.cycle >= self.retry_after:
            receiver = receivers[0]
            target = self.pass_candidate(holder, receiver)
            if target is not None:
                self.pass_plan = PassPlan(holder, receiver, target, f.ball, f.cycle)
                return self.run_pass()
        return self.hold_and_support(holder)
