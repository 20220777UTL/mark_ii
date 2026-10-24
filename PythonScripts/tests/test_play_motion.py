"""加载实际 Python Play，运动/踢球/调试出口全部替换；不运行原生 State.run。"""
import contextlib
import importlib
import io
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'PythonScripts'))


class PlayMotionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # DLL/ini 使用工程根目录；只初始化模块，不启动视觉循环或车辆控制。
        previous = Path.cwd()
        try:
            os.chdir(ROOT)
            with contextlib.redirect_stdout(io.StringIO()):
                import InitAllModules
            cls.normal = importlib.import_module('Play.Normal.ExampleNormalPlay')
            cls.free = importlib.import_module('Play.RefPlay.ExampleFreeKick')
        finally:
            os.chdir(previous)

    def setUp(self):
        from Geometry import CGeoPoint, CVector
        self.point, self.vector = CGeoPoint, CVector
        self.geo, self.actions, self.cfg = self.normal.geo, self.normal.actions, self.normal.p
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.moves = []
        def movement(target, angle=0, **kwargs):
            self.moves.append((target, angle, kwargs))
            return lambda number: None, lambda number: target
        for name in ('RushTo', 'SmartGoTo'):
            self.stack.enter_context(patch.object(self.actions.Skill, name, side_effect=movement))
        self.stack.enter_context(patch.object(self.actions.Skill, 'Goalie',
            return_value=(lambda n: None, lambda n: CGeoPoint(0, 0))))
        self.static = self.stack.enter_context(patch.object(self.actions.Skill, 'StaticGetBallV4',
            side_effect=AssertionError('FreeKick must not use native re-alignment')))
        self.kicks = SimpleNamespace(setKick=Mock(), setChipKick=Mock())
        self.stack.enter_context(patch.object(self.actions, 'kickStatus', self.kicks))
        self.stack.enter_context(patch.object(self.free, 'debugEngine', Mock()))

    def robot(self, number, x, y, heading=0):
        return self.geo.Robot(number, self.point(x, y), heading, self.vector(900, 0))

    def frame(self, robot, ball, *, airborne=False):
        return self.geo.Frame(1, ball, self.vector(0, 0), {robot.number: robot}, {}, airborne,
                              landing=ball if airborne else None)

    def test_all_attack_movement_targets_stay_outside_penalty(self):
        front, side = self.geo.penalty_bounds(self.cfg)
        robot = self.robot(1, front - 200, 0)
        ball = self.point(front + 500, 0)
        frame = self.frame(robot, ball)
        calls = (
            lambda: self.actions.move_to(robot, ball, 0, self.cfg),
            lambda: self.actions.hold_ball(robot, frame, self.cfg),
            lambda: self.actions.pass_ball(robot, frame, self.point(4500, 0), self.cfg),
            lambda: self.actions.shoot_ball(robot, frame, self.point(4500, 0), self.cfg),
            lambda: self.actions.get_ball(robot, frame, self.cfg),
            lambda: self.actions.receive_ball(robot, frame, ball, robot.pos, self.cfg, in_flight=True),
        )
        for call in calls:
            call()
            target = self.moves[-1][0]
            self.assertFalse(self.geo.PenaltyRouting.inside((target.x(), target.y()), front, side))
            self.assertFalse(self.geo.penalty_crosses(robot.pos, target, self.cfg))
        frame.airborne, frame.landing = True, ball
        self.actions.get_ball(robot, frame, self.cfg)
        self.assertFalse(self.geo.penalty_crosses(robot.pos, self.moves[-1][0], self.cfg))
        self.assertEqual(frame.ball.x(), ball.x())

    def test_corner_and_inward_terminal_velocity_are_zeroed(self):
        front, side = self.geo.penalty_bounds(self.cfg)
        robot = self.robot(1, front + 300, side + 300)
        self.actions.dribble_to(robot, self.point(front + 300, -side - 300), 0, self.cfg,
                                velocity=self.vector(0, -500))
        self.assertEqual(self.moves[-1][2]['vel'].mod(), 0)
        self.actions.dribble_to(robot, self.point(front, 0), 0, self.cfg, velocity=self.vector(500, 0))
        self.assertEqual(self.moves[-1][2]['vel'].mod(), 0)

    def test_shoot_keeps_original_goal_and_power_target(self):
        robot = self.robot(1, 0, 0)
        frame = self.frame(robot, self.point(self.cfg.MOUTH_OFFSET, 0))
        goal = self.point(4500, 0)
        with patch.object(self.actions, 'evalPower', return_value=2345) as power:
            execute, _ = self.actions.shoot_ball(robot, frame, goal, self.cfg)
            execute(1)
        self.assertEqual(power.call_args.args[1], goal)
        self.kicks.setKick.assert_called_once_with(1, 2345)

    def setup_free(self, *, permitted=True, heading=0, x=None):
        state = self.free.FreeKickState()
        state.passer, state.receiver = 1, 2
        state.receive_pos = self.point(1500, 0)
        robot = self.robot(1, -self.cfg.MOUTH_OFFSET if x is None else x, 0, heading)
        frame = self.frame(robot, self.point(0, 0))
        # 接应者尚远且运动中；不应阻塞满足车嘴/角度的出球者。
        frame.ours[2] = self.robot(2, 500, 800)
        self.stack.enter_context(patch.object(self.geo, 'read_frame', return_value=frame))
        self.stack.enter_context(patch.object(self.free.Ball, 'valid', return_value=True))
        self.stack.enter_context(patch.object(self.free.Conditions, 'isGameOn', return_value=False))
        self.stack.enter_context(patch.object(self.free.Conditions, 'isNormalStart', return_value=permitted))
        self.stack.enter_context(patch.object(self.free, 'getChipPower_DirectKick', return_value=1000))
        return state, frame

    def test_free_kick_fires_immediately_without_speed_or_receiver_wait(self):
        state, _ = self.setup_free()
        tasks = state.getTasks()
        self.assertEqual(state.stage, 'pass')
        tasks['A'].skill_cpp(1)
        self.kicks.setChipKick.assert_called_once_with(1, 1200)
        self.assertIsNotNone(state.kick_origin)
        self.assertEqual(tasks['B'].fixedNumber, 2)

    def test_free_kick_referee_permission_still_required(self):
        state, _ = self.setup_free(permitted=False)
        state.stage = 'pass'
        state.getTasks()['A'].skill_cpp(1)
        self.kicks.setChipKick.assert_not_called()

    def test_holder_on_wrong_side_still_routes_behind_ball(self):
        import math
        state, frame = self.setup_free(x=self.cfg.MOUTH_OFFSET, heading=math.pi)
        state.getTasks()
        self.assertEqual(state.stage, 'approach')
        self.kicks.setChipKick.assert_not_called()

    def test_free_kick_after_departure_runs_passer_and_receives(self):
        state, frame = self.setup_free()
        state.stage = 'receive'
        state.kick_origin = self.point(0, 0)
        frame.ball = self.point(700, 0)
        frame.velocity = self.vector(1000, 0)
        tasks = state.getTasks()
        self.assertGreater(tasks['A'].matchPos(1).dist(frame.ours[1].pos), 100)
        self.assertEqual(set(tasks), {'A', 'B', 'G'})
        self.kicks.setChipKick.assert_not_called()

    def test_free_kick_uses_two_degree_threshold_without_confirmation_wait(self):
        state, frame = self.setup_free(heading=0.06)
        state.stage = 'pass'
        state.getTasks()['A'].skill_cpp(1)
        self.kicks.setChipKick.assert_not_called()
        frame.ours[1].heading = 0
        state.getTasks()['A'].skill_cpp(1)
        self.kicks.setChipKick.assert_called_once()

    def test_free_kick_back_point_and_direction_share_target(self):
        import math
        state, frame = self.setup_free(x=-250)
        state.receive_pos = self.point(1500, 1500)
        frame.ours[1].pos = self.point(-250 / math.sqrt(2), -250 / math.sqrt(2))
        state.getTasks()
        self.assertEqual(state.stage, 'pass')
        center, facing, _ = self.moves[0]
        self.assertAlmostEqual(facing, math.pi / 4)
        self.assertAlmostEqual(center.x(), center.y())

    def test_free_kick_live_ball_exits_from_any_stage(self):
        state, _ = self.setup_free()
        with patch.object(self.free.Conditions, 'isGameOn', return_value=True):
            for stage in ('approach', 'pass', 'receive'):
                state.stage = stage
                self.assertEqual(state.transFunction(), 'exit')


if __name__ == '__main__':
    unittest.main()
