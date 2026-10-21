"""测试模式策略占位。使用 main.py --test 启用；技能返回值需包装为 Task。"""

from RoleMatch_LuaStyle import State, StateMachine, Task, declare_state_machine
from RoleMatch_LuaStyle.Skills import Skill
from Vision import Ball, Enemy, Player
from WorldModel import Flags, Conditions, Params, Positions


class TestState(State):
    def getTasks(self):
        # 在此填写要单独测试的机器人任务。

        return {"G": Task(Skill.Goalie())}

    def getMatchString(self):
        return "[G]"

    def transFunction(self):
        return "TestState"


@declare_state_machine(TestState)
class ExampleTest(StateMachine):
    pass
