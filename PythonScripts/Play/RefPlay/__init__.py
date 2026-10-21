"""3V3 定位球辅助接口及最小占位状态机；按子模块导入。

例如：from Play.RefPlay.ExampleKickOff import getKickOffPos, getAlignPos
四个子模块均提供无参数的 play_switch()，合计 29 个公开函数。
例如：from Play.RefPlay.ExampleKickOffDefend import play_switch
在状态的 transFunction 中返回 play_switch()。
默认判断比赛是否进入进行中；FreeKickDefend 还保留球移动且非 GameStop 的退出条件。
选手可在自己的 transFunction 中增加踢球检测、球速或超时等结束条件。
只需理解退出条件，无需使用原策略的状态名称。
返回 "exit" 只标记定位球流程完成，是否进入常规比赛仍由外层裁判调度决定。
这不是开球许可判断；我方开球不能等待 isGameOn() 成立才第一次踢球。
四个子模块各提供同名状态机类，例如：
    from Play.RefPlay.ExampleKickOff import ExampleKickOff
占位状态机的 getTasks() 默认留空，选手在对应状态中填写具体任务。
本包不统一重新导出这些类，也不自动创建任务或决策器。
"""
