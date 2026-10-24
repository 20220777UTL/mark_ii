# NormalPlay 流畅性审查清单

审查范围：ExampleNormalPlay.py、NormalPlayActions.py、NormalPlayGeometry.py，
以及这些文件调用 Skill 时设置的参数。下面的剩余项尚未修改，按编号由用户决定。
数值以当前 SIMULATION=True 为准。这里只说明代码实际行为，不把每一项都认定为现场故障。

## 本轮已经删除

- 重复 cycle 直接 return，导致整轮任务不下发。
- cycle 跳号清零持球计数，以及出球准备要求 cycle 必须恰好加一。
- cycle 倒退触发整套策略重置；正常的 Play 切换重置仍保留。
- 球 Valid 为假就停止两名场上队员、冻结主状态、清除配合和准备计数。
- 持球辅助函数中 Ball.Valid/frame.valid 的前置拒绝条件，包括开球兼容入口。
- Frame 中已不再读取、保存球 Valid；没有新增丢帧宽限期或丢帧检测。

## 剩余项目：逐项决定

### 01 射门确认期间两名队员一起停

位置：ExampleNormalPlay.attack 的 shoot/confirm 分支。
发出射门指令后，未满足离脚条件时返回空任务，两个场上队员通过默认值执行 Stop，最长等 0.6 秒。
即使另一名队员可以跑位或争第二点，也没有任务。

### 02 传球后传球者一直停

位置：ExampleNormalPlay.run_pass 的 confirm/receive 分支。
传球者一直执行 Stop，直到接球完成或失败退出。接球者可以运动；传球者不参与二次跑位。

### 03 固定 0.4 秒冷却

位置：cancel_pass、attack、retry_after。
配合失败后，以及射门确认结束后，0.4 秒内不再射门或启动新配合。
冷却作用于整个进攻决策，即使出现新的射门机会也会等待；接球成功不加此冷却。

### 04 单次踢球指令和离脚双距离确认

位置：NormalPlayActions._aim_and_kick、ExampleNormalPlay.ball_departed/run_pass/attack。
只发一次踢球指令，然后等球离原点超过 160 mm，且离传球者超过 MOUTH_MAX+40=175 mm。
确认前不会补发踢球；接球队员仍执行准备动作，而非动态接球。0.6 秒未确认则取消/重评。
离脚确认这个思路在先前方案中讨论过；上述具体门槛及等待行为是实现细节。

### 05 持球/自由球需要多次确认

位置：update_ball_control、CONTROL_FRAMES、FREE_FRAMES。
持球需连续 3 次相同球权和车号，无人持球需 2 次。候选变化就从 1 重计。
本轮已删除 cycle 连续性要求，但这些确认次数仍保留，按调度调用累计。

### 06 出球就绪还要再累计 3 次

位置：stable_ready、READY_FRAMES。
射门/传球即使当次已满足位置、角度和速度要求，还要连续 3 次满足同一动作条件。
任一条件不成立或动作 key 改变就重计。本轮已删除 cycle 必须加一的要求。

### 07 持球必须满足相对速度上限，争抢优先判敌方

位置：NormalPlayGeometry.geometric_holder、utils.got_ball、update_ball_control。
球在车嘴范围内仍需球车相对速度不超过 350 mm/s；双方都满足时优先判对手持球。
这会影响 ball_control，也会影响开球脚本通过 utils.got_ball 判断接球完成。
车嘴几何范围本身属于用户要求的几何判断；这里列的是额外速度门槛和争抢偏置。

### 08 出球者必须降速到 250 mm/s 以下

位置：NormalPlayActions.kick_ready。
即使已经持球、方向正确且线路畅通，只要车速超过 250 mm/s 也不踢。

### 09 接应者必须先到位、转正、降速才传

位置：NormalPlayActions.receiver_ready、ExampleNormalPlay.run_pass。
同时要求距接球车中心目标不超过 100 mm、朝向误差不超过 15°、车速不超过 250 mm/s。
因此不能边跑到位边提前传球。稳定传接球是用户要求，但这组全部满足才出球的数值门槛可单独调整。

### 10 长距离出球自动收紧角度

位置：NormalPlayActions.pass_tolerance/shot_tolerance。
传球按最多 80 mm 横向误差收紧：5 m 时约 0.92°。
射门按门柱余量再扣球半径 21.5 mm 和额外 40 mm 收紧：5 m 角点射门约 0.78°。
原本的 6°/4°只是最大容差，实际可能小很多，容易等待对准。

### 11 挑传强制最短等待 0.35 秒

位置：chip_settling、update_ball_control、run_pass。
不论底层是否报告空中状态，都在发出挑球指令后 0.35 秒内排除接球队员的持球成功，
并给接球动作强制设置 airborne=True。

### 12 空中标志和落点过滤限制接球

位置：NormalPlayGeometry.geometric_holder/read_frame、NormalPlayActions.receive_target。
native ChipBallState 为真时不认定持球；优先等落点，不追球的地面投影。
预测点需在场内且有限，并且偏离原接触点不超过 900 mm、通过站位检查，否则继续等原定点。
如果预测明显改变，这个 900 mm 门槛可能阻止接球队员及时追上。

### 13 五个传球点的额外淘汰条件

位置：pass_candidate。以下可分项选择：

- 13a：传球距离必须在 600–5500 mm 内。
- 13b：接触点或接球车中心距任何对手不足 450 mm，直接淘汰。
- 13c：射线上近端 350 mm 内有对手，直接淘汰挑传点。
- 13d：接球队员距目标超过 5 秒 × 1000 mm/s = 5000 mm，直接淘汰。
- 13e：除传接双方外的己方车辆挡线，直接淘汰，即使设置为挑传。
- 13f：PASS_IS_CHIP=False 时，任何对手挡线都淘汰；当前全挑传模式不触发此项。

这些条件先于“路线上人数最少”的排名执行，可能把五个点全部排除。
场地/禁区及车中心合法性另见 14。

### 14 加大禁区/边界余量，找不到合法点就停或护球

位置：NormalPlayGeometry.legal/legalize、NormalPlayActions.move_to/hold_ball/get_ball/receive_ball/kick_ready/_aim_and_kick。
以车半径 90 + FIELD_MARGIN 45 = 135 mm 扩大禁区并内缩场地边界。
get_ball 也用这套车中心标准判断球位：球在这个扩大区域内，就不直接拿球，而是去修正后的边界点。
运动目标修正仍失败时执行 Stop；瞄准车中心不通过检查时退回护球；出球车当前中心不合法也不能踢。
真实场地/禁区约束与额外余量、失败就停止的处理应分开决定。

### 15 队友让路只尝试两个点

位置：clearance_position、choose_shot、shoot。
只试射线两侧两个点，并要求离射手至少 450 mm、离其他车至少 225 mm，还需通过边界/禁区检查。
两个点都不合格就放弃该射门目标；没有更多让路点搜索。队友没有让路目标时会 Stop。
“队友挡线先让开再射”是用户明确要求；这里待决定的是两点搜索和额外距离条件。

### 16 掉球几何立即打断准备

位置：run_pass、attack。
准备传球时，一次几何持球不满足便改为重新拿球并清出球就绪计数；球距传球者超过 280 mm 则取消配合。
Attack 也会在当前几何持球不满足时退出射门选择，转向内部抢球，即使球权累计结果仍为我方。

### 17 球员视觉失效即从候选剔除

位置：NormalPlayGeometry.read_frame、field_players、run_pass、utils.got_ball/arrive_at。
球员 Valid 为假，或位置/速度/朝向出现非有限值，则不加入快照。
缺失球员没有战术动作时会走默认 Stop；传接任一方不在快照中就取消当前配合。
本轮删除的是球有效性门控，这一组球员有效性门控尚未改动。

### 18 准备/确认/接球超时会取消配合

位置：run_pass；参数为 5 秒 / 0.6 秒 / 4 秒。
未在规定时间完成就取消并重评，失败会叠加 03 的冷却。
所有超时及冷却仍用 cycle/Params.frameRate（当前 75）计时；尚未改为时钟时间。

### 19 持球转向和运动加速度限制

位置：NormalPlayActions._aim_and_kick/hold_ball/move_to。
持球瞄准每次最多调整 8°；持球加速度 850 mm/s²，普通移动 1800 mm/s²。
这些数值可能让转身、射门准备和跑位变慢；它们是稳定控制的实现参数，不是用户指定的值。

### 20 接球近距离小步和退让

位置：NormalPlayActions.receive_target/receive_ball。
距球 650 mm 内，目标步长限制为 60 mm，加速度降到 650 mm/s²；近接触时顺来球退让，
速度取球速的 35%，上限 650 mm/s，提前窗口 0.2 秒；地滚球预测最多 0.8 秒。
接近球已满足几何持球时立即转为 hold_ball。用户要求避免把球撞飞，具体实现与数值可逐项调整。

### 21 支援位置刷新间隔和无点停止

位置：support。
支援目标缓存 0.6 秒；要求离持球者至少 450 mm、离敌方至少 450 mm。
无合格支援点时接应者直接 Stop。五点全不可用时“护球、调整支援、重评”本身已获用户暂时同意；
这里列出的是刷新间隔、额外距离和无点就停。

### 22 没有显式任务就一律 Stop

位置：_NormalState.getTasks 的 orders.get(number, Skill.Stop())。
空字典/遗漏车号不会保持原动作，而会立即停止。01 的双停、17 的缺车停都通过这里发生。
这是统一兜底行为，也应与具体状态的任务选择分开决定。

### 23 每次规划先关闭吸球

位置：planTasks 中 setDribbleCommand(number, 0)，随后由持球/接球技能设为 3。
本轮已经没有“球无效所以不执行技能”的分支，但这种先清零再由技能开启的下发顺序仍在。
它是否形成实际吸球断续取决于底层是否只发送最终命令，不能仅凭这两行断定。

### 24 路径避障和力度异常处理

这两项与底层接口相关，单列出来，不混同为等待策略：

- 24a：NormalPlayActions.MOVE_FLAGS 总是开启 allow_dss、dodge_our_defense_box；
  move_to 默认 dodge_ball，接球准备也 dodge_ball。底层可能因此绕行、减速或挡住接近目标。
  本次只审查这些传入参数，没有底层避障实现的运行证据。
- 24b：evalPower 返回非有限值或 <=0 时不踢、不记出球，只继续瞄准；当前没有额外错误提示。
  表现可能是一直对准不出球，但这是力度数据/接口问题，删除判断不能生成有效力度。

## 已明确要求或已同意的设计

- Attack/Defend/Get 转换表、两个场上队员、固定 Goalie。
- 三个射门点、线路检查、己方挡线先让路。
- 五个半圆接应点、配合全局 PASS_IS_CHIP。
- 红外严格判断先注释；传球、接球、射门独立封装。
- 路线恶化重选仅写 TODO，没有执行逻辑。
- 五点不可用时暂时护球、调整支援、重评：用户已同意先这样实现，并要求后续复议。

车号范围、除零处理、NaN/Inf 数据检查等基础接口/数学约束也已检查。
它们的运行影响归入 14、17、24b，不另增加隐藏的等待状态。
