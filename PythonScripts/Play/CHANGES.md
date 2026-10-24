# Play_marcus_zz（2026-10-24）

复制来源：同级 Play_new。原目录未修改。可整体部署为 PythonScripts/Play，也可在配置中导入 Play_marcus_zz；内部 Play 引用已改为相对导入，避免加载原版本的参数/函数。

## 持球确认（保持原逻辑）

将球相对车中心的位移投影到机器人朝向：forward = dx*cos(h)+dy*sin(h)，lateral = -dx*sin(h)+dy*cos(h)。70 <= forward <= 135 mm 且 abs(lateral) <= 38 mm 时，进入几何持球候选。双方多个候选同时成立时，选择车中心距球最近的候选。同一 (team, number) 连续达到 CONTROL_FRAMES 次策略更新后确认持球；当前 SIMULATION=True，阈值为 3，切换为非仿真参数则为 5。无人满足条件连续 2 次（非仿真 3 次）确认自由球。更新次数没有按视觉 cycle 去重。确认前保留上一球权。

不检查红外、球速、球是否在空中或持球车是否停稳。接球车被确认为持球者后，run_pass 清除当前传球计划并当帧再次调用 attack。出球时的 kick_ready 则检查当帧车嘴几何和朝向误差，不再单独等待 3 次确认。

## 本次修改

1. KICK_ANGLE 改为 5°，调试消息从参数读取角度。开球同样引用该参数。
2. choose_shot 的我方挡线排除/排序、shoot_ball 的我方挡线出球门槛均删除；保留对手挡线判断。队友仍跑让路点，但不再等待其实际离开射线。
3. 射门同期：让路场上车带 dodge_ball，门将仅在 shoot 调度帧传入 dodge_ball，其它阶段恢复原门将标志。其它支援/跑位维持现有避球。准备接球、confirm、receive 三阶段以及任意球接应车均不带 dodge_ball。标志每帧由任务生成，不写全局常驻避球标志。
4. KICK_CONTACT_SPEED=600.0 mm/s。常规传球、射门的 RushTo 传入 vel = 球速向量 + 600 * 出球方向单位向量；这让命令中的车球相对速度为沿出球方向 600 mm/s。vel 是 CVector，不能用标量 600。保持 needDribble=True；底层速度/加速度限制和接触动力学决定实际速度，不能据此保证实际吸球效果。
5. 开球取球阶段及 JustKick 替代动作也使用该目标速度。原 JustKick 封装没有 vel 参数，因此在本副本开球模块内使用 RushTo + setKick 实现，不修改共享 Skill.py。
6. 保持尚未接触球时的 RushTo 接球缓冲；几何进入车嘴后，先执行下述 SimpleGoTo 跨帧贴球，再回到原来的保持/转向/出球动作。

## 接球后 SimpleGoTo 贴球（追加）

按用户确认保留当前 RushTo + setKick/setChipKick 出球方式；未启用的 NormalShoot 历史脚本不改。SimpleGoTo 的公开接口没有 vel 参数，因此通过跨帧位置目标让车产生位移，不能保证指定的实际速度。

- 每辆场上车独立保存 ContactPreparation，接球、取球、保持、传球和射门共享同一辆车的贴球进度。
- 球进入车嘴几何范围后锁定当时车头方向。横向偏差超过 8 mm 时，SimpleGoTo 横移对中，保持纵向位置和朝向。
- 横向偏差不超过 8 mm、朝向误差不超过 5° 后，进入前移阶段；持续开启吸球，不加 dodge_ball。
- 前移目标至少在当前车前 20 mm，较远时采用比名义接触位置前移 20 mm 的目标。这样球已经很近时也不会命令车辆后退。
- 必须在后续视觉 cycle 观测到沿锁定方向累计前移至少 10 mm，且球处于车前 70～105.5 mm、横向偏差不超过 8 mm，才交给原来的保持/转向/出球逻辑。前移累计从完成横向对中时开始。若横向失准则重新对中并重新计量。
- 准备阶段每帧只给该车下发 SimpleGoTo 和吸球命令，不下发踢球，也不触发 on_fire。原有出球 RushTo 的相对球速 600 mm/s、5° 朝向要求继续保留。
- 几何失球、该车发出踢球命令、Play 重入时重置进度；接球者被确认为持球者并重新选择进攻动作时，保留已完成的贴球进度。
- 调试行 MOUTH_Rn 后附 align / press moved=... / ready。无实际前移时不按超时放行；需要观察该信息判断是否被底层避障、运动死区等阻止移动。

此阶段验证的是实际前移和几何位置，不是红外吸球确认。球可随车一起运动；没有要求球相对车持续滑动。实车是否吸稳及转向后是否仍能出球尚需现场验证。

## 其他击球入口检查

- NormalPlayActions 的 pass_ball/shoot_ball：共享 _aim_and_kick，本次均已修改。
- ExampleKickOff 原 JustKick：内部 RushTo 未指定 vel，本次已补。
- ExampleFreeKick 的 StaticGetBallV4：Python/native 公开接口没有 vel；原生代码负责拿球、对准和出球。保留原技能，不能从 Python 封装确认其触球速度，未声称已改为 600 mm/s。
- Goalie 原生清球和历史 ExampleNormalPlayOld 中的 NormalShoot：同样不暴露触球速度参数；保留底层行为。历史 Old 文件未被当前配置加载。
- RefPlay 的 getDirectShootDir 辅助函数仍有我方挡线判断；当前 FreeKickState 不调用这个直射辅助函数。本次改变的是运行中的 NormalPlay 射门决策。

## 验证与实际限制

29 项离线测试通过，包含原整合回归、5° 边界、四方向及移动球的相对速度、敌我挡线区别、传球三个阶段的接球车标志、同期射门/下一帧门将标志、开球速度、任意球接球标志、持球边界和确认次数、副本包导入隔离。新增覆盖旋转坐标系下横移方向、同帧或静止禁止出球、近距离前移目标、贴球纵向距离、横向失准重置、丢球重置，以及接球到下一轮进攻的进度连续性。使用实际 State/StateMachine/Task/技能封装，原生视觉和执行接口为测试替身。

未进行实车或模拟器动力学测试。dodge_ball 是下发给底层的避球标志，不等于保证在高速射门前及时离开整条射线；取消我方挡线门槛后，仍可能打到尚未让开的队友。

本机测试：/Users/robinzheng/Documents/ChatGPT/SRC Match/marcus-zz-20261024/test_variant.py
