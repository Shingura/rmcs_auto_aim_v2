# 自瞄改进改动记录

## 1. 评分从距离改成角度

评分函数原来算“目标中心到相机光轴的垂直距离”，改成算角度。

- `src/utility/math/camera.cpp` 第 56 至 67 行：新增 `compute_angle2cam_x`。照着上面的 `compute_distance2cam_x`（第 44 至 54 行）写，末行改成 `atan2(侧向偏移, 前方距离)`，返回弧度。
- `src/utility/math/camera.hpp` 第 31 行：新增该函数的声明。
- `src/kernel/tracker.cpp` 第 380 至 382 行：评分函数换成调用新函数。

后续引入其他判断标准，因此将 calcutale() 返回的变量名改为 score。

## 2. 评分由一项变三项

原来只按偏离角排序。现在改成三项相加：偏离角 + 兵种优先级 + 装甲板朝向，分数越小越优先。

- `src/utility/robot/priority.hpp` 第 9 行：`PriorityMode` 定为 `std::unordered_map<DeviceId, double>`。
- `src/kernel/tracker.cpp` 第 29 至 45 行：`Impl::Config` 新增 `priority_autonomous` 与 `priority_teleoperated` 两个 `std::map<std::string, double>`，并在 metas 里登记这两个键。
- `src/kernel/tracker.cpp` 第 139 至 153 行：构造函数里加 `fill`，把配置中的字符串名转成 `DeviceId` 写进优先级表，两张表各跑一遍；默认 `priority_table` 取 `priority_teleoperated`。
- `src/kernel/tracker.cpp` 第 414 至 447 行：`calculate` 的代表点参数更名 `center`，新增第三个参数 `armors`（该设备的可打点）。朝向项的做法是取每块板的 `orientation` 四元数作用在 X 轴上再取反得到板面外法向，与「板心 → 相机」的单位向量做夹角，所有板取最小值。返回 `deviation + priority + facing`。
- `src/kernel/tracker.cpp` 第 459、480、540 行：三个调用点分别传前哨站的 `full()`、不传（能量机关的五片符叶共面，没有法向差别）、机器人的 `model.full()`。
- `src/utility/math/camera.cpp` 第 69 至 81 行：新增 `compute_armor_facing`，把朝向夹角的计算从 `calculate` 里抽出来，便于单独测试。
- `config/config.yaml` 第 52 至 64 行：priority 拆成 `priority_autonomous`（哨兵，HERO 0.0 至 RUNE 1.0）与 `priority_teleoperated`（留空）。

## 3. 按本车兵种分成两种形态

兵种优先级与装甲板朝向只在哨兵上生效。其他兵种有操作手，程序替操作手挑目标会和操作手抢准星，所以这两项在那些车上不参与打分，只按偏离角跟随云台。

- `src/kernel/tracker.cpp` 第 24 至 27 行：`Impl` 里放两张源表，加一张当前生效的表。
- `src/kernel/tracker.cpp` 第 108 行：新增成员 `autonomous_mode`，默认 `false`。
- `src/kernel/tracker.cpp` 第 180 至 183 行：`Impl::update_autonomous_mode` 同时切换该标志与生效表。
- `src/kernel/tracker.cpp` 第 427 行：朝向项的条件改为 `autonomous_mode && !armors.empty()`。
- `src/kernel/tracker.hpp` 第 62 至 65 行：新增 `update_autonomous_mode` 声明；`src/kernel/tracker.cpp` 第 586 行是对应的定义。
- `src/kernel/auto_aim.cpp` 第 244 至 253 行：把哨兵判断抽成 `is_autonomous`，同时传给 `update_aim_cleanup` 与 `update_autonomous_mode`。

## 4. 哨兵锁定后的换人判据

锁定后一直不换人这件事本身是有意的。`src/kernel/tracker.hpp` 里两个接口的注释写明了这条规格：自瞄意图开启时锁定该目标，即使它离开视野；超时清理关闭时，一旦锁定就一直瞄着它，直到自瞄意图关闭。所以这次只给哨兵补判据，有人操作的车保持原样。

哨兵没有操作手，没人替它决定换人。原来的超时清理只管“目标从视野消失”这一种。下面几小节处理其余几种打不到的情况。

### 4.1 一个漏写，目标解算失败后程序会卡住

每个跟踪中的目标带一个滤波器，程序靠它反推这台车的中心位置、朝向和转速。两条路径会把这个滤波器删掉：刚看到目标时算不出初始状态，以及跟踪过程中估计值越算越离谱。

这两条路径原来都删了滤波器，也删掉了“最后一次观测时间”，但没有清锁定标记。

超时清理靠遍历“最后一次观测时间”这张表判断超时。记录删掉之后遍历不到它，锁定标记就再也解不开。目标还在画面里时，下一帧会重建滤波器，记录跟着刷新，这条路能走通。目标彻底离开画面时就走不通了：程序锁着一个不存在的目标，每帧都跳过其他候选，算出来的结果一直是空的。哨兵没有松开自瞄这个动作，只能重启。

同一个函数里另外三处超时处理都写了“锁定的就是它就解锁”，作者知道这一步该做，这两处漏了。

修法是只删滤波器，不删时间记录。

- `src/kernel/tracker.cpp`：滤波器初始化失败和发散两处，去掉 `robot_stamps.erase(id)`。

### 4.2 让火控说出“打不到”

目标一直被识别到、但一直打不到时，时间记录每帧都在刷新，超时不会触发。这类情况要另加判据。

火控那边本来就有结论。弹道解不出来时它返回一个空结果，意思是这一帧给不出可执行的瞄准。它不负责换目标，那是 `tracker.cpp` 里的事，所以要把这个结论传过去。

传递方式上有一处要改。component 那边是“有结果才回填”，算不出解时整段跳过，addition 里的字段会停在上一次的值。

- `src/kernel/fire_control.cpp`：`aim()` 在偏置校正之后判断瞄准角是否超出云台可达范围，超出按无解处理。
- `src/kernel/fire_control.hpp`：`Config` 加 `yaw_min` 与 `yaw_max`，构造时把度转成弧度。
- `config/executor.yaml`：加这两个键，默认 ±180 表示不限。
- `src/component.cpp`：有解和无解两个分支都写 `aim_solved`。
- `src/kernel/auto_aim.hpp` 与 `auto_aim.cpp`：addition 加这个字段，每帧转给跟踪器。

### 4.3 打不到持续够久就放弃

`src/kernel/tracker.cpp` 记录“最近一次打得到的时间”。火控说打不到时，拿当前时刻和它比，超过 `fire_timeout_seconds` 就清掉锁定标记。目标切换到别的候选时重置起点，否则新目标会继承上一个目标的旧时间，第一帧就被判超时。

这一段只在 `autonomous_mode` 为真时运行。有人操作的车不受影响。

- `src/kernel/tracker.hpp` 与 `tracker.cpp`：新增 `update_aim_solved` 入口、一个时间戳成员和计时判断。
- `config/config.yaml`：加 `fire_timeout_seconds`。

### 4.4 换人要有分数差门槛

只加“打不到就放弃”会带来新问题。两个分数接近的目标会来回切换：放弃一个、选另一个、那个也打不到、再换回来。所以放弃要同时满足两件事：打不到够久，并且另一个候选明显更优。分数越小越好，所以条件是另一个候选的分数加上余量之后，仍然小于当前目标的分数。

这里有个前提要补上。锁定时其他候选原本被直接跳过，程序拿不到它们的分数，所以要让它们在锁定时也参与算分，只是不参与选择。

判据单独放在一个头文件里，方便测试。

- `src/module/tracker/selection.hpp`：新增 `should_give_up_lock`。
- `src/kernel/tracker.cpp`：选目标时记录其他候选的最好分数；放弃判据挪到选目标之后，才能用本帧算出的分数。
- `config/config.yaml`：加 `switch_margin`，单位是度。

### 4.5 本次没有覆盖的一点

目标只露出一半装甲板时，自瞄判断不出来。朝向角只反映板面正不正，和遮挡无关。露出的一角能不能打中，取决于掩体的位置，自瞄的输入里没有这个信息。这一条归到问题 3。

## 5. 问题 3 和问题 4 的评估结论

这两条都评估过。两条都没有动代码。

### 5.1 问题 3：作者判断出现极少，暂时放

现象是目标被掩体挡住、只露一根灯条时，识别层配不出装甲板，这台车从识别结果里消失。要等它整块板露出来才能重新找到。

`src/kernel/detector.cpp` 第 194 行到第 199 行原本有一段补救。它以一块已知装甲板为参照，在它左右各往外一个板宽的位置找灯条，通过长度和角度筛选后作为额外观测输出。这段代码被注释掉了，注释在第 188 行到第 192 行。

读代码之后看到两点。

第一点，这段逻辑帮不到最关键的情况。它的前提是 `armors.size() == 1`，也就是这台车已经识别到一整块装甲板。一块板都认不出来的时候，它没有参照物。

第二点，作者的判断。注释里写的三个理由是：需要严格的门禁、此处信息不足无法有效筛选、只看到单根灯条而非一整块装甲板的情况极少。最后一条是数量判断，它决定这件事值不值得做。用户对这条没有把握，暂时放。

### 5.2 问题 4：原因没有记录，做不了

`src/utility/math/corners_optimizor.cpp` 的两个重载都停在第一行。第 134 行和第 192 行各有一个 `return;`。

查了提交历史。引入这两个 `return;` 和那句注释的是同一个提交 `aed9851`，2026 年 6 月 21 日。这次提交对该文件的改动只有加注释和加 `return;`，算法代码一行没改。提交正文里对应的一行是 `* chore: turn off corners optimizor`，归类在 chore。这一栏通常放杂项，说明作者当时没有认定它是缺陷，提交里也没有写症状。

逐行读过实现，从逻辑上没找到明确错误。唯一可疑的是构造 PCA 点集时把亮度取整当重复次数那一步，量化很粗，但这一处不足以断定就是注释所指的问题。

结论是仓库里查不到问题是什么，作者当时只是把它关掉。要确定只能打开它和现状对比，这不在当前能做的范围内。

