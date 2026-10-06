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

兵种优先级与装甲板朝向只在哨兵上生效。其他兵种只按偏离角跟随云台，避免考虑到其他因素后与操作手意图冲突。

- `src/kernel/tracker.cpp` 第 24 至 27 行：`Impl` 里放两张源表，加一张当前生效的表。
- `src/kernel/tracker.cpp` 第 108 行：新增成员 `autonomous_mode`，默认 `false`。
- `src/kernel/tracker.cpp` 第 180 至 183 行：`Impl::update_autonomous_mode` 同时切换该标志与生效表。
- `src/kernel/tracker.cpp` 第 427 行：朝向项的条件改为 `autonomous_mode && !armors.empty()`。
- `src/kernel/tracker.hpp` 第 62 至 65 行：新增 `update_autonomous_mode` 声明；`src/kernel/tracker.cpp` 第 586 行是对应的定义。
- `src/kernel/auto_aim.cpp` 第 244 至 253 行：把哨兵判断抽成 `is_autonomous`，同时传给 `update_aim_cleanup` 与 `update_autonomous_mode`。

## 4. 哨兵锁定后的换人判据

锁定后一直不换人这件事本身是有意的。`src/kernel/tracker.hpp` 里两个接口的注释写明了这条规格：自瞄意图开启时锁定该目标，即使它离开视野；超时清理关闭时，一旦锁定就一直瞄着它，直到自瞄意图关闭，同样是为了遵循操作手意愿。所以这里的改动同样只对哨兵生效。为此，我们加入一个 `autonomous_mode` 变量，作为是否为哨兵（自主决策兵种）的标记。

哨兵原来的超时清理只处理“目标从视野消失”这一种情况。当目标一直被识别到但一直无法命中时，时间记录每帧都在刷新，不会触发观测超时清理。我们要做的是另加一个火控超时清理。

### 4.1 目标解算失败后可能会卡住

每个跟踪中的目标带一个滤波器，程序靠它反推这台车的中心位置、朝向和转速。在刚看到目标时算不出初始状态，以及跟踪过程中估计值发散这两种情况下，程序会放弃识别目标。

这两条路径原来都删了滤波器，也删掉了“最后一次观测时间”，但没有清锁定标记。然而，超时清理需要遍历“最后一次观测时间”来判断超时，记录删掉之后超时清理会失效。

此时，若目标还在画面里，下一帧会重建滤波器，不会产生问题。但当目标离开画面时，哨兵会一直锁定在这个已经离开的目标，一直等到其再次出现才能继续；如果目标不再回到画面中，哨兵自瞄将失效。

解决方法比较简单，只删滤波器不删时间记录即可。

- `src/kernel/tracker.cpp`：滤波器初始化失败和发散两处，去掉 `robot_stamps.erase(id)`。

### 4.2 火控反馈无法命中

火控弹道解不出来时会返回一个空结果，意思是这一帧给不出可执行的瞄准，可以作为我们的判据。

同时，考虑到云台的机械结构，我们加入了 `yaw_min` 和 `yaw_max` 变量，表示云台最大可达范围。

- `src/kernel/fire_control.cpp`：`aim()` 在偏置校正之后判断瞄准角是否超出云台可达范围，超出按无解处理。
- `src/kernel/fire_control.hpp`：`Config` 加 `yaw_min` 与 `yaw_max`，构造时把度转成弧度。
- `config/executor.yaml`：加这两个键，默认 ±180 表示不限。
- `src/component.cpp`：有解和无解两个分支都写 `aim_solved`。
- `src/kernel/auto_aim.hpp` 与 `auto_aim.cpp`：addition 加这个字段，每帧转给跟踪器。

### 4.3 长时间无法命中时解除锁定

`src/kernel/tracker.cpp` 记录“最近一次可以命中的时间”。火控反馈无法命中时，将当前时刻与上一次可以命中的时间进行对比，超过 `fire_timeout_seconds` 就解除锁定。目标切换时重置起点。

这一段只在 `autonomous_mode` 为真时运行。有人操作的兵种不受影响。

- `src/kernel/tracker.hpp` 与 `tracker.cpp`：新增 `update_aim_solved` 入口、一个时间戳成员和计时判断。
- `config/config.yaml`：加 `fire_timeout_seconds`。

### 4.4 切换目标需要门槛

只加“一段时间无法命中即解锁”可能会带来新问题，当视野里出现两个分数接近且快速变动的目标，自瞄可能会不停来回切换，这对云台的要求很高。在此基础上加入「另一个候选明显更优」的条件，当两者的分数差小于设定门槛时，不切换目标。

在原先的代码中，自瞄锁定后其他候选原本被直接跳过。为了对比视野内多个候选目标的分数，我们需要删去这个跳过的逻辑。

判据单独放在一个头文件里，方便测试。

- `src/module/tracker/selection.hpp`：新增 `should_give_up_lock`。
- `src/kernel/tracker.cpp`：选目标时记录其他候选的最好分数；放弃判据挪到选目标之后，才能用本帧算出的分数。
- `config/config.yaml`：加 `switch_margin`，单位是度。

### 4.5 本次没有覆盖的一点

目标只露出一半装甲板时，自瞄判断不出来。朝向角只反映板面正不正，和遮挡无关。露出的一角能不能打中，取决于掩体的位置，自瞄的输入里没有这个信息。这一条归到问题 3。

## 5. 问题 3 和问题 4 的评估结论

这两个问题暂时放弃解决。

### 5.1 问题 3 ：出现概率极低，等到后续解决

注释里写到只看到单根灯条而非一整块装甲板的情况极少，考虑到工程复杂度，暂时放弃修改。

### 5.2 问题 4：需要线下调试对比

`src/utility/math/corners_optimizor.cpp` 的两个重载各有一个 `return;` 停用了函数。

修改此处的提交正文里对应的一行是 `* chore: turn off corners optimizor`，没有写症状，需要线下进一步评估。

这里的实现从逻辑上没找到明确错误。唯一可疑的是构造 PCA 点集时把亮度取整当重复次数那一步，量化很粗，但这一处不足以断定就是注释所指的问题。

结论是仓库里查不到问题是什么，要确定只能打开它和现状对比，这不在当前能做的范围内。

## 6. 锁定时切换目标的两处缺陷

这一节只改 `src/kernel/tracker.cpp` 选目标的那一段。

### 6.1 前哨站与大符不参与换目标比较

`other_score`（锁定时其他候选里的最好分数）原来只在遍历机器人模型的循环里更新。前哨站与大符各自被 `!locked || track_genre == ...` 挡在选择之外，也就进不了这份比较。后果是哨兵锁在机器人上时，画面里的前哨站再合适也换不过去；锁在前哨站上时，大符同样不参与比较。

改法是把「候选是否参与选择」与「候选是否要登记分数」拆开：新增 `register_candidate`，锁定时除锁定目标以外的候选只登记分数与目标本体，其余情况照旧参与选择。前哨站与大符两个分支去掉外层门，统一交给它判断，调试信息的填充条件与改动前保持一致。

### 6.2 解锁当帧不能立刻换人

解锁判定读的是选目标之前就算好的 `locked`，解锁那一帧 `if (!locked)` 不成立：`track_genre` 已经清空，但 `result` 仍然是旧目标的 trackable，新目标要等下一帧才写得进去。

登记候选时把目标本体一起留了下来（`other_device` 与 `other_result`）。`should_unlock` 成立的前提是存在更优候选，所以解锁分支里一定有目标可换：直接写 `track_genre` 与 `result`，并按「目标切换时重置起点」的约定重置火控解算计时（`fire_timeout_seconds` 的起点），火控在这一帧就能拿到新目标。

## 7. 图像边界过滤的边界位置错误

`src/kernel/tracker.cpp` 的 `Tracker::Impl::store(std::span<const Armor2d>)` 用相机内参主点乘二当图像宽高（`camera_matrix[0][2] * 2` 与 `[1][2] * 2`），配置里这组内参算出来是 1402×1129，而实际帧是 1440×1080（`rmcs_msgs::CameraFrame` 的约定尺寸，检测角点已经映射回原图）。

这道门要拦的是「被画面裁掉一部分的装甲板」：贴边的板角点不再是真实角点，PnP 会解出错误位置，传入给滤波器会把模型带偏。尺寸算错之后，上、左下右四边里只有上、左是对的：

- 右边：安全带实际宽 87 像素（本该 50），右侧 37 像素带内完整可见的板被误丢；
- 下边：安全带实际宽 0.8 像素（本该 50），等于没有保护，贴着下边缘的半块板会被送进滤波器。

改动：

- `src/utility/image/frame_bounds.hpp`：新增纯函数 `within_image_margin`，输入包围盒、图像宽高与边距；尺寸未知（小于等于 0）时不做判断并放行，避免异常路径上把观测全部丢掉。
- `src/kernel/tracker.hpp` 与 `src/kernel/tracker.cpp`：新增 `update_image_size`，`store` 改为「本帧尺寸加纯函数」，不再从相机内参推算。
- `src/kernel/auto_aim.cpp`：每帧在 `store` 之前用当前帧的真实尺寸调用 `update_image_size`。
- `config/config.yaml`：给 `image_margin` 补上说明。
- `test/image_margin.cpp`：新增 7 项测试，覆盖贴边、正好等于边距、坐标越界、尺寸未知，另加两条修复前后的对照（下边缘门槛 1030 对 1079.2，右边门槛 1390 对 1352.6）。

