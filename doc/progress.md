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

