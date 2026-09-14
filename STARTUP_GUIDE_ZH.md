# GO2 在线自主探索启动手册

**本次急停修复已部署：**启动命令不变，下一次启动生效。规划器要求停车时立即输出零速度，RViz 新增“规划停车请求 / 自动恢复准入”，显示具体等待条件；急停现场数据单独写入本轮状态日志。验证和现场分析见 [STOP_TIMING_FIX_ZH.md](STOP_TIMING_FIX_ZH.md)。

**选点效率更新：**启动命令不变，RViz 增加“探索效率 / 等待原因”，显示路径长度、预计观测收益、等待时间和阻塞坐标。保留 0.30 m/s、0.50 rad/s 上限；五级台阶的自主通行尚未适配。详见 [EFFICIENCY_CHANGES_ZH.md](EFFICIENCY_CHANGES_ZH.md)。

**当前默认：平地探索模式。**地面几何评分不再导致停车；仍检查避障点云、地图、TF 和指令是否有效。RViz 的“避障感知”显示准入状态，“地面支撑（仅显示）”不参与停车判定。启动命令不变，需要对照旧版时添加 `--terrain-geometry-checks`。本轮变更与验证见 [FLAT_GROUND_CHANGES_ZH.md](FLAT_GROUND_CHANGES_ZH.md)；下文历史验收中“拟合高度、地形支撑必须通过”的要求仅适用于启用几何检查的模式。

适用机器：`192.168.50.110`，用户 `nvidia`。工作空间：`/home/nvidia/go2_explore_ws`。更新日期：2026-09-14。

本系统直接边建图边探索，不需要先加载地图或执行重定位。统一使用 `run_go2_explore`，它自动准备独立 ROS 环境和端口。原 `/home/nvidia/go2_nav_ws` 及原 `run_go2` 保持原样。

**当前安装与验收状态：**探索默认开启可恢复故障后的自动重新使能，具体条件见第 4 节真机入口说明；126 项安全测试、62 项探索测试及六组 ROS/mock 联调通过，实体自动恢复待现场验证，最新记录见 `artifacts/auto_reenable_20260914/DIAGNOSIS.md`。相机已重新安装，RViz 默认显示视频，球形滤波默认 0.70 m。此前短测记录约 31 厘米里程计位移和足端运动反馈；里程计跳变和雷达连续性故障仍保持锁止，不自动清除。

**磁盘使用：**已清理两份被后续测试替代的完整录包和一份无效中间包，释放约 7.87 GiB，清理后可用约 8.5 GiB。日常运行省略 `--record`；完整录包在现场测试中可接近三分钟 5 GB，使用前检查 `df -h /home/nvidia`。地图保存不依赖该参数。

2026-09-14 已修复探索入口遗漏地面点云输出开关的问题。修复前已经启动的会话须先正常停止，再以新地图名重开；无需重新编译。详细原因见 `artifacts/ground_output_fix_20260914/DIAGNOSIS.md`。

2026-09-14 最新更新：启动默认打开 RViz 和中文状态面板；已核对真实 TF/点云转换、修复机身轨迹显示，并增加使能后姿态稳定等待及目标结束后的减速尾段处理。正常站姿静态观测的雷达离地约 0.48 m，地形、地图和覆盖数据有效，PCD 保存成功；早期 0.25 m 的记录属于历史低姿态检查。实际迈步、真实接管、全屋覆盖及物理停车仍待现场验证。最新证据见 `artifacts/rviz_health_20260914/DIAGNOSIS.md`，初次部署记录见 `artifacts/ACCEPTANCE.md`。

## 1. 连接并准备

在电脑终端连接机器狗，按提示输入 SSH 密码：

```bash
ssh nvidia@192.168.50.110
```

以下所有 `run_go2_explore` 命令都在机器狗终端执行：

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore status
```

准备工作：

1. 用原系统的正常方式结束原建图或导航任务，等其保存退出。两套入口共用互斥锁，不能同时运行。
2. 由现场人员使用原遥控器让机器狗保持正常站姿，雷达无遮挡，启动初始化期间保持静止。不要通过修改高度门限来适配趴伏姿态。
3. 真机行走测试时，现场人员持遥控器并确认可接管，周围留出测试空间。原控制器使能要求电量至少 25%。
4. 每次使用新的地图名，只允许英文字母、数字、下划线和短横线，且首字符为字母或数字。例如 `room_check_20260914_01`；不要使用中文、空格或斜杠。

无需手动执行 `roscore`、`source` 或 `run_go2 enable`。探索入口自动加载系统 ROS 与新工作空间，ROS Master 默认端口为 **11321**。

## 2. 先做静态检查

终端 A：

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore observe room_check_20260914_01 --record
```

此模式使用真实雷达、FAST-LIO、地形感知和地图积累，底盘使用 mock，仅预览候选目标和查询规划路径，不会自动使能真实底盘。`--record` 同时保存传感器与探索状态录包。

保持终端 A 开启。另开终端 B，再次 SSH 登录机器狗后查看状态：

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore status
```

正常初始化完成后应检查：

| 状态字段 | 静态检查期望 |
|---|---|
| `real_sdk` | `false` |
| `auto_start` | `false` |
| `/exploration/odom_status` | `healthy` |
| `/terrain/healthy` | `true` |
| `/exploration/map_status` | 以 `ready:` 开头，并持续获得新观测 |
| `/exploration/coverage_status` | 以 `ready:` 开头 |
| `/exploration/frontier_status` | 有任务时为 `AVAILABLE`；`INVALID` 表示数据条件不满足 |
| `/go2/control/enabled` | `false` |
| `/exploration/state` | `WAITING_FOR_ARM` 是 observe 模式的正常状态 |

连续观察至少 3 分钟，结合 RViz 确认地图更新、候选目标和全局路径。若一直出现 `INVALID`、高度过低或点云/TF 过期，先处理对应原因，不进入真机行走步骤。静态环境中没有可用路径不能当作探索完成。

检查结束后，在终端 B 执行：

```bash
./run_go2_explore stop
```

等终端 A 完全退出，再导出这次静态地图：

```bash
./run_go2_explore export-map room_check_20260914_01
```

PCD 保存成功与 PGM/2.5D 导出成功是两项独立结果。导出仍可能因有效地面样本不足而失败，以终端结果和导出日志为准。

## 3. 自动探索连接模拟底盘

静态感知检查通过后，可以在新会话中检查自动启动与规划逻辑：

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore explore room_mock_20260914_01 --record
```

此模式仍使用真实传感器；自动选择目标、规划并把速度交给 mock bridge。**它不会虚拟推动真实里程计，也不会让实体机器狗行走。** 因此，机器狗静止时长期无法到达目标、等待或重新选点是可能的，不能据此验证全屋覆盖。

使用第二个终端的 `status` 查看状态，结束时执行 `stop`。三种模式共用同一入口和锁，每次只启动一种。

## 4. 现场真机自动探索

完成正常站姿静态检查后，在现场监督下使用新的地图名：

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore explore room_real_20260914_01 --real --record
```

**此命令在输入就绪后自动使能并探索；当前版本还默认开启可恢复故障后的自动重新使能。**

故障停车后保留本次在线地图和探索进度，取消旧目标及旧速度。地形、地图、TF/点云、规划指令等暂时异常恢复后，需确认 SDK 已禁用、目标已取消、真实零速度心跳及连续停稳，且当前完整轮廓和前方停车范围安全，再连续 10 秒确认全部数据正常并新增至少两次有效地图更新。随后自动重新使能，按原姿态准备流程准入，再根据当前地图生成新目标，继续本次探索。

遥控接管、人工停止、急停/姿态切换、无迈步响应、SDK 拒绝使能、节点退出不会自动恢复。里程计跳变、时间倒退或雷达连续性锁止不能只靠清除安全门恢复；应处理原因后重开会话。真实 SDK 遥测、电量、模式及原桥接器健康诊断也必须通过恢复检查。两次自动恢复至少间隔 30 秒，5 分钟内最多尝试 3 次；等待原因显示在 RViz 的“故障自动恢复”一行。关闭此功能可添加 `--no-auto-reenable`，初次自动使能不受影响。

正常流程为：传感器和里程计初始化 → 地形、地图及规划器就绪 → 请求 SDK 使能 → 确认使能成功 → 等待至少 5 秒，再连续 3 秒确认姿态与感知稳定 → 安全门准入 → 开始选点与探索。准备期间速度输出为零；20 秒内无法稳定则禁用并报告失败，不自动重试使能。

探索上限为前进速度 **0.30 m/s**、转向速度 **0.50 rad/s**，禁止倒车和横移。SDK 网卡默认沿用 `eth0`，通常无需设置；如现场确实使用其他 SDK 网卡，可显式传入 `--interface 网卡名`。

真机启动后，用第二个终端观察：

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore status
```

| 探索状态 | 含义与操作 |
|---|---|
| `WAITING_FOR_READY` | 等待初始化或健康输入，查看地形及安全诊断中的具体原因 |
| `WAITING_FOR_POSTURE_STABILITY` | SDK 已使能，仍保持零速度，等待姿态与感知稳定 |
| `WAITING_FOR_FAULT_RECOVERY` | 故障停车，等待数据、停稳、范围检查及连续 10 秒稳定后自动恢复 |
| `AUTO_REENABLING` | 自动重新申请 SDK 使能；随后等待姿态稳定及新目标 |
| `WAITING_FOR_GOAL` | 等待可执行目标；有任务但无路径时也可能停留于此 |
| `EXPLORING` | 当前存在活动探索目标 |
| `WAITING_FOR_OBSTACLE` | 已停车并取消旧目标，检查旧方向清空或停稳后的可行恢复方向 |
| 安全门 `OBSTACLE_REPLAN_READY` | 已停稳且当前轮廓、前方范围检查通过；仍为零输出，等待新目标 |
| `RECOVERING_AFTER_OBSTACLE` | 恢复条件满足后的输入稳定确认，随后重新选点 |
| `PAUSED_WAITING_FOR_DATA` | 数据条件暂时不满足，查看过期或缺失输入 |
| `FAULT_STOPPED` | 自动恢复关闭、不可恢复故障、人工停止或接管等导致锁停；查看“故障自动恢复”原因 |
| `CONFIRMING_COMPLETE` | 正在确认持续没有合格探索任务，不等于已经完成 |
| `COMPLETED` | 满足完成条件，随后停车、保存并退出 |

普通障碍清空并稳定后可自动重新选点。人工停止、遥控接管或硬故障后，应结束当前会话、处理原因，再以新的地图名启动；不要用原 `run_go2 enable` 或手动调用 arm/reset 来绕过新监督器。

## 5. 停止与保存

### 正常停止

在第二个终端执行：

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore stop
```

也可在启动终端按一次 **Ctrl+C**。两种方式均先关闭运动输出、取消目标、禁用控制，再保存地图并清理本次会话。保存期间等待终端返回，不要用 Ctrl+Z 暂停，也不要直接关闭窗口代替停止。需要立即人工接管时，使用已熟悉的原遥控器接管方式。

正常结束的结果应包含：

```json
{
  "save_status": "saved",
  "snapshot_manifest": true,
  "forced_shutdown": false,
  "success": true
}
```

`success: true` 表示本次会话正常结束且保存校验通过，不代表已经覆盖全部房间；只有相应完成状态才能说明算法满足了结束条件。保存失败、异常退出或校验失败会明确报告。

### 运行中保存快照

```bash
./run_go2_explore save-map
```

该命令只保存当前 PCD/轨迹快照，探索会继续运行。需要结束运动和任务时应使用 `stop`。

### 退出后导出地图

```bash
./run_go2_explore export-map room_real_20260914_01
```

导出生成供后续使用的 PGM/YAML 和 2.5D 地形成果，不是在线探索的前置步骤。已存在的导出成果会拒绝覆盖。

## 6. 地图和日志位置

所有输出均在新工作空间：

```text
/home/nvidia/go2_explore_ws/
├── maps/<地图名>/
│   ├── public_map.pcd
│   ├── traversed_path_map.pcd
│   ├── mapping_snapshot.sha256
│   ├── map.pgm                  # 导出成功后
│   ├── map.yaml                 # 导出成功后
│   └── terrain_2p5d.yaml        # 导出成功后，另有配套地形资产
├── logs/<会话目录>/
│   ├── console.log
│   ├── session.json
│   ├── session_result.json
│   ├── status_events.jsonl      # 默认保存状态变化与首次地形失败原因
│   ├── sensors.bag              # 启动时带 --record
│   └── ros/
└── artifacts/ACCEPTANCE.md
```

启动时会打印本次日志目录；运行中的 `status` 也会显示 `session_dir`。日志名称使用机器狗自身时间，上次验收时其日期与开发机不一致，因此定位日志应以启动输出为准。

## 7. RViz 与调试选项

`observe` 和 `explore` 现在默认打开机器狗桌面的 RViz；SSH 启动时自动查找当前桌面显示授权：

```bash
./run_go2_explore observe room_visual_20260914_01
```

若图形会话不可用，入口在启动硬件前明确报错；需要无界面运行时显式加 `--no-rviz`。当前会话中意外关闭了 RViz，可在另一终端运行 `./run_go2_explore rviz` 重新打开，它只订阅状态，不使能控制。RViz 配置文件位于：

```text
/home/nvidia/go2_explore_ws/src/go2_exploration/config/go2_exploration.rviz
```

主要查看导航碰撞记忆、局部避障代价、GO2 轮廓、候选目标、全局路径和当前障碍点。诊断图层中的地面覆盖图可区分“可通行”和“实际已观测地面”；预测视野不会被写成已观测空闲。

右侧状态面板显示运行模式、停车原因、控制使能、TF/里程计、地形高度与支撑面积、地图和覆盖更新、选点原因、规划/整形/安全输出速度，以及真实 SDK 的电量、接管和步态响应。模拟底盘没有真实电池和足端遥测，对应字段显示缺失，不能当作真实反馈。默认记录紧凑状态日志；`--record` 才额外录完整 rosbag，录制前检查磁盘容量。

**D435i 视频：**默认随真实感知启动，在 RViz 左上方“D435i 视频”窗口显示 640×480、15 FPS 彩色画面。话题为 `/exploration_camera/color/image_raw`。该功能不使用 D435i 的 IMU，也不发布未经标定的相机外参；FAST-LIO 仍使用 MID360 的 IMU。相机不可用时可以添加 `--no-camera` 关闭视频，其他探索检查仍照常执行。模拟合成场景不启动相机。

**雷达断流：**右侧“雷达网络 / 数据连续性”显示 `eth1` 物理链路及雷达/IMU 时序。若出现 `FAULT:`，本次估计器停止接收新数据，运动准入关闭；重新插紧或更换雷达网线、检查供电并处理原因后，停止旧会话，换新地图名重新启动。不要通过重置地图坐标轴、放宽超时或手动使能来处理漂移。2026-09-14 现场已确认过每 2–3 秒的物理链路掉线，重新插紧后掉线计数停止增加。

**坐标系辨认：**固定坐标系保持 `map`。红 X 为机身前方，绿 Y 为左方，蓝 Z 为上方。`map → odom` 为恒等变换，零点是启动时机身中心，不是地面；正常站姿的地面约在 map z = −0.32 m。`lidar_link` 位于机身前方 0.187 m、上方 0.16 m，沿原外参俯倾 39°，其红轴不能当作水平机身前向。`lio_odom/body_lio` 是估计器内部坐标；不要把 RViz 固定系改成它们来调整画面。蓝色轨迹来自 `/odom_robot`，表示机身中心。

需要核对时展开“更多地图与点云”，勾选“雷达轴”“地图原点”和“完整 TF 树”。参考网格表示初始机身高度，不代表地面。右侧顶部“真实传感器 · 模拟底盘”和“全虚拟模拟场景”有明确区别；观察模式显示 `WAITING_FOR_ARM`、控制禁用和零速度是预期行为。

常用可选参数：

| 参数 | 用途 |
|---|---|
| `--record` | 保存本会话的传感器与控制状态录包 |
| `--rviz` | 打开探索 RViz，现为默认行为 |
| `--no-rviz` | 显式关闭界面，适用于无桌面 SSH 运行 |
| `--no-camera` | 关闭 D435i 彩色视频；默认开启 |
| `--lidar-blind 0.70` | 本会话雷达球形近距剔除范围，允许 0.30–0.70 m，默认 0.70；范围内的环境点也会被剔除。可显式用 0.30 恢复原范围 |
| `--real` | 仅用于 `explore`，选择真实 SDK；默认始终为 mock |
| `--no-auto-start` | 关闭自动开始，对应保留的 `auto_start:=false` 调试选项 |
| `--no-auto-reenable` | 关闭故障后的自动重新使能；仍保留初次自动开始 |
| `--interface eth0` | 指定 SDK 网卡，默认即为 eth0 |

调试示例：

```bash
./run_go2_explore explore room_debug_20260914_01 --no-auto-start
```

该会话不会自动使能和执行探索；结束调试后，重新用新的地图名正常启动即可。当前入口没有在运行中把该选项切换为自动开始的子命令。

## 8. 常见问题

| 现象 | 处理方式 |
|---|---|
| 提示原系统占用 `stack lock` | 回到原任务终端，正常停止并等待其保存退出；不要删除锁文件绕过互斥 |
| `An exploration session is already running` | 先执行 `status` 找到当前会话，再按需 `stop` |
| 提示重复驱动、FAST-LIO 或控制进程 | 结束相应旧任务；不要使用 killall/pkill 清理所有 ROS 进程 |
| `Refusing to overwrite existing map` | 换一个未使用的地图名 |
| `Refusing to overwrite exported map asset` | 该地图已有导出成果；本入口不提供强制覆盖选项 |
| 高度低于 0.43 m | 检查机器狗是否趴伏或站姿过低；若已经正常站立，保留诊断并检查现场感知，不改原外参或放宽保护 |
| 地形 false、覆盖 `INVALID` | 看 `status` 中 `/terrain/status` 的具体原因；检查站姿、地面观测和传感器更新 |
| 地形 healthy，但地图提示地面输入缺失 | 检查 `/terrain/ground_points` 是否有发布者；修复前启动的旧会话需要重启以启用地面输出 |
| observe 一直 `WAITING_FOR_ARM` | 正常，该模式只观察和预览；不要为了消除该状态手动使能 |
| 有任务但没有可执行路径 | 查看候选和代价图，等待新观测或处理通道障碍；不能判为完成 |
| 前方可通行，却一直拒绝目标 | 最新版已修复机身边缘采样与栅格中心判定不一致、仅选择边界目标以及最高观测收益朝向不可转入的问题。新会话会考虑 0.6–1.0 m 的前向接近位置，仍须通过实际地图、完整轮廓和到达朝向检查 |
| `FORWARD_KNOWN_ROUTE_CONNECTION` | 正在沿当前路线、已观测通道执行短距离前进连接；实际速度仍经过原整形器、安全门和 SDK。RViz 显示相应局部路径，下一控制周期继续尝试 TEB |
| 地面清空状态 `ready: support_cells=… clearing_samples=…` | 原 GO2 已确认地面格已转换到探索代价图的采样尺度。该话题 `/exploration/clearing_points` 仅用于新探索；原 `/terrain/clearing_points` 和原工程保持原样 |
| 使能时动一下，随后不迈步 | 用 `status` 检查 `/explore/selection_status`、`goal_generation` 和 SDK 的 `last_nonzero_command_age_sec`；只有使能成功、没有目标或非零速度时，不能据此判断步态故障 |
| `LOCAL_STOP_ENVELOPE_BLOCKED` 或 `obstacle_in_costmap_stop_region` | 对照局部代价图检查实际速度下的完整转动/停车范围。原整形器会把纯转向提高至 0.50 rad/s、有效前进提高至 0.30 m/s；新版规划预检查已计入这些门槛。未知区仍不可直接通行，不能把未知区改为空闲来解除阻挡 |
| 步态请求回执为 0，但机器人静止 | 回执只表示请求被接受。看 RViz 的“实际运动证据”：`last_nonzero_command_age_sec=-1` 表示 SDK 尚未收到非零指令；该情况下无法用足端静止判断步态是否失败 |
| `robot_tf_stale` / `lio_odometry_stale` / `odometry_time_reversed` | 保留当前日志，停止后用新地图名运行 `observe NAME --record` 检查数据链。`console.log` 的 `Odometry health transition` 与 `Odometry first continuity fault` 记录 TF/里程计时间戳、接收时间和故障来源；不要放宽超时或取消连续性保护来恢复行走 |
| 选点 `global_footprint` / `local_stop_envelope` | 候选路径碰到完整轮廓或前方停车范围内的障碍/未知栅格；本版已修复网格起点偏移导致的误拒绝，仍保留真实姿态起点和连续轮廓检查 |
| `FAULT_STOPPED` / `WAITING_FOR_FAULT_RECOVERY` | 查看“故障自动恢复”。暂时性输入故障正常后可自动恢复；坐标连续性故障、人工停止、接管、SDK/步态问题等仍保持停止 |
| 障碍停车后仍长时间等待 | RViz 查看“障碍恢复”。新增连续停稳检查及前方范围检查，旧转弯受阻时可换方向重新规划。新方向的障碍、未知格、停稳未确认或输入异常仍阻止恢复；全过程保持零输出，直到新目标及速度通过安全检查 |
| 录包提示 `Less than 1G of space free` | rosbag 已停止录制，不能把该包当作完整运行记录；先检查 `df -h` 并归档历史录包。`--record` 是可选项，不要降低磁盘保护阈值 |
| 导出提示地面样本不足 | PCD 保存与地形导出是独立结果；恢复正常站姿、补充有效采集后使用新地图复测 |
| `status` 显示 `running: false` | 当前没有由该入口管理的探索会话；不是对所有外部 ROS 任务的检查 |

2026-09-14 的整形速度预检查修复已经编译部署，结束旧会话、换新地图名启动即可生效。`--record` 现也记录 SDK 诊断、规划执行模式、状态面板和聚合日志，以便关联规划指令、安全门输出与实际足端响应。会话 `room_real_20260914_01` 中最初是转动范围被未知区阻挡，后来才出现 TF/点云中断，不能将两者混为同一个步态故障；详细证据见 `artifacts/live_20260912_190529_0e8097cb/DIAGNOSIS.md`。

后续工程复查已补齐栅格碰撞判定、前向接近目标、可达观测朝向和地面格清空适配。真实场景回归使用当前记录的地面/障碍回波复现问题，修改前没有有效目标或非零指令，修改后输出有效前进命令；详细范围及验收结果见 `artifacts/engineering_20260912_203137_ef59c413/DIAGNOSIS.md`。RViz 新增“地面网格清空”和“规划执行方式”。静态检查或模拟底盘模式下机器狗保持静止，只有显式 `explore NAME --real` 才连接真实 SDK。

## 9. 恢复原系统

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore stop
```

等待探索完全退出，然后按原来的方式使用 `run_go2`。无需修改 `.bashrc`、复制地图回原目录或重新安装 SDK。新工作空间已经编译完成，日常启动无需重新构建；只有修改源码后才执行本工作空间的 `./build_workspace.sh`。

## 10. 命令速查

| 目的 | 命令 |
|---|---|
| 静态检查 | `./run_go2_explore observe NAME --record` |
| 自动逻辑连接 mock | `./run_go2_explore explore NAME` |
| 现场真机自动探索 | `./run_go2_explore explore NAME --real --record` |
| 查看状态及等待原因 | `./run_go2_explore status` |
| 重开当前会话的 RViz | `./run_go2_explore rviz` |
| 运行中保存快照 | `./run_go2_explore save-map` |
| 停止、保存并退出 | `./run_go2_explore stop` 或启动终端 Ctrl+C |
| 停止后导出 | `./run_go2_explore export-map NAME` |

以上命令均在 `/home/nvidia/go2_explore_ws` 执行。将 `NAME` 替换为地图名，不要将表中的所有启动模式同时执行。

## 11. 相机重新安装与自身回波处理

相机可以继续安装并显示视频。三维自身过滤模块已部署，支持单独描述相机和支架，但当前 `camera_self_filter.yaml` 为关闭状态、过滤形状为空，尚未启用实际剔除。RViz 中的“相机与支架过滤”显示标定状态。`--no-camera` 仅关闭视频，不影响机械安装件的过滤配置。最新标定记录见 `artifacts/camera_install_4be378d3/DIAGNOSIS.md`。

已有球形滤波是 FAST-LIO 的 `preprocess/blind`，球心为雷达原始点云原点。根据用户在短测后的要求，独立探索入口及探索 launch 默认均改为 **0.70 m**，正常启动无需再加参数。原 FAST-LIO 配置副本仍保留 0.30 m，由探索 launch 覆盖；原导航工作空间不变。地形的水平排除距离仍为原 0.35 m。球形滤波也会剔除范围内的环境点，不是仅识别相机的过滤。

用户手持遥控器授权后，已用 `--lidar-blind 0.70` 完成一次短时真机试验：SDK 正常使能、生成 2 个目标、累计约 1.95 秒非零指令，里程计最大位移约 31 厘米，并出现足端卸载和关节运动反馈；达到预设距离后停车、禁用控制并保存。期间仍出现一次代价图障碍等待。该次测试结束时曾恢复默认 0.30 m，之后按用户要求将默认改为 0.70 m；此结果不代表近距离避障及完整覆盖验收通过。详见 `artifacts/blind070_real_53614c22/DIAGNOSIS.md`。

1. 保留已验证的雷达安装与外参，调整相机及支架，使其避开雷达实际扫描视野，并固定线缆。雷达已俯倾 39°，不能仅凭“相机在雷达下方”判断不遮挡。Livox 的 [MID360 安装指南](https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf) 要求安装时避免视场被物体遮挡；可参考其 [官方视场三维模型](https://www.livoxtech.com/mid-360/downloads)。
2. 在最终安装位置测量相机外壳、支架相对 `base_link` 的实际位置、旋转和尺寸。优先使安装件留在现有规划轮廓内；若伸出轮廓，需要单独核对探索碰撞几何，不能一边滤除它、一边忽略伸出的实体部分。
3. 用这些实体的三维包围盒或网格构建自身过滤，只排除经验证落在硬件本体及小幅测量余量内的回波。原始数据仍保留用于对照；供建图与地形感知使用的数据采用一致的过滤入口。遮挡后的区域保持未知，不因删除自身点而写为空闲或发出清空射线。过滤形状必须先在 RViz 对齐验证。
4. 用 `observe NEW_NAME` 静止检查安装前后同一场景的点云；再检查正常站姿与姿态变化。将真实障碍放在硬件边界之外，确认其仍进入避障检查，然后做模拟控制联调。滤除后的丢点比例、区域和状态应可观测；外壳区域之外的异常点仍单独分析，不能只按低反射强度或低出现频率删除。

此次全部移除相机、支架和线缆后，指定近处区域仍有 4.17% 的地形帧出现障碍回波。三维自身过滤只解决能确认属于安装件的回波，不能保证消除这些剩余来源，也不能恢复被硬件遮挡的环境信息。相机视频话题 `/exploration_camera/color/image_raw` 与这种 LiDAR 过滤独立，重新安装后省略 `--no-camera` 即可启用视频。
