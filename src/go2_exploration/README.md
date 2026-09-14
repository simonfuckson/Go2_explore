# GO2 在线探索

操作入口和验收边界见工作空间根目录 `README.md` 与 `artifacts/ACCEPTANCE.md`。
使用 `/home/nvidia/go2_explore_ws/run_go2_explore`，不要直接启动硬件 launch，否则无法获得与原导航入口的互斥保护。

本包从小车 wheeltec_exploration 迁移，复用 GO2 的建图、外参、实时地形和控制。独立 move_base 使用 `/nav_static_map` 全局规划、原 GO2 地形点云局部避障。ObservedObstacleLayer 输出未膨胀观测，observed_map_memory 维护碰撞记忆与地面覆盖，go2_explore_lite 选择前沿/覆盖缺口，监督器管理开始、普通障碍恢复、硬故障、完成和保存。

几何读取 GO2 0.70 × 0.31 m 轮廓与 0.03 m 余量；雷达安装方向读取原外参。预测视野只参与收益评分，不能成为地面已观测证据。碰撞图扩展保留旧观测；连续 30 秒没有合格任务且新增两次有效地图更新才允许完成。有任务无路径、数据过期或障碍等待均不算完成。

`launch/mock_simulation.launch` 仅用于完全虚拟的自动化联调。日常 `observe` 使用真实传感器、模拟底盘和候选预览；默认 `explore` 自动逻辑仍连接模拟底盘；只有 `explore NAME --real` 才加载真实 SDK。日志与地图仅进入新工作空间。
