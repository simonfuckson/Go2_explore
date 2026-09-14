# 平地探索简化版（2026-09-14）

## 修改依据

会话 `20260912_171300_cb9a17d9` 的状态记录覆盖约 177.5 秒。电量为 94%–96%。首次地形故障时，雷达拟合高度约 0.4859 m、坡度约 2.56°、拟合误差约 0.0031 m；近场支撑面积 0.1575 m² 低于原阈值 0.18 m²，触发 `terrain_health_unavailable`，随后一直等待恢复。该记录是状态和节点日志，不是完整点云录包。

地形故障之前还出现过一次局部代价图障碍停车。此次保留该碰撞保护；取消地形评分不会允许机器人穿过代价图中的障碍或未知区域。

## 默认行为

- 移除地面点数、连通面积、近场支撑、扇区数量、拟合高度及拟合质量对运动的否决；不再执行只为几何评分服务的地面拟合。
- 保留实际地面和障碍物提取、已观测空闲区清空、未知区检查、整形后的碰撞预测、速度上限及禁止倒车/横移。
- 保留有效输入数量、输入帧、时间戳、点云更新、处理时限与输出频率检查。雷达/TF/里程计/地图/规划指令异常仍会停止运动。
- 保留自动使能、符合条件的故障恢复、遥控接管、人工停止、完成判定、地图保存和 D435i 视频。

`go2_terrain_guard` 仍提供避障所需的点云，不能整体删除。`/terrain/healthy` 在平地模式下表示避障感知链有效；`/terrain/status` 的 `ground_geometry_checks=disabled` 明确标明模式。没有把该话题强制设为真，也没有把未知区域改为空闲。

## 启动

```bash
cd /home/nvidia/go2_explore_ws
./run_go2_explore explore NEW_MAP_NAME --real
```

默认打开 RViz、状态面板和 D435i 视频。地图名必须未使用。真机运动继续由现场验收。

对照原几何检查：

```bash
./run_go2_explore explore ANOTHER_NEW_MAP_NAME --real --terrain-geometry-checks
```

该选项对应 `terrain_geometry_checks:=true`；节点底层参数为 `/go2_terrain_guard/health/require_ground_geometry`。底层默认值保持 `true`，只有独立探索入口默认覆盖为 `false`。会话元数据会记录此选择。

停止和保存：`./run_go2_explore stop`。原 `/home/nvidia/go2_nav_ws` 不参与本次修改。

## 版本与验证

修改前源码备份标签：`backup-before-flat-ground-20260914`，提交 `0d7d20499aff34ffdbea5dbbb6960a19fff9a7a5`。备份包括源码、外参、脚本和固定版本第三方依赖；不包含运行地图、日志、构建产物和登录凭据。清单见 `backup/before_source_manifest.json`。

验证结果写入 `validation/flat_ground_20260914.json`。`scripts/verify_flat_terrain.py` 使用相同点云对照两种模式，检查稀疏地面放行、障碍和清空点云一致性，以及错误帧、空点云、低频和断流拒绝。模拟控制验证不调用真实 SDK。

本次修改不等于完成全屋覆盖或物理防碰撞验收。0.70 m 球形近场过滤仍按已有设置运行，它也会过滤该范围内的环境回波；实际近距离避障能力仍须现场验证。
