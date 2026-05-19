# clic_calib

UAV–RTK 锚定的路侧 LiDAR / 相机外参标定（连续时间 B 样条），面向 ICRA 2027。

原工程：[APRIL-ZJU/clic](https://github.com/APRIL-ZJU/clic)（车载 LICO SLAM）。本仓库**倒置传感器拓扑**：运动体为带 RTK 的 UAV，路侧传感器静态。

## 构建（ROS Noetic）

```bash
cd ~/catkin_ws/src
ln -s /path/to/clic_calib .
cd ~/catkin_ws
catkin_make --pkg clic_calib
catkin_make run_tests_clic_calib_gtest_test_spline_recovery  # 可选
```

## 运行（Phase 0 骨架）

```bash
rosrun clic_calib calibrate_offline _config_dir:=/path/to/clic_calib/config
```

当前 Phase 0 仅验证包结构与 B 样条基础设施；优化因子在 Phase 1 接入。

## 文档

- `RECON_REPORT.md` — 重构前代码侦察
- `doc/DERIVATIONS.md` — **规范数学定义（§4.1–§4.8，代码审查对照此文件）**
- `doc/MIGRATION.md` — **模块迁移对照表（§5）**
- `legacy_clic/README.md` — 从 clic 剥离的模块说明

## 许可

GPLv3（与上游 clic 一致）
