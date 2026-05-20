# clic_calib

UAV–RTK 锚定的路侧 LiDAR / 相机外参标定（连续时间 B 样条），面向 ICRA 2027。

原工程：[APRIL-ZJU/clic](https://github.com/APRIL-ZJU/clic)（车载 LICO SLAM）。本仓库**倒置传感器拓扑**：运动体为带 RTK 的 UAV，路侧传感器静态。

## Quickstart

### 1. 构建（ROS Noetic）

```bash
cd ~/catkin_ws/src
ln -s /path/to/clic_calib .
cd ~/catkin_ws
catkin_make --pkg clic_calib -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
```

### 2. 端到端实验（合成数据，无需 rosbag）

```bash
cd /path/to/clic_calib
bash scripts/run_full_experiment.sh /tmp/clic_experiment
# 输出: /tmp/clic_experiment/experiment_report.pdf
```

可选参数：

```bash
# 200 m 距离多层飞行（默认）
bash scripts/run_full_experiment.sh --range-m 200 /tmp/clic_multilayer

# 共面消融（可观性退化）
bash scripts/run_full_experiment.sh --coplanar /tmp/clic_coplanar
```

### 3. 分步运行

```bash
# 生成合成 obs.clicob + rtk.csv
python3 scripts/simulate_uav_trajectory.py --output-dir /tmp/clic --range-m 200

# 离线标定
rosrun clic_calib calibrate_offline config /tmp/clic/obs.clicob /tmp/clic/rtk.csv \
  -o /tmp/clic/calibration.json

# 可观性分析
rosrun clic_calib analyze_observability config /tmp/clic/obs.clicob /tmp/clic/rtk.csv \
  -o /tmp/clic/observability.json

# 可视化
python3 scripts/plot_residuals.py /tmp/clic/calibration.json -o /tmp/clic/residuals.png
python3 scripts/fim_visualizer.py /tmp/clic/observability.json -o /tmp/clic/fim.png
python3 scripts/generate_experiment_report.py /tmp/clic -o /tmp/clic/experiment_report.pdf
```

### 4. 真实 rosbag

```bash
bash scripts/run_full_experiment.sh --rosbag /path/to/flight.bag /tmp/clic_real
```

需已安装 PCL、apriltag、OpenCV，并完成 `preprocess_rosbag` 构建。

### 5. 回归测试

```bash
bash scripts/run_regression_tests.sh
```

覆盖 Phase 2 因子 Jacobian、合成 pipeline、可观性消融、专利 Z 轴精度（200 m 多层 < 0.1 m vs 共面 > 1 m）。

## 文档

- `doc/DERIVATIONS.md` — 规范数学定义（§4.1–§4.8）；LaTeX 见 `doc/supplementary_section4.tex`
- `doc/MIGRATION.md` — 模块迁移对照表
- `RECON_REPORT.md` — 重构前代码侦察
- `legacy_clic/README.md` — 从 clic 剥离的模块说明

## 许可

GPLv3（与上游 clic 一致）
