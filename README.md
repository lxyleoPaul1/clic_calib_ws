# clic_calib

**UAV–RTK 锚定的路侧 LiDAR / 相机外参标定**（连续时间 B 样条轨迹 + Ceres 批量优化），面向 ICRA 2027 论文。

原工程：[APRIL-ZJU/clic](https://github.com/APRIL-ZJU/clic)（车载 LICO SLAM）。本仓库**倒置传感器拓扑**：运动体为带 RTK 的 UAV，路侧 LiDAR 与相机静态安装；通过球靶（LiDAR）与 AprilTag（相机）建立跨模态几何约束，联合估计外参、时间偏移与 UAV 轨迹。

---

## 功能概览

| 模块 | 说明 |
|------|------|
| **连续时间轨迹** | SO(3)+R³ B 样条，RTK 位置因子锚定 |
| **LiDAR 因子** | 点到球隐式距离（RANSAC 球提取 → 批量因子） |
| **相机因子** | AprilTag 角点重投影（radtan 模型） |
| **时间偏移** | 可估计 LiDAR / 相机相对 RTK 的 `t_d^L`、`t_d^C` |
| **可观性分析** | Fisher 信息矩阵 Schur 补，外参子块条件数 |
| **合成仿真** | Python 轨迹 + 观测生成，无需 rosbag 即可端到端验证 |
| **离线工具链** | rosbag 预处理 → 标定 → 残差图 / FIM 图 → PDF 报告 |

---

## 项目结构

```
clic_calib/
├── app/                    # 可执行入口
│   ├── calibrate_offline.cpp
│   ├── analyze_observability.cpp
│   └── preprocess_rosbag.cpp
├── include/clic_calib/     # 头文件（因子、样条、IO、估计器）
├── src/clic_calib/         # 库实现
├── config/                 # YAML 配置（传感器、样条、噪声、靶标几何）
├── scripts/                # 仿真、实验、回归测试脚本
├── test/                   # GTest（因子 Jacobian、pipeline、可观性、噪声 sweep）
├── doc/                    # 数学规范、诊断、论文表格
│   ├── DERIVATIONS.md
│   ├── results/synthetic_evaluation.md
│   └── supplementary_section4.tex
└── legacy_clic/            # 自 clic 剥离的参考模块
```

---

## 依赖

### 系统 / ROS

- **ROS Noetic**（Ubuntu 20.04 推荐）
- **C++17** 编译器

### 第三方库

| 库 | 用途 |
|----|------|
| Eigen3 | 线性代数 |
| Ceres Solver | 非线性最小二乘 |
| yaml-cpp | 配置加载 |
| PCL | LiDAR 点云处理 |
| OpenCV | 相机图像 |
| libapriltag | AprilTag 检测 |
| Google Test | 单元 / 回归测试 |

无 ROS 环境时，可用 `scripts/compile_local_tests.sh` 直接 g++ 编译核心回归测试（因子 Jacobian + pipeline sweep）。

---

## 快速开始

### 1. 构建（ROS / catkin）

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
# 200 m 距离多层飞行（默认，利于 pitch / Z 可观性）
bash scripts/run_full_experiment.sh --range-m 200 /tmp/clic_multilayer

# 共面消融（可观性退化对照）
bash scripts/run_full_experiment.sh --coplanar /tmp/clic_coplanar
```

### 3. 分步运行

```bash
# 生成合成 obs.clicob + rtk.csv（噪声见 config/noise_model.yaml）
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

无 catkin 时脚本会自动调用 `compile_local_tests.sh` 并在 `build/local_tests/` 下运行二进制。

---

## 配置说明

所有 YAML 位于 `config/`，标定程序通过第一个参数传入配置目录：

| 文件 | 内容 |
|------|------|
| `sensor_rig.yaml` | 外参先验均值 / 标准差、时间偏移边界 |
| `spline.yaml` | B 样条阶数、结点间隔、平滑正则 |
| `noise_model.yaml` | **噪声 SSOT**：RTK / LiDAR / 相机 σ（仿真与估计器白化必须一致） |
| `lever_arms.yaml` | RTK 天线、球心、AprilTag 相对 body 的杠杆臂 |
| `target_geometry.yaml` | 球半径、AprilTag 尺寸与 ID |
| `target_detection.yaml` | 球提取与 AprilTag 检测阈值 |

默认噪声（`noise_model.yaml`）：

- RTK：σ_h = 10 mm，σ_v = 20 mm  
- LiDAR 测距：σ_r = 20 mm  
- 相机像素：σ_pix = 1.0 px  

---

## 测试分层

回归套件分四层，**请勿把 smoke 数字当作论文精度**：

| 层级 | 测试 | 作用 |
|------|------|------|
| **Smoke** | `test_full_pipeline_synthetic` | 无噪声接线 / Jacobian 冒烟（严格容差，毫秒级） |
| **Noise sweep** | `test_pipeline_noise_sweep` | N=20 种子，全模态噪声下的联合 pipeline 表征 |
| **200 m 可观性** | `test_patent_z_accuracy` | 多层 vs 共面 pitch / Z @ 200 m + FIM |
| **先验消融** | `test_prior_ablation` | 默认 / 弱 / 无外参先验下的几何可观性 |
| **FIM 结构** | `test_observability_synthetic` | 噪声-free Fisher 块结构消融 |
| **因子** | `test_*_factor_jacobian` | 解析 Jacobian 数值差分验证 |

论文级数值汇总见 [`doc/results/synthetic_evaluation.md`](doc/results/synthetic_evaluation.md)。

### 已知结论（合成评估，诚实表述）

- **不可声称**：在 realistic noise 下联合标定达到 cm 级精度；noise sweep 轨迹 RMS 均值约 **92 mm**，存在系统偏差。
- **可声称（限定条件）**：默认外参先验（5°、0.5 m）+ 多层激励下，200 m 处共面 Z 误差约为多层的 **3.4×**（9.996 mm vs 2.973 mm，N=20）。
- **不可声称**：当前 init=prior=GT 设定下 FIM 后验与 Monte Carlo 一致（二者几乎完全 disagree）。
- Smoke 测试仅验证**实现正确性**，不是方法论评估。

---

## 数学与文档

| 文档 | 说明 |
|------|------|
| [`doc/DERIVATIONS.md`](doc/DERIVATIONS.md) | §4.1–§4.8 规范数学定义（实现必须与此一致） |
| [`doc/supplementary_section4.tex`](doc/supplementary_section4.tex) | 论文 supplementary LaTeX |
| [`doc/results/synthetic_evaluation.md`](doc/results/synthetic_evaluation.md) | 噪声 regime 实验表格与 claim 路由 |
| [`doc/diagnostics/time_offset_observability.md`](doc/diagnostics/time_offset_observability.md) | 时间偏移 gauge / 诊断记录 |
| [`doc/MIGRATION.md`](doc/MIGRATION.md) | 自 clic 的模块迁移对照 |
| [`CHANGELOG.md`](CHANGELOG.md) | 分阶段开发记录 |
| [`RECON_REPORT.md`](RECON_REPORT.md) | 重构前代码侦察 |
| [`legacy_clic/README.md`](legacy_clic/README.md) | 剥离模块说明 |

---

## 开发分支

| 分支 | 说明 |
|------|------|
| `main` | 稳定基线 |
| `refactor/phase6-validation` | 端到端验证与文档 |
| `experiments/noise-regime` | 统一噪声模型 + N=20 seed sweep + 先验消融 + 论文表格 |

---

## 引用与上游

若使用本仓库，请同时引用上游 [clic / LICO](https://github.com/APRIL-ZJU/clic)。

---

## 许可

GPLv3（与上游 clic 一致）
