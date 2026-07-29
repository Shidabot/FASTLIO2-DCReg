# FAST-LIO2-DCReg

将 **DCReg（Decoupled Characterization for Efficient Degenerate LiDAR Registration）** 接入 FAST-LIO2 的 LiDAR-IMU 紧耦合迭代 EKF，实现退化场景下的在线退化检测、可解释诊断与定向正则化建图。

本仓库包含两部分：

| 目录 | 内容 |
| --- | --- |
| `FAST_LIO-main/` | 可编译运行的 FAST-LIO2-DCReg ROS1 工程 |
| `DCReg-main/` | DCReg 论文对应的独立配准实验、基线与实验结果 |

## 1. 解决什么问题

在长走廊、平面、隧道、稀疏结构、重复纹理或视场受限环境中，点云残差无法同时约束六自由度位姿。传统 FAST-LIO 的迭代更新可能出现条件数恶化、某些位姿方向过度更新，进而导致漂移或地图变形。

DCReg 将六自由度的可观测性拆分为平移与旋转两个子空间：

1. 从 FAST-LIO 的位姿信息矩阵中取出 6×6 位姿块，状态顺序为 `[tx, ty, tz, rx, ry, rz]`；
2. 通过 Schur 补分别消去旋转或平移耦合项，得到平移和旋转的解耦信息矩阵；
3. 对两个子空间分别特征分解，识别弱约束方向，并将其投影为 X/Y/Z 轴能量占比；
4. 仅对弱方向实施 MAP 谱正则化（特征值钳制），保留强约束方向和原有旋转—平移交叉项；
5. 使用 FAST-LIO 原有完整误差状态（23 维）LDLT 求解，保持 EKF 状态更新的一致性。

> 该实现将退化处理限制在每帧测量更新的**第一轮迭代**，避免反复修改同一帧的 Hessian。

## 2. 工程改动

| 文件 | 作用 |
| --- | --- |
| `FAST_LIO-main/include/dcreg.hpp` | Schur 补退化分析、弱方向判断、MAP 谱正则化 |
| `FAST_LIO-main/include/IKFoM_toolkit/esekfom/esekfom.hpp` | 在迭代 EKF 第一轮测量更新接入 DCReg，并输出诊断日志 |
| `FAST_LIO-main/src/laserMapping.cpp` | 读取 ROS 参数并传递给滤波器 |
| `FAST_LIO-main/config/*.yaml` | Avia、Horizon、MID360、Velodyne、Ouster 等传感器默认参数 |
| `FAST_LIO-main/include/ikd-Tree/` | 使用模板化 ikd-Tree 实现，确保与 `pcl::PointXYZINormal` 链接一致 |

## 3. 环境要求

- Ubuntu 18.04/20.04/22.04（推荐 Ubuntu 20.04）
- ROS1 Melodic 或 Noetic
- C++14、Eigen ≥ 3.3、PCL ≥ 1.8
- `livox_ros_driver`（Livox 传感器必需）
- 已正确安装并 source ROS 环境

本工程面向 Linux/ROS1 编译；Windows 用于编辑或 Git 操作，不直接编译 ROS 节点。

## 4. 编译

将工程放入 catkin 工作空间：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Shidabot/FASTLIO2-DCReg.git
ln -s ~/catkin_ws/src/FASTLIO2-DCReg/FAST_LIO-main fast_lio
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

也可以直接复制 `FAST_LIO-main` 到 `~/catkin_ws/src/fast_lio` 后编译：

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

若使用 Livox，请先 source 驱动工作空间：

```bash
source ~/ws_livox/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### 常见编译问题

**`KD_TREE is not a template` 或 `undefined reference to KD_TREE<...>`**

确认 `FAST_LIO-main/include/ikd-Tree/ikd_Tree.h` 与 `ikd_Tree.cpp` 是本仓库成对版本，之后清理并重新编译：

```bash
cd ~/catkin_ws
rm -rf build devel
catkin_make -DCMAKE_BUILD_TYPE=Release
```

不要只替换头文件；`ikd_Tree.cpp` 同样包含模板显式实例化。

## 5. 运行

先选择与 LiDAR 对应的配置和 launch 文件。

```bash
# Livox Avia
roslaunch fast_lio mapping_avia.launch

# Livox MID-360
roslaunch fast_lio mapping_mid360.launch

# Velodyne
roslaunch fast_lio mapping_velodyne.launch

# Ouster-64
roslaunch fast_lio mapping_ouster64.launch
```

然后启动传感器驱动或播放 rosbag：

```bash
rosbag play your_data.bag --clock
```

运行前请检查对应 YAML 中的：`lid_topic`、`imu_topic`、`scan_line`、`timestamp_unit`、`extrinsic_T`、`extrinsic_R`。LiDAR 与 IMU 时间同步、每点时间戳和外参正确性，是 DCReg 能发挥作用的前提。

## 6. DCReg 参数

配置位于各传感器 YAML 的 `mapping:` 下：

```yaml
dcreg_enable: true
dcreg_log_enable: true
dcreg_log_every_n_frames: 30
dcreg_eigenvalue_threshold: 120.0
dcreg_condition_threshold: 10.0
dcreg_kappa_target: 10.0
dcreg_regularization_alpha: 1.0
dcreg_inverse_relative_threshold: 1.0e-9
```

| 参数 | 含义 | 调参建议 |
| --- | --- | --- |
| `dcreg_enable` | 总开关 | 对照原始 FAST-LIO 时设为 `false` |
| `dcreg_log_enable` | 输出诊断日志 | 调参阶段开启，部署时可关闭 |
| `dcreg_log_every_n_frames` | 周期日志间隔 | 20–50；退化频繁时用 10–20 |
| `dcreg_eigenvalue_threshold` | 弱特征值判定阈值 | 默认 120；量纲受点数、残差权重、体素大小影响，必须结合日志调整 |
| `dcreg_condition_threshold` | 退化条件数阈值 | 建议 10 |
| `dcreg_kappa_target` | 正则化后的目标条件数 | 通常等于 `condition_threshold` |
| `dcreg_regularization_alpha` | 正则化强度 | 1.0 为完整修正；0.3–0.8 为更柔和的修正 |
| `dcreg_inverse_relative_threshold` | Schur 补逆的相对阈值 | 一般保持 `1e-9` |

### 推荐调参顺序

1. 用默认值跑一段包含走廊/平面的 rosbag，开启日志；
2. 观察弱方向和条件数是否与环境现象一致；
3. 若退化未触发，逐步降低 `dcreg_eigenvalue_threshold`，例如 `120 → 80 → 50`；
4. 若普通场景频繁触发或轨迹发紧，逐步提高阈值，或将 `regularization_alpha` 降至 `0.5–0.8`；
5. `condition_threshold` 与 `kappa_target` 保持相同，先从 10 开始，不要一开始设得很小；
6. 每次仅修改一个参数，并使用相同 rosbag 对比轨迹、局部地图和 CPU 占用。

`dcreg_eigenvalue_threshold` 不是跨数据集通用常数。它随特征点数量、点到平面权重、体素滤波和传感器噪声变化；日志比“固定抄参数”更可靠。

## 7. 日志判读

触发时终端会出现 `[DCReg]`。日志包含旋转/平移 Schur 子空间的条件数、特征值与弱方向。

- **translation / t**：平移子空间；
- **rotation / r**：旋转子空间；
- **X/Y/Z energy**：特征方向在对应物理坐标轴的能量占比；
- **large condition number**：该子空间存在显著弱约束；
- **weak direction**：该方向被实施谱正则化。

典型规律：

- 长直走廊常表现为沿走廊方向的平移弱约束；
- 大平面常表现为面内平移或绕法向旋转的弱约束；
- 稀疏或窄通道场景可能在平移和旋转子空间同时触发。

日志反映的是当前局部地图、扫描和 IMU 预测共同形成的信息矩阵，不等同于“环境标签”。如果日志长期异常而非仅在退化处触发，应先检查外参、时间戳、点云字段与 IMU 噪声参数。

## 8. 验证流程

建议至少做以下对照：

1. 将 `dcreg_enable: false` 跑一遍，保存轨迹与地图；
2. 使用相同 rosbag 设置为 `true` 再跑一遍；
3. 对比走廊/平面段轨迹连续性、回环前后的地图厚度、姿态抖动和运行频率；
4. 检查 `[DCReg]` 是否主要在低约束片段触发；
5. 不要只看“是否触发”，还应确认正常结构丰富区域没有被过度抑制。

## 9. 注意事项

- DCReg 用于增强退化条件下的数值稳定性，不能替代正确标定、可靠时间同步和足够的环境几何信息。
- 本实现使用完整 EKF 信息矩阵的 LDLT 求解，不把六自由度近似问题单独改成 PCG，避免破坏其余误差状态耦合。
- 每帧仅在第一轮测量迭代执行一次 DCReg，这与论文的集成原则一致。
- 项目中的 `DCReg-main` 含论文实验、结果与若干基线；日常建图只需使用 `FAST_LIO-main`。

## 10. 参考与致谢

- DCReg: *Decoupled Characterization for Efficient Degenerate LiDAR Registration*，论文与原始实验代码位于 `DCReg-main/`。
- FAST-LIO/FAST-LIO2：HKU MARS Lab 的 LiDAR-IMU 里程计框架。
- ikd-Tree：FAST-LIO 的增量地图近邻搜索结构。

请在学术或工程发布中遵循 DCReg、FAST-LIO 和所含第三方组件的原始许可证与引用要求。
