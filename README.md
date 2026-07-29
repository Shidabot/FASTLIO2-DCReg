# FAST-LIO2-DCReg

A ROS1 LiDAR-inertial odometry and mapping system based on FAST-LIO2. It supports Livox, Velodyne, Ouster, and other LiDAR sensors, with an integrated degeneracy-aware update for low-constraint scenes such as corridors, tunnels, large planes, and sparse environments.

This repository is an engineering integration and test project based on [FAST_LIO](https://github.com/hku-mars/FAST_LIO) and [DCReg](https://github.com/JokerJohn/DCReg/tree/main).

The runnable ROS package is located in [`FAST_LIO-main`](FAST_LIO-main/).

## Features

- Tightly coupled LiDAR-IMU odometry with an iterated error-state Kalman filter
- Direct scan-to-map registration and incremental ikd-Tree mapping
- Support for Livox Avia/Horizon/MID-360, Velodyne, and Ouster configurations
- Online degeneracy-aware pose update for weakly constrained geometry
- ROS topics, RViz visualization, and rosbag playback workflow compatible with FAST-LIO2

## Requirements

- Ubuntu 18.04/20.04/22.04 (Ubuntu 20.04 recommended)
- ROS Melodic or Noetic
- C++14
- Eigen >= 3.3 and PCL >= 1.8
- `livox_ros_driver` for Livox sensors

This is a ROS1/Linux project. Windows can be used for editing and Git operations, but the mapping node should be built and run on Linux with ROS.

## Build

Create a catkin workspace and place the ROS package under `src`:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Shidabot/FASTLIO2-DCReg.git
ln -s ~/catkin_ws/src/FASTLIO2-DCReg/FAST_LIO-main fast_lio

cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Alternatively, copy `FAST_LIO-main` directly to `~/catkin_ws/src/fast_lio` and run the same `catkin_make` command.

For Livox sensors, source the Livox driver workspace before building and running:

```bash
source ~/ws_livox/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## Run

Select the launch file that matches your LiDAR:

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

Then start the sensor driver or play a rosbag:

```bash
rosbag play your_data.bag --clock
```

Before running, edit the matching YAML file in `FAST_LIO-main/config/` and verify:

- `lid_topic` and `imu_topic`
- `scan_line` and `timestamp_unit` for spinning LiDARs
- `extrinsic_T` and `extrinsic_R`
- LiDAR-IMU synchronization and per-point timestamps

Correct calibration and timing are essential for stable odometry and mapping.

## Configuration

Each sensor configuration contains the normal FAST-LIO mapping parameters plus the optional degeneracy-aware update:

```yaml
mapping:
  dcreg_enable: true
  dcreg_log_enable: true
  dcreg_log_every_n_frames: 30
  dcreg_eigenvalue_threshold: 120.0
  dcreg_condition_threshold: 10.0
  dcreg_kappa_target: 10.0
  dcreg_regularization_alpha: 1.0
  dcreg_inverse_relative_threshold: 1.0e-9
```

| Parameter | Description | Recommended start value |
| --- | --- | --- |
| `dcreg_enable` | Enables the degeneracy-aware update | `true` |
| `dcreg_log_enable` | Prints diagnostic messages | `true` while tuning |
| `dcreg_log_every_n_frames` | Diagnostic print interval | `30` |
| `dcreg_eigenvalue_threshold` | Weak-direction detection threshold | `120.0` |
| `dcreg_condition_threshold` | Condition-number trigger threshold | `10.0` |
| `dcreg_kappa_target` | Target condition number after correction | `10.0` |
| `dcreg_regularization_alpha` | Correction strength | `1.0` |
| `dcreg_inverse_relative_threshold` | Numerical threshold for the Schur complement inverse | `1e-9` |

For a baseline FAST-LIO2 comparison, set `dcreg_enable: false`.

### Tuning guidance

Start with the provided defaults. Use the same rosbag for every comparison and change only one parameter at a time.

- If low-constraint segments are not detected, reduce `dcreg_eigenvalue_threshold` gradually (for example, `120 -> 80 -> 50`).
- If the correction activates frequently in feature-rich scenes, increase that threshold or reduce `dcreg_regularization_alpha` to `0.5-0.8`.
- Keep `dcreg_condition_threshold` and `dcreg_kappa_target` equal initially.
- The eigenvalue threshold depends on point count, voxel filtering, residual weights, and sensor noise; it is not a universal constant.

## Diagnostics

When enabled, the terminal prints `[DCReg]` messages. They report the conditioning of the translation and rotation subspaces, detected weak directions, and their X/Y/Z energy distribution.

Typical observations:

- A long corridor often weakens translation along its main direction.
- A dominant plane can weaken in-plane translation or rotation about the plane normal.
- Sparse or narrow spaces can weaken both translation and rotation.

The diagnostics describe the current local map and scan geometry; they should be interpreted together with trajectory quality and map appearance.

## Troubleshooting

### `KD_TREE is not a template` or undefined `KD_TREE` references

Make sure both files below come from this repository and are kept as a matching pair:

```text
FAST_LIO-main/include/ikd-Tree/ikd_Tree.h
FAST_LIO-main/include/ikd-Tree/ikd_Tree.cpp
```

Then perform a clean rebuild:

```bash
cd ~/catkin_ws
rm -rf build devel
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### The map drifts or the update is unstable

Check the following before changing degeneracy parameters:

1. LiDAR and IMU timestamps are synchronized.
2. The point cloud contains per-point time information.
3. LiDAR-to-IMU extrinsics are correct.
4. IMU noise and bias parameters match the sensor.
5. The LiDAR topic, scan line count, and timestamp unit match the driver output.

## Project structure

```text
FASTLIO2-DCReg/
└── FAST_LIO-main/          # ROS1 FAST-LIO2 mapping package
    ├── config/             # Sensor configurations
    ├── launch/             # ROS launch files
    ├── include/            # Filter, mapping, and utility headers
    └── src/                # Mapping and preprocessing nodes
```

## Credits

This project builds on [FAST_LIO](https://github.com/hku-mars/FAST_LIO) from HKU MARS Lab, integrates the degeneracy-aware method from [DCReg](https://github.com/JokerJohn/DCReg/tree/main), and uses ikd-Tree for incremental nearest-neighbor search. Please follow the licenses and citation requirements of the original projects and their dependencies.
