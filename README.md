# tanh-ctrl
Robust controller for PX4 quadrotor based on hyperbolic tangent function feedback 

## 仿真工作区

仓库现在包含三类 ROS 2 包：

- `src/tanh_ctrl`：tanh 四旋翼控制器，订阅 PX4 风格状态 topic，发布 thrust/torque setpoint。
- `src/utils/flat_trajectory_msgs`：自定义 `FlatTrajectoryReference` 消息包，供控制器、轨迹发布器和仿真器共享。
- `src/utils/px4_msgs`：PX4 ROS 2 消息 submodule，用于论文代码开源复现。
- `src/uav_simulator`：MuJoCo Python 仿真桥，使用真实四旋翼参数改写的 Skydio X2 MJCF 模型。
- `src/traj_pub`：C++ 李萨如 8 字参考轨迹发布器。

首次克隆后拉取 submodule：

```bash
git submodule update --init --recursive
```

Python 仿真依赖：

```bash
python3 -m pip install -r src/uav_simulator/requirements.txt
```

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

单独启动 MuJoCo-PX4 桥：

```bash
ros2 launch uav_simulator uav_simulator.launch.py use_viewer:=true
```

启动仿真器、控制器和 8 字轨迹发布器：

```bash
ros2 launch tanh_ctrl mujoco_sim.launch.py use_viewer:=true
```

`src/traj_pub/config/lissajous_figure8.yaml` 可调整 8 字轨迹参数，包括 `origin_ned`、`amplitude_ned`、`period_s`、各轴 harmonic/phase 和 `yaw_rad`。默认等待控制器发布 `/mission/start_tracking` 后开始发布 `/tanh_ctrl/reference`。
