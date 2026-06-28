# se3_hopf

`se3_hopf` 已改为 ROS 2 Humble 节点，面向 PX4 + Micro XRCE Agent 的 Offboard 控制链路。

当前版本特性：

- 订阅 PX4 XRCE 话题：
  - `/fmu/out/vehicle_odometry`
  - `/fmu/out/vehicle_attitude`
  - `/fmu/out/vehicle_angular_velocity`
  - `/fmu/out/vehicle_imu`
  - `/fmu/out/vehicle_status`
- 订阅上层轨迹：
  - `/command/trajectory` (`trajectory_msgs/msg/MultiDOFJointTrajectory`)
- 发布 Offboard 控制：
  - `/fmu/in/offboard_control_mode`
  - `/fmu/in/vehicle_attitude_setpoint`
  - `/fmu/in/vehicle_rates_setpoint`
  - `/fmu/in/vehicle_command`
- 保留状态机：
  - 等待 PX4 数据
  - 预热 Offboard setpoint
  - 自动切 OFFBOARD
  - 自动 ARM
  - 自动 TAKEOFF / 任务执行 / LAND
- 保留 `/land` 服务：`std_srvs/srv/SetBool`

## 坐标系

- 节点固定假设接收的是 PX4 原生 NED/FRD 数据
- 控制器内部统一转换到 ENU 计算，再转换回 PX4 所需的姿态/角速度输出
- IMU 语义按 ROS1 MAVROS 路线对齐：
  - 姿态来自 `vehicle_attitude`
  - 角速度来自 `vehicle_angular_velocity`
  - 线加速度来自 `vehicle_imu`
- 因此上层通过 `/command/trajectory` 输入的轨迹应按 ROS 常用 ENU 语义给定

## 输出模式

通过参数 `control_mode` 选择：

- `attitude`
  - 发布 `px4_msgs/msg/VehicleAttitudeSetpoint`
- `body_rate`
  - 发布 `px4_msgs/msg/VehicleRatesSetpoint`

两种模式都使用算法输出的归一化推力，并映射到 PX4 期望的 `thrust_body[2]`。

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select se3_hopf
source install/setup.bash
```

## 启动

```bash
ros2 launch se3_hopf se3_hopf.launch.py
```

或直接运行：

```bash
ros2 run se3_hopf se3_hopf_node --ros-args --params-file src/Diff-Planner-PX4/src/se3_hopf/config/default.yaml
```

## 常用参数

- `px4_namespace`：默认 `/fmu/`
- `control_mode`：`attitude` 或 `body_rate`
- `enable_auto_offboard`
- `enable_auto_arm`
- `auto_takeoff`
- `takeoff_height`
- `hover_percent`
- `max_hover_percent`
- `geo_fence.x/y/z`

参数默认值见 [config/default.yaml](./config/default.yaml)。
