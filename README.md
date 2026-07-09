# se3_hopf

ROS 2 Humble SE(3) geometric controller with Hopf fibration for PX4 offboard control.

## Features

- **SE(3) geometric controller** based on Hopf fibration, with full position-velocity-acceleration error feedback and derivative control
- **Dual IMU input paths** — primary from `vehicle_angular_velocity` + `vehicle_imu`; automatic fallback to `sensor_combined` (gyro + accelerometer) when those topics are unavailable
- **Runtime diagnostics** — tracking error threshold warnings, control effort spike detection, sample timestamp synchronisation monitoring, odometry/attitude quaternion discrepancy checks
- **Safety** — configurable geo-fence with automatic emergency landing; IMU readiness gate holding neutral command until sensor bundle is ready
- **State machine** — WAITING_FOR_CONNECTED → OFFBOARD → ARM → TAKEOFF → MISSION_EXECUTION → LAND → LANDED, with EMERGENCY handling
- Dual output mode: `attitude` (default) or `body_rate`

### Subscribed topics (PX4 XRCE)

| Topic | Type |
|-------|------|
| `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` |
| `/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` |
| `/fmu/out/vehicle_angular_velocity` | `px4_msgs/msg/VehicleAngularVelocity` |
| `/fmu/out/vehicle_imu` | `px4_msgs/msg/VehicleImu` |
| `/fmu/out/sensor_combined` | `px4_msgs/msg/SensorCombined` (fallback IMU) |
| `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` |
| `/command/trajectory` | `trajectory_msgs/msg/MultiDOFJointTrajectory` |

### Published topics

| Topic | Type |
|-------|------|
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` |
| `/fmu/in/vehicle_attitude_setpoint` | `px4_msgs/msg/VehicleAttitudeSetpoint` |
| `/fmu/in/vehicle_rates_setpoint` | `px4_msgs/msg/VehicleRatesSetpoint` |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` |
| `/flight_state` | `std_msgs/msg/Int8` |
| `/controller/reference_pose` | `geometry_msgs/msg/PoseStamped` |
| `/controller/reference_velocity` | `geometry_msgs/msg/TwistStamped` |
| `/controller/reference_accel` | `geometry_msgs/msg/AccelStamped` |

### Services

- `/land` — `std_srvs/srv/SetBool`

## Coordinate frames

- Input assumed to be PX4 native NED/FRD; internally transformed to ENU for computation, then converted back to PX4 attitude/bodyrate output
- Trajectory via `/command/trajectory` should be given in ROS-standard ENU semantics
- IMU pipeline:
  - Attitude: `vehicle_attitude`
  - Angular velocity: `vehicle_angular_velocity` (or `sensor_combined.gyro_rad`)
  - Linear acceleration: `vehicle_imu` (or `sensor_combined.accelerometer_m_s2`)

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Tfly6/px4_se3Ctrl_ros2.git
# Required: px4_msgs
git clone https://github.com/PX4/px4_msgs.git
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select se3_hopf
source install/setup.bash
```

## Usage

> Make sure the official ROS 2 Offboard example works before first use.

1. Start PX4 SITL:
```bash
cd <PX4_DIR>
make px4_sitl gz_x500
```

2. Start Micro XRCE Agent:
```bash
MicroXRCEAgent udp4 -p 8888
```

3. Launch controller:
```bash
ros2 launch se3_hopf se3_hopf.launch.py
```

Or run directly:
```bash
ros2 run se3_hopf se3_hopf_node --ros-args --params-file src/se3_hopf/config/default.yaml
```

## Parameters

See [config/default.yaml](./config/default.yaml) for full defaults.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `px4_namespace` | `/fmu/` | PX4 topic namespace |
| `control_mode` | `attitude` | Output mode: `attitude` or `body_rate` |
| `require_imu_for_control` | `true` | Hold neutral command until IMU bundle ready |
| `enable_auto_offboard` | `true` | Automatically switch to OFFBOARD mode |
| `enable_auto_arm` | `true` | Automatically arm |
| `auto_takeoff` | `true` | Automatically take off after arm |
| `takeoff_height` | `2.0` | Takeoff altitude (m) |
| `hover_percent` / `max_hover_percent` | `0.25` / `0.75` | Thrust mapping range |
| `geo_fence.x/y/z` | `80.0` / `80.0` / `7.0` | Position boundary (m) |
| `tracking_warn_pos_threshold` | `0.8` | Position tracking warning (m) |
| `tracking_warn_vel_threshold` | `1.0` | Velocity tracking warning (m/s) |
| `control_warn_bodyrate_threshold` | `4.0` | Bodyrate spike warning (rad/s) |
| `control_warn_thrust_delta_threshold` | `0.35` | Thrust step warning (normalised) |
| `sample_sync_warn_ms` | `20.0` | Sample timestamp diff warning (ms) |
| `odom_imu_quat_warn_deg` | `8.0` | Odom-IMU quaternion mismatch warning (deg) |

And full set of `kp_px/py/pz`, `kp_vx/vy/vz`, `kp_ax/ay/az`, `kp_qx/qy/qz`, `kp_wx/wy/wz` gains with their derivative counterparts `kd_*`, plus error/derivative limits `limit_err_p/v/a`, `limit_d_err_p/v/a`.

## References

- [HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller) — Original SE(3) controller
- [Tfly6/OpenDrone](https://github.com/Tfly6/OpenDrone) — PX4 and ROS SITL framework
