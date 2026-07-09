#include "se3_hopf/se3_ctrl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

namespace
{

std::string normalize_px4_namespace(std::string px4_namespace)
{
	if (px4_namespace.empty()) {
		return "/fmu/";
	}

	if (px4_namespace.front() != '/') {
		px4_namespace.insert(px4_namespace.begin(), '/');
	}

	if (px4_namespace.back() != '/') {
		px4_namespace.push_back('/');
	}

	return px4_namespace;
}

double saturate(double value, double min_value, double max_value)
{
	return std::max(min_value, std::min(max_value, value));
}

double rad2deg(double radians)
{
	return radians * 180.0 / M_PI;
}

double quaternion_angle_deg(Eigen::Quaterniond lhs, Eigen::Quaterniond rhs)
{
	lhs.normalize();
	rhs.normalize();
	Eigen::Quaterniond dq = lhs.conjugate() * rhs;
	dq.normalize();
	const double w = std::clamp(std::fabs(dq.w()), 0.0, 1.0);
	return rad2deg(2.0 * std::acos(w));
}

}  // namespace

Se3HopfCtrl::Se3HopfCtrl()
: Node("se3_hopf_node")
{
	loadParameters();

	offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
		px4_namespace_ + "in/offboard_control_mode", 10);
	attitude_sp_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
		px4_namespace_ + "in/vehicle_attitude_setpoint", 10);
	rates_sp_pub_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
		px4_namespace_ + "in/vehicle_rates_setpoint", 10);
	vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
		px4_namespace_ + "in/vehicle_command", 10);

	flight_state_pub_ = create_publisher<std_msgs::msg::Int8>("/flight_state", 10);
	reference_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/controller/reference_pose", 10);
	reference_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/controller/reference_velocity", 10);
	reference_acc_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>("/controller/reference_accel", 10);

	odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
		px4_namespace_ + "out/vehicle_odometry",
		rclcpp::SensorDataQoS(),
		std::bind(&Se3HopfCtrl::odomCallback, this, std::placeholders::_1));
	attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
		px4_namespace_ + "out/vehicle_attitude",
		rclcpp::SensorDataQoS(),
		std::bind(&Se3HopfCtrl::attitudeCallback, this, std::placeholders::_1));
	angular_velocity_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
		px4_namespace_ + "out/vehicle_angular_velocity",
		rclcpp::SensorDataQoS(),
		std::bind(&Se3HopfCtrl::angularVelocityCallback, this, std::placeholders::_1));
	imu_sub_ = create_subscription<px4_msgs::msg::VehicleImu>(
		px4_namespace_ + "out/vehicle_imu",
		rclcpp::SensorDataQoS(),
		std::bind(&Se3HopfCtrl::imuCallback, this, std::placeholders::_1));
	sensor_combined_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
		px4_namespace_ + "out/sensor_combined",
		rclcpp::SensorDataQoS(),
		std::bind(&Se3HopfCtrl::sensorCombinedCallback, this, std::placeholders::_1));
	status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
		px4_namespace_ + "out/vehicle_status_v1",
		rclcpp::SensorDataQoS(),
		std::bind(&Se3HopfCtrl::statusCallback, this, std::placeholders::_1));
	trajectory_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
		"/command/trajectory",
		10,
		std::bind(&Se3HopfCtrl::multiDOFJointCallback, this, std::placeholders::_1));

	land_service_ = create_service<std_srvs::srv::SetBool>(
		"/land",
		[this](
			const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
			std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
			(void)request;
			flight_state_ = LANDING;
			response->success = true;
			response->message = "landing requested";
		});

	se3_hopf_.init(hover_percent_, max_hover_percent_);
	se3_hopf_.setup(
		kp_p_, kp_v_, kp_a_, kp_q_, kp_w_,
		kd_p_, kd_v_, kd_a_, kd_q_, kd_w_,
		limit_err_p_, limit_err_v_, limit_err_a_,
		limit_d_err_p_, limit_d_err_v_, limit_d_err_a_);

	exec_timer_ = create_wall_timer(10ms, std::bind(&Se3HopfCtrl::execFSMCallback, this));

	RCLCPP_INFO(
		get_logger(),
		"se3_hopf ROS 2 node started. PX4 namespace: %s, control mode: %s, imu input: vehicle_attitude + vehicle_angular_velocity + vehicle_imu",
		px4_namespace_.c_str(),
		command_mode_ == CommandMode::Attitude ? "attitude" : "body_rate");
}

std::string Se3HopfCtrl::state2string(FlightState state) const
{
	switch (state) {
	case WAITING_FOR_CONNECTED:
		return "WAITING_FOR_CONNECTED";
	case WAITING_FOR_OFFBOARD:
		return "WAITING_FOR_OFFBOARD";
	case TAKEOFF:
		return "TAKEOFF";
	case MISSION_EXECUTION:
		return "MISSION_EXECUTION";
	case LANDING:
		return "LANDING";
	case LANDED:
		return "LANDED";
	case EMERGENCY:
		return "EMERGENCY";
	default:
		return "UNKNOWN";
	}
}

uint64_t Se3HopfCtrl::now_us() const
{
	return static_cast<uint64_t>(now().nanoseconds() / 1000);
}

void Se3HopfCtrl::loadParameters()
{
	px4_namespace_ = normalize_px4_namespace(declare_parameter<std::string>("px4_namespace", "/fmu/"));
	enable_auto_offboard_ = declare_parameter<bool>("enable_auto_offboard", true);
	enable_auto_arm_ = declare_parameter<bool>("enable_auto_arm", true);
	auto_takeoff_ = declare_parameter<bool>("auto_takeoff", true);
	publish_reference_topics_ = declare_parameter<bool>("publish_reference_topics", true);
	require_imu_for_control_ = declare_parameter<bool>("require_imu_for_control", true);
	// vel_in_body_ = declare_parameter<bool>("velocity_in_body", false);
	offboard_warmup_count_ = std::max<int>(
		1,
		static_cast<int>(declare_parameter<int64_t>("offboard_warmup_count", 20)));
	request_interval_ = std::max(0.1, declare_parameter<double>("request_interval", 1.0));
	takeoff_height_ = declare_parameter<double>("takeoff_height", 2.0);
	hover_percent_ = declare_parameter<double>("hover_percent", 0.25);
	max_hover_percent_ = declare_parameter<double>("max_hover_percent", 0.75);
	tracking_warn_pos_threshold_ = std::max(0.1, declare_parameter<double>("tracking_warn_pos_threshold", 0.8));
	tracking_warn_vel_threshold_ = std::max(0.1, declare_parameter<double>("tracking_warn_vel_threshold", 1.0));
	control_warn_bodyrate_threshold_ = std::max(0.1, declare_parameter<double>("control_warn_bodyrate_threshold", 4.0));
	control_warn_thrust_delta_threshold_ = std::max(0.01, declare_parameter<double>("control_warn_thrust_delta_threshold", 0.35));
	sample_sync_warn_ms_ = std::max(1.0, declare_parameter<double>("sample_sync_warn_ms", 20.0));
	odom_imu_quat_warn_deg_ = std::max(0.1, declare_parameter<double>("odom_imu_quat_warn_deg", 8.0));

	geo_fence_(0) = declare_parameter<double>("geo_fence.x", 10.0);
	geo_fence_(1) = declare_parameter<double>("geo_fence.y", 10.0);
	geo_fence_(2) = declare_parameter<double>("geo_fence.z", 6.0);

	const std::string control_mode = declare_parameter<std::string>("control_mode", "attitude");
	command_mode_ = (control_mode == "body_rate") ? CommandMode::BodyRate : CommandMode::Attitude;

	kp_p_ << declare_parameter<double>("kp_px", 0.85),
		declare_parameter<double>("kp_py", 0.85),
		declare_parameter<double>("kp_pz", 1.5);
	kp_v_ << declare_parameter<double>("kp_vx", 1.5),
		declare_parameter<double>("kp_vy", 1.5),
		declare_parameter<double>("kp_vz", 1.5);
	kp_a_ << declare_parameter<double>("kp_ax", 1.5),
		declare_parameter<double>("kp_ay", 1.5),
		declare_parameter<double>("kp_az", 1.5);
	kp_q_ << declare_parameter<double>("kp_qx", 5.5),
		declare_parameter<double>("kp_qy", 5.5),
		declare_parameter<double>("kp_qz", 0.1);
	kp_w_ << declare_parameter<double>("kp_wx", 1.5),
		declare_parameter<double>("kp_wy", 1.5),
		declare_parameter<double>("kp_wz", 0.1);

	kd_p_ << declare_parameter<double>("kd_px", 0.1),
		declare_parameter<double>("kd_py", 0.1),
		declare_parameter<double>("kd_pz", 0.0);
	kd_v_ << declare_parameter<double>("kd_vx", 0.0),
		declare_parameter<double>("kd_vy", 0.0),
		declare_parameter<double>("kd_vz", 0.0);
	kd_a_ << declare_parameter<double>("kd_ax", 0.0),
		declare_parameter<double>("kd_ay", 0.0),
		declare_parameter<double>("kd_az", 0.0);
	kd_q_ << declare_parameter<double>("kd_qx", 0.0),
		declare_parameter<double>("kd_qy", 0.0),
		declare_parameter<double>("kd_qz", 0.0);
	kd_w_ << declare_parameter<double>("kd_wx", 0.0),
		declare_parameter<double>("kd_wy", 0.0),
		declare_parameter<double>("kd_wz", 0.0);

	limit_err_p_ = declare_parameter<double>("limit_err_p", 3.0);
	limit_err_v_ = declare_parameter<double>("limit_err_v", 2.0);
	limit_err_a_ = declare_parameter<double>("limit_err_a", 1.0);
	limit_d_err_p_ = declare_parameter<double>("limit_d_err_p", 3.5);
	limit_d_err_v_ = declare_parameter<double>("limit_d_err_v", 1.0);
	limit_d_err_a_ = declare_parameter<double>("limit_d_err_a", 1.0);
}

void Se3HopfCtrl::logStateInputDiagnostics()
{
	if (!odom_received_ || !attitude_received_ || !linear_accel_received_) {
		return;
	}

	const auto delta_ms = [](uint64_t newer, uint64_t older) -> double {
		return static_cast<double>(static_cast<int64_t>(newer) - static_cast<int64_t>(older)) / 1000.0;
	};

	const double odom_att_ms = delta_ms(last_odom_timestamp_sample_, last_attitude_timestamp_sample_);
	const double odom_angvel_ms = delta_ms(last_odom_timestamp_sample_, last_angular_velocity_timestamp_sample_);
	const double odom_imu_ms = delta_ms(last_odom_timestamp_sample_, last_imu_timestamp_sample_);
	const double att_angvel_ms = delta_ms(last_attitude_timestamp_sample_, last_angular_velocity_timestamp_sample_);
	const double att_imu_ms = delta_ms(last_attitude_timestamp_sample_, last_imu_timestamp_sample_);
	const double quat_err_deg = quaternion_angle_deg(odom_data_.q, imu_data_.q);

	if (std::fabs(odom_att_ms) > sample_sync_warn_ms_ ||
		std::fabs(odom_angvel_ms) > sample_sync_warn_ms_ ||
		std::fabs(odom_imu_ms) > sample_sync_warn_ms_ ||
		std::fabs(att_angvel_ms) > sample_sync_warn_ms_ ||
		std::fabs(att_imu_ms) > sample_sync_warn_ms_ ||
		quat_err_deg > odom_imu_quat_warn_deg_) {
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			*get_clock(),
			500,
			"State input mismatch: pose_frame=%u velocity_frame=%u "
			"sample_ms(odom-att=%.2f odom-angvel=%.2f odom-imu=%.2f att-angvel=%.2f att-imu=%.2f) "
			"quat_err_deg=%.2f "
			"odom_q=(%.3f, %.3f, %.3f, %.3f) imu_q=(%.3f, %.3f, %.3f, %.3f)",
			static_cast<unsigned>(last_pose_frame_),
			static_cast<unsigned>(last_velocity_frame_),
			odom_att_ms,
			odom_angvel_ms,
			odom_imu_ms,
			att_angvel_ms,
			att_imu_ms,
			quat_err_deg,
			odom_data_.q.w(), odom_data_.q.x(), odom_data_.q.y(), odom_data_.q.z(),
			imu_data_.q.w(), imu_data_.q.x(), imu_data_.q.y(), imu_data_.q.z());
	}
}

void Se3HopfCtrl::publishReference() const
{
	if (!publish_reference_topics_) {
		return;
	}

	const auto stamp = now();
	geometry_msgs::msg::PoseStamped ref_pose;
	ref_pose.header.stamp = stamp;
	ref_pose.header.frame_id = "map";
	ref_pose.pose.position.x = desired_state_.p(0);
	ref_pose.pose.position.y = desired_state_.p(1);
	ref_pose.pose.position.z = desired_state_.p(2);
	ref_pose.pose.orientation.w = 1.0;
	reference_pose_pub_->publish(ref_pose);

	geometry_msgs::msg::TwistStamped ref_vel;
	ref_vel.header = ref_pose.header;
	ref_vel.twist.linear.x = desired_state_.v(0);
	ref_vel.twist.linear.y = desired_state_.v(1);
	ref_vel.twist.linear.z = desired_state_.v(2);
	reference_vel_pub_->publish(ref_vel);

	geometry_msgs::msg::AccelStamped ref_acc;
	ref_acc.header = ref_pose.header;
	ref_acc.accel.linear.x = desired_state_.a(0);
	ref_acc.accel.linear.y = desired_state_.a(1);
	ref_acc.accel.linear.z = desired_state_.a(2);
	reference_acc_pub_->publish(ref_acc);
}

void Se3HopfCtrl::publishOffboardControlMode() const
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = now_us();
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = (command_mode_ == CommandMode::Attitude);
	msg.body_rate = (command_mode_ == CommandMode::BodyRate);
	msg.thrust_and_torque = false;
	msg.direct_actuator = false;
	offboard_mode_pub_->publish(msg);
}

void Se3HopfCtrl::publishVehicleCommand(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = now_us();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	vehicle_cmd_pub_->publish(msg);
}

void Se3HopfCtrl::tryBuildImuFromFallback()
{
	if (!attitude_received_ || !sensor_combined_received_) {
		return;
	}

	if (vehicle_angular_velocity_received_ && vehicle_imu_received_) {
		return;
	}

	imu_data_.feed(last_attitude_, last_sensor_combined_);
	imu_received_ = true;
	using_sensor_combined_fallback_ = true;
	angular_velocity_received_ = true;
	linear_accel_received_ = true;
	last_angular_velocity_timestamp_sample_ = last_sensor_combined_timestamp_;
	last_imu_timestamp_sample_ = last_sensor_combined_timestamp_;

	if (!logged_sensor_combined_fallback_) {
		logged_sensor_combined_fallback_ = true;
		RCLCPP_WARN(
			get_logger(),
			"Using /fmu/out/sensor_combined as IMU fallback because vehicle_angular_velocity and/or vehicle_imu are unavailable.");
	}

	if (!logged_first_imu_bundle_) {
		logged_first_imu_bundle_ = true;
		RCLCPP_INFO(
			get_logger(),
			"First IMU bundle ready from sensor_combined fallback: q=(%.3f, %.3f, %.3f, %.3f) bodyrate=(%.3f, %.3f, %.3f) accel=(%.3f, %.3f, %.3f)",
			imu_data_.q.w(), imu_data_.q.x(), imu_data_.q.y(), imu_data_.q.z(),
			imu_data_.w(0), imu_data_.w(1), imu_data_.w(2),
			imu_data_.a(0), imu_data_.a(1), imu_data_.a(2));
	}
}

void Se3HopfCtrl::sendCommand(const Controller_Output_t &output, bool publish_orientation) const
{
	const float thrust_z = static_cast<float>(saturate(-output.thrust, -1.0, 0.0));

	if (command_mode_ == CommandMode::Attitude || publish_orientation) {
		px4_msgs::msg::VehicleAttitudeSetpoint msg{};
		msg.timestamp = now_us();
		msg.q_d[0] = static_cast<float>(output.q.w());
		msg.q_d[1] = static_cast<float>(output.q.x());
		msg.q_d[2] = static_cast<float>(output.q.y());
		msg.q_d[3] = static_cast<float>(output.q.z());
		msg.yaw_sp_move_rate = static_cast<float>(output.bodyrates(2));
		msg.thrust_body[0] = 0.0f;
		msg.thrust_body[1] = 0.0f;
		msg.thrust_body[2] = thrust_z;
		msg.reset_integral = false;
		msg.fw_control_yaw_wheel = false;
		attitude_sp_pub_->publish(msg);
		return;
	}

	px4_msgs::msg::VehicleRatesSetpoint msg{};
	msg.timestamp = now_us();
	msg.roll = static_cast<float>(output.bodyrates(0));
	msg.pitch = static_cast<float>(output.bodyrates(1));
	msg.yaw = static_cast<float>(output.bodyrates(2));
	msg.thrust_body[0] = 0.0f;
	msg.thrust_body[1] = 0.0f;
	msg.thrust_body[2] = thrust_z;
	msg.reset_integral = false;
	rates_sp_pub_->publish(msg);
}

void Se3HopfCtrl::sendNeutralCommand() const
{
	Controller_Output_t neutral;
	neutral.q = odom_received_ ? odom_data_.q : Eigen::Quaterniond::Identity();
	neutral.bodyrates.setZero();
	neutral.thrust = 0.0;
	sendCommand(neutral, true);
}

void Se3HopfCtrl::trySetOffboard()
{
	if (landing_locked_ || !enable_auto_offboard_ || !status_received_) {
		return;
	}
	if (curr_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
		return;
	}
	if (offboard_warmup_counter_ < offboard_warmup_count_) {
		return;
	}
	if ((now() - last_mode_request_).seconds() < request_interval_) {
		return;
	}

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
	last_mode_request_ = now();
	offboard_triggered_ = true;
	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Requested PX4 OFFBOARD mode.");
}

void Se3HopfCtrl::tryArm()
{
	if (landing_locked_ || !enable_auto_arm_ || !status_received_) {
		return;
	}
	if (curr_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
		return;
	}
	if (enable_auto_offboard_ &&
		curr_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
		return;
	}
	if ((now() - last_arm_request_).seconds() < request_interval_) {
		return;
	}

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
	last_arm_request_ = now();
	arm_triggered_ = true;
	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Requested PX4 arming.");
}

void Se3HopfCtrl::tryLand()
{
	if ((now() - last_land_request_).seconds() < request_interval_) {
		return;
	}

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
	last_land_request_ = now();
	landing_locked_ = true;
	RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Requested PX4 landing.");
}

void Se3HopfCtrl::execFSMCallback()
{
	std_msgs::msg::Int8 flight_state_msg;
	flight_state_msg.data = static_cast<int8_t>(flight_state_);
	flight_state_pub_->publish(flight_state_msg);
	publishReference();
	logStateInputDiagnostics();

	if (flight_state_ != prev_flight_state_) {
		RCLCPP_WARN(
			get_logger(),
			"State changed from %s to %s",
			state2string(prev_flight_state_).c_str(),
			state2string(flight_state_).c_str());
		prev_flight_state_ = flight_state_;
	}

	publishOffboardControlMode();

	switch (flight_state_) {
	case WAITING_FOR_CONNECTED:
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for PX4 status/odometry...");
		if (status_received_) {
			offboard_warmup_counter_ = 0;
			flight_state_ = WAITING_FOR_OFFBOARD;
		}
		break;

	case WAITING_FOR_OFFBOARD:
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Streaming setpoints before OFFBOARD...");
		sendNeutralCommand();
		++offboard_warmup_counter_;
		trySetOffboard();
		tryArm();
		if (status_received_ &&
			curr_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
			curr_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
			if (auto_takeoff_) {
				desired_state_.p = init_pose_;
				desired_state_.p(2) = init_pose_(2) + takeoff_height_;
				desired_state_.yaw = utils::fromQuaternion2yaw(odom_data_.q);
				desired_state_.yaw_rate = 0.0;
				flight_state_ = TAKEOFF;
			} else {
				flight_state_ = MISSION_EXECUTION;
			}
		}
		break;

	case TAKEOFF:
	case MISSION_EXECUTION:
	{
		const Eigen::Vector3d pos_err = desired_state_.p - odom_data_.p;
		const Eigen::Vector3d vel_err = desired_state_.v - odom_data_.v;
		if (pos_err.norm() > tracking_warn_pos_threshold_ || vel_err.norm() > tracking_warn_vel_threshold_) {
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				500,
				"Tracking error: state=%s pos=(%.2f, %.2f, %.2f) ref=(%.2f, %.2f, %.2f) pos_err=(%.2f, %.2f, %.2f)|%.2f "
				"vel=(%.2f, %.2f, %.2f) ref_vel=(%.2f, %.2f, %.2f) vel_err=(%.2f, %.2f, %.2f)|%.2f",
				flight_state_ == TAKEOFF ? "TAKEOFF" : "MISSION",
				odom_data_.p(0), odom_data_.p(1), odom_data_.p(2),
				desired_state_.p(0), desired_state_.p(1), desired_state_.p(2),
				pos_err(0), pos_err(1), pos_err(2), pos_err.norm(),
				odom_data_.v(0), odom_data_.v(1), odom_data_.v(2),
				desired_state_.v(0), desired_state_.v(1), desired_state_.v(2),
				vel_err(0), vel_err(1), vel_err(2), vel_err.norm());
		}

		if (require_imu_for_control_ && !imu_received_) {
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				1000,
				"IMU bundle not ready, holding neutral command. Expected either vehicle_attitude + vehicle_angular_velocity + vehicle_imu, or sensor_combined fallback.");
			sendNeutralCommand();
			break;
		}

		Controller_Output_t output;
		if (se3_hopf_.calControl(odom_data_, imu_data_, desired_state_, output)) {
			const double thrust_delta = std::fabs(output.thrust - last_commanded_thrust_);
			if (!logged_first_control_output_) {
				logged_first_control_output_ = true;
				RCLCPP_INFO(
					get_logger(),
					"First control output: thrust=%.3f q=(%.3f, %.3f, %.3f, %.3f) bodyrates=(%.3f, %.3f, %.3f)",
					output.thrust,
					output.q.w(), output.q.x(), output.q.y(), output.q.z(),
					output.bodyrates(0), output.bodyrates(1), output.bodyrates(2));
			}
			if (std::fabs(output.bodyrates(0)) > control_warn_bodyrate_threshold_ ||
				std::fabs(output.bodyrates(1)) > control_warn_bodyrate_threshold_ ||
				std::fabs(output.bodyrates(2)) > control_warn_bodyrate_threshold_ ||
				thrust_delta > control_warn_thrust_delta_threshold_) {
				RCLCPP_WARN_THROTTLE(
					get_logger(),
					*get_clock(),
					500,
					"Control effort spike: thrust=%.3f d_thrust=%.3f q=(%.3f, %.3f, %.3f, %.3f) bodyrates=(%.3f, %.3f, %.3f) mode=%s",
					output.thrust,
					thrust_delta,
					output.q.w(), output.q.x(), output.q.y(), output.q.z(),
					output.bodyrates(0), output.bodyrates(1), output.bodyrates(2),
					command_mode_ == CommandMode::Attitude ? "attitude" : "body_rate");
			}
			sendCommand(output, command_mode_ == CommandMode::Attitude);
			last_commanded_thrust_ = output.thrust;
			// RCLCPP_INFO_THROTTLE(
			// 	get_logger(),
			// 	*get_clock(),
			// 	1000,
			// 	"%s z=%.2f z_sp=%.2f thrust=%.3f rates=[%.2f %.2f %.2f]",
			// 	flight_state_ == TAKEOFF ? "TAKEOFF" : "MISSION",
			// 	odom_data_.p(2),
			// 	desired_state_.p(2),
			// 	output.thrust,
			// 	output.bodyrates(0),
			// 	output.bodyrates(1),
			// 	output.bodyrates(2));
			se3_hopf_.estimateTa(imu_data_.a);
		} else {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Control output not valid, sending neutral command.");
			sendNeutralCommand();
		}

		if (flight_state_ == TAKEOFF &&
			std::fabs(odom_data_.p(2) - desired_state_.p(2)) < 0.10) {
			RCLCPP_INFO(get_logger(), "Takeoff complete.");
			flight_state_ = MISSION_EXECUTION;
		}
		break;
	}

	case LANDING:
		sendNeutralCommand();
		tryLand();
		if (status_received_ &&
			curr_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
			flight_state_ = LANDED;
		}
		break;

	case LANDED:
		sendNeutralCommand();
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Vehicle landed.");
		break;

	case EMERGENCY:
		RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Emergency state, switching to landing.");
		flight_state_ = LANDING;
		break;
	}
}

void Se3HopfCtrl::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
	const bool velocity_in_body = (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD);
	RCLCPP_INFO_ONCE(
		get_logger(),
		"Odometry velocity frame: %s",
		velocity_in_body ? "BODY_FRD" : "LOCAL_NED");
	last_odom_timestamp_sample_ = msg->timestamp_sample;
	last_pose_frame_ = msg->pose_frame;
	last_velocity_frame_ = msg->velocity_frame;
	if (!logged_first_odom_frame_info_) {
		logged_first_odom_frame_info_ = true;
		RCLCPP_INFO(
			get_logger(),
			"VehicleOdometry frame info: pose_frame=%u velocity_frame=%u timestamp_sample=%llu",
			static_cast<unsigned>(msg->pose_frame),
			static_cast<unsigned>(msg->velocity_frame),
			static_cast<unsigned long long>(msg->timestamp_sample));
	}
	odom_data_.feed(*msg, velocity_in_body);
	odom_received_ = true;

	if (!desired_state_initialized_) {
		init_pose_ = odom_data_.p;
		desired_state_ = Desired_State_t(odom_data_);
		desired_state_initialized_ = true;
	}

	const bool judge_x = std::fabs(odom_data_.p(0)) >= geo_fence_(0);
	const bool judge_y = std::fabs(odom_data_.p(1)) >= geo_fence_(1);
	const bool judge_z = odom_data_.p(2) >= geo_fence_(2);
	if ((judge_x || judge_y || judge_z) &&
		curr_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
		RCLCPP_ERROR(
			get_logger(),
			"Geofence violated: pos=(%.2f, %.2f, %.2f) fence=(%.2f, %.2f, %.2f), entering EMERGENCY.",
			odom_data_.p(0), odom_data_.p(1), odom_data_.p(2),
			geo_fence_(0), geo_fence_(1), geo_fence_(2));
		flight_state_ = EMERGENCY;
	}
}

void Se3HopfCtrl::attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
	last_attitude_ = *msg;
	last_attitude_timestamp_sample_ = msg->timestamp_sample;
	attitude_received_ = true;

	if (vehicle_angular_velocity_received_ && vehicle_imu_received_) {
		imu_data_.feed(last_attitude_, last_angular_velocity_, last_vehicle_imu_);
		imu_received_ = true;
		using_sensor_combined_fallback_ = false;
	}

	tryBuildImuFromFallback();
}

void Se3HopfCtrl::angularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
{
	last_angular_velocity_ = *msg;
	last_angular_velocity_timestamp_sample_ = msg->timestamp_sample;
	vehicle_angular_velocity_received_ = true;
	angular_velocity_received_ = true;

	if (attitude_received_ && vehicle_imu_received_) {
		imu_data_.feed(last_attitude_, last_angular_velocity_, last_vehicle_imu_);
		imu_received_ = true;
		using_sensor_combined_fallback_ = false;
		if (!logged_regular_imu_bundle_) {
			logged_regular_imu_bundle_ = true;
			logged_first_imu_bundle_ = true;
			RCLCPP_INFO(
				get_logger(),
				"First IMU bundle ready from vehicle_angular_velocity + vehicle_imu: q=(%.3f, %.3f, %.3f, %.3f) bodyrate=(%.3f, %.3f, %.3f) accel=(%.3f, %.3f, %.3f)",
				imu_data_.q.w(), imu_data_.q.x(), imu_data_.q.y(), imu_data_.q.z(),
				imu_data_.w(0), imu_data_.w(1), imu_data_.w(2),
				imu_data_.a(0), imu_data_.a(1), imu_data_.a(2));
		}
	}
}

void Se3HopfCtrl::imuCallback(const px4_msgs::msg::VehicleImu::SharedPtr msg)
{
	last_vehicle_imu_ = *msg;
	last_imu_timestamp_sample_ = msg->timestamp_sample;
	vehicle_imu_received_ = true;
	linear_accel_received_ = true;

	if (attitude_received_ && vehicle_angular_velocity_received_) {
		imu_data_.feed(last_attitude_, last_angular_velocity_, last_vehicle_imu_);
		imu_received_ = true;
		using_sensor_combined_fallback_ = false;
		if (!logged_regular_imu_bundle_) {
			logged_regular_imu_bundle_ = true;
			logged_first_imu_bundle_ = true;
			RCLCPP_INFO(
				get_logger(),
				"First IMU bundle ready from vehicle_angular_velocity + vehicle_imu: q=(%.3f, %.3f, %.3f, %.3f) bodyrate=(%.3f, %.3f, %.3f) accel=(%.3f, %.3f, %.3f)",
				imu_data_.q.w(), imu_data_.q.x(), imu_data_.q.y(), imu_data_.q.z(),
				imu_data_.w(0), imu_data_.w(1), imu_data_.w(2),
				imu_data_.a(0), imu_data_.a(1), imu_data_.a(2));
		}
	}
}

void Se3HopfCtrl::sensorCombinedCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
{
	last_sensor_combined_ = *msg;
	last_sensor_combined_timestamp_ = msg->timestamp;
	sensor_combined_received_ = true;
	if (!vehicle_angular_velocity_received_ || !vehicle_imu_received_ || using_sensor_combined_fallback_) {
		tryBuildImuFromFallback();
	}
}

void Se3HopfCtrl::statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
	curr_status_ = *msg;
	status_received_ = true;
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		2000,
		"PX4 status: nav_state=%u arming_state=%u failsafe=%s pre_flight_checks=%s",
		curr_status_.nav_state,
		curr_status_.arming_state,
		curr_status_.failsafe ? "true" : "false",
		curr_status_.pre_flight_checks_pass ? "true" : "false");
	if (curr_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND && !landing_locked_) {
		landing_locked_ = true;
		RCLCPP_WARN(get_logger(), "Landing lock enabled (PX4 AUTO_LAND detected).");
	}
}

void Se3HopfCtrl::multiDOFJointCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
{
	if (msg->points.empty() || msg->points.front().transforms.empty()) {
		RCLCPP_WARN(get_logger(), "Received empty trajectory message.");
		return;
	}

	const auto &pt = msg->points.front();
	const auto &transform = pt.transforms.front();

	desired_state_.p(0) = transform.translation.x;
	desired_state_.p(1) = transform.translation.y;
	desired_state_.p(2) = transform.translation.z;

	if (!pt.velocities.empty()) {
		desired_state_.v(0) = pt.velocities.front().linear.x;
		desired_state_.v(1) = pt.velocities.front().linear.y;
		desired_state_.v(2) = pt.velocities.front().linear.z;
	} else {
		desired_state_.v.setZero();
	}

	if (!pt.accelerations.empty()) {
		desired_state_.a(0) = pt.accelerations.front().linear.x;
		desired_state_.a(1) = pt.accelerations.front().linear.y;
		desired_state_.a(2) = pt.accelerations.front().linear.z;
	} else {
		desired_state_.a.setZero();
	}

	desired_state_.j.setZero();
	desired_state_.q = Eigen::Quaterniond(
		transform.rotation.w,
		transform.rotation.x,
		transform.rotation.y,
		transform.rotation.z);
	desired_state_.q.normalize();
	desired_state_.yaw = utils::fromQuaternion2yaw(desired_state_.q);
	desired_state_.yaw_rate = 0.0;

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		200,
		"Trajectory command: points=%zu pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f) acc=(%.2f, %.2f, %.2f) yaw=%.2f",
		msg->points.size(),
		desired_state_.p(0), desired_state_.p(1), desired_state_.p(2),
		desired_state_.v(0), desired_state_.v(1), desired_state_.v(2),
		desired_state_.a(0), desired_state_.a(1), desired_state_.a(2),
		desired_state_.yaw);
}
