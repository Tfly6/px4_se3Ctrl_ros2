#ifndef SE3_CTRL_H
#define SE3_CTRL_H

#include <string>

#include <Eigen/Dense>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include "se3_hopf/se3_hopf.hpp"

class Se3HopfCtrl : public rclcpp::Node
{
public:
	Se3HopfCtrl();
	~Se3HopfCtrl() override = default;

private:
	enum class CommandMode
	{
		Attitude,
		BodyRate
	};

	enum FlightState
	{
		WAITING_FOR_CONNECTED,
		WAITING_FOR_OFFBOARD,
		TAKEOFF,
		MISSION_EXECUTION,
		LANDING,
		LANDED,
		EMERGENCY
	};

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_sp_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_sp_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr flight_state_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_pub_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr reference_vel_pub_;
	rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr reference_acc_pub_;

	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr imu_sub_;
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
	rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_sub_;

	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr land_service_;
	rclcpp::TimerBase::SharedPtr exec_timer_;

	px4_msgs::msg::VehicleStatus curr_status_{};
	px4_msgs::msg::VehicleAttitude last_attitude_{};
	px4_msgs::msg::VehicleAngularVelocity last_angular_velocity_{};
	px4_msgs::msg::VehicleImu last_vehicle_imu_{};
	px4_msgs::msg::SensorCombined last_sensor_combined_{};
	Odom_Data_t odom_data_;
	Imu_Data_t imu_data_;
	Desired_State_t desired_state_;
	SE3_HOPF_CONTROLLER se3_hopf_;

	bool status_received_{false};
	bool odom_received_{false};
	bool imu_received_{false};
	bool attitude_received_{false};
	bool angular_velocity_received_{false};
	bool linear_accel_received_{false};
	bool sensor_combined_received_{false};
	bool vehicle_angular_velocity_received_{false};
	bool vehicle_imu_received_{false};
	bool desired_state_initialized_{false};
	bool landing_locked_{false};
	bool enable_auto_offboard_{true};
	bool enable_auto_arm_{true};
	bool auto_takeoff_{true};
	bool debug_log_{false};
	bool require_imu_for_control_{true};
	bool using_sensor_combined_fallback_{false};
	bool publish_reference_topics_{true};
	bool vel_in_body_{false};
	bool offboard_triggered_{false};
	bool arm_triggered_{false};
	bool logged_first_control_output_{false};
	bool logged_first_imu_bundle_{false};
	bool logged_first_odom_frame_info_{false};
	bool logged_sensor_combined_fallback_{false};
	bool logged_regular_imu_bundle_{false};
	int offboard_warmup_counter_{0};
	int offboard_warmup_count_{20};
	double request_interval_{1.0};
	double takeoff_height_{2.0};
	double hover_percent_{0.25};
	double max_hover_percent_{0.75};
	double tracking_warn_pos_threshold_{0.8};
	double tracking_warn_vel_threshold_{1.0};
	double control_warn_bodyrate_threshold_{4.0};
	double control_warn_thrust_delta_threshold_{0.35};
	double sample_sync_warn_ms_{20.0};
	double odom_imu_quat_warn_deg_{8.0};
	double last_commanded_thrust_{0.0};
	Eigen::Vector3d init_pose_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d geo_fence_{10.0, 10.0, 4.0};

	Eigen::Vector3d kp_p_, kp_v_, kp_a_, kp_q_, kp_w_, kd_p_, kd_v_, kd_a_, kd_q_, kd_w_;
	double limit_err_p_{3.0}, limit_err_v_{2.0}, limit_err_a_{1.0};
	double limit_d_err_p_{3.5}, limit_d_err_v_{1.0}, limit_d_err_a_{1.0};

	CommandMode command_mode_{CommandMode::Attitude};
	FlightState flight_state_{WAITING_FOR_CONNECTED};
	FlightState prev_flight_state_{WAITING_FOR_CONNECTED};

	std::string px4_namespace_{"/fmu/"};
	rclcpp::Time last_mode_request_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_arm_request_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_land_request_{0, 0, RCL_ROS_TIME};
	uint64_t last_odom_timestamp_sample_{0};
	uint64_t last_attitude_timestamp_sample_{0};
	uint64_t last_angular_velocity_timestamp_sample_{0};
	uint64_t last_imu_timestamp_sample_{0};
	uint64_t last_sensor_combined_timestamp_{0};
	uint8_t last_pose_frame_{px4_msgs::msg::VehicleOdometry::POSE_FRAME_UNKNOWN};
	uint8_t last_velocity_frame_{px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN};

	std::string state2string(FlightState state) const;
	uint64_t now_us() const;
	void loadParameters();
	void logStateInputDiagnostics();
	void execFSMCallback();
	void publishReference() const;
	void publishOffboardControlMode() const;
	void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
	void sendCommand(const Controller_Output_t &output, bool publish_orientation) const;
	void sendNeutralCommand() const;
	void trySetOffboard();
	void tryArm();
	void tryLand();
	void tryBuildImuFromFallback();

	void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
	void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
	void angularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
	void imuCallback(const px4_msgs::msg::VehicleImu::SharedPtr msg);
	void sensorCombinedCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
	void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
	void multiDOFJointCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg);
};

#endif
