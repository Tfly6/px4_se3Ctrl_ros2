#ifndef SE3_HOPF_HPP
#define SE3_HOPF_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <utility>

#include <Eigen/Dense>

#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "se3_hopf/utils.hpp"

struct Odom_Data_t
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Vector3d p{Eigen::Vector3d::Zero()};
	Eigen::Vector3d v{Eigen::Vector3d::Zero()};
	Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
	Eigen::Vector3d w{Eigen::Vector3d::Zero()};

	std::chrono::steady_clock::time_point rcv_stamp{};
	bool recv_new_msg{false};

	static Eigen::Matrix3d nedEnuRotation()
	{
		Eigen::Matrix3d rotation;
		rotation << 0.0, 1.0, 0.0,
			1.0, 0.0, 0.0,
			0.0, 0.0, -1.0;
		return rotation;
	}

	static Eigen::Matrix3d frdFluRotation()
	{
		Eigen::Matrix3d rotation;
		rotation << 1.0, 0.0, 0.0,
			0.0, -1.0, 0.0,
			0.0, 0.0, -1.0;
		return rotation;
	}

	void feed(
		const px4_msgs::msg::VehicleOdometry &msg,
		bool vel_in_body)
	{
		rcv_stamp = std::chrono::steady_clock::now();
		recv_new_msg = true;

		p << msg.position[0], msg.position[1], msg.position[2];
		v << msg.velocity[0], msg.velocity[1], msg.velocity[2];
		w << msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2];

		const Eigen::Matrix3d world_rotation = nedEnuRotation();
		const Eigen::Matrix3d body_rotation = frdFluRotation();
		const Eigen::Matrix3d px4_rotation =
			Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]).normalized().toRotationMatrix();

		p = world_rotation * p;
		q = Eigen::Quaterniond(world_rotation * px4_rotation * body_rotation);
		q.normalize();
		w = body_rotation * w;

		if (vel_in_body) {
			v = q.toRotationMatrix() * (body_rotation * v);
		} else {
			v = world_rotation * v;
		}
	}
};

struct Desired_State_t
{
	Eigen::Vector3d p{Eigen::Vector3d::Zero()};
	Eigen::Vector3d v{Eigen::Vector3d::Zero()};
	Eigen::Vector3d a{Eigen::Vector3d::Zero()};
	Eigen::Vector3d j{Eigen::Vector3d::Zero()};
	Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
	double yaw{0.0};
	double yaw_rate{0.0};

	Desired_State_t() = default;

	explicit Desired_State_t(const Odom_Data_t &odom)
	{
		p = odom.p;
		v.setZero();
		a.setZero();
		j.setZero();
		q = odom.q.normalized();
		yaw = utils::fromQuaternion2yaw(q);
		yaw_rate = 0.0;
	}
};

struct Controller_Output_t
{
	Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
	Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
	double thrust{0.0};
};

struct Imu_Data_t
{
	Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
	Eigen::Vector3d w{Eigen::Vector3d::Zero()};
	Eigen::Vector3d a{Eigen::Vector3d::Zero()};

	std::chrono::steady_clock::time_point rcv_stamp{};
	bool recv_new_msg{false};
	bool recv_attitude{false};
	bool recv_angular_velocity{false};
	bool recv_linear_acceleration{false};

	void feed(
		const px4_msgs::msg::VehicleAttitude &attitude_msg,
		const px4_msgs::msg::VehicleAngularVelocity &angular_velocity_msg,
		const px4_msgs::msg::VehicleImu &imu_msg)
	{
		rcv_stamp = std::chrono::steady_clock::now();
		recv_new_msg = true;
		recv_attitude = true;
		recv_angular_velocity = true;
		recv_linear_acceleration = true;

		q = Eigen::Quaterniond(
			attitude_msg.q[0],
			attitude_msg.q[1],
			attitude_msg.q[2],
			attitude_msg.q[3]);
		q.normalize();

		w << angular_velocity_msg.xyz[0], angular_velocity_msg.xyz[1], angular_velocity_msg.xyz[2];

		const double accel_dt = static_cast<double>(imu_msg.delta_velocity_dt);
		if (accel_dt > 0.0) {
			a << imu_msg.delta_velocity[0], imu_msg.delta_velocity[1], imu_msg.delta_velocity[2];
			a *= 1e6 / accel_dt;
		} else {
			a.setZero();
		}

		const Eigen::Matrix3d body_rotation = Odom_Data_t::frdFluRotation();
		const Eigen::Matrix3d world_rotation = Odom_Data_t::nedEnuRotation();
		const Eigen::Matrix3d px4_rotation = q.toRotationMatrix();
		q = Eigen::Quaterniond(world_rotation * px4_rotation * body_rotation);
		q.normalize();
		w = body_rotation * w;
		a = body_rotation * a;
	}
};

class SE3_HOPF_CONTROLLER
{
private:
	Eigen::Vector3d Kp_p_, Kp_v_, Kp_a_, Kp_q_, Kp_w_, Kd_p_, Kd_v_, Kd_a_, Kd_q_, Kd_w_;
	double limit_err_p_{3.0}, limit_err_v_{2.0}, limit_err_a_{1.0};
	double limit_d_err_p_{3.5}, limit_d_err_v_{1.0}, limit_d_err_a_{1.0};
	bool have_last_err_{false};

	double hover_percent_{0.25}, max_hover_percent_{0.75};
	double T_a_{0.0};
	double P_ = 1e6;
	const double rho_ = 0.998;
	const double gravity_ = 9.81;
	Eigen::Vector3d grav_vec_{0.0, 0.0, gravity_};
	Eigen::Vector3d last_err_p_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d last_err_v_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d last_err_a_{Eigen::Vector3d::Zero()};
	std::queue<std::pair<std::chrono::steady_clock::time_point, double>> timed_thrust_;

	void computeFlatInput_Hopf_Fibration(const Desired_State_t &desired_state, Odom_Data_t &desired_odom)
	{
		const Eigen::Vector3d abc = desired_state.a.normalized();
		const double a = abc(0);
		const double b = abc(1);
		const double c = abc(2);
		const Eigen::Vector3d abc_dot =
			(desired_state.a.dot(desired_state.a) * Eigen::MatrixXd::Identity(3, 3)
			 - desired_state.a * desired_state.a.transpose()) /
			desired_state.a.norm() / desired_state.a.squaredNorm() * desired_state.j;
		const double a_dot = abc_dot(0);
		const double b_dot = abc_dot(1);
		const double c_dot = abc_dot(2);
		double yaw = desired_state.yaw;
		double yaw_dot = desired_state.yaw_rate;
		double syaw = std::sin(yaw);
		double cyaw = std::cos(yaw);

		if (c > 0.0) {
			const double norm = std::sqrt(2.0 * (1.0 + c));
			const Eigen::Quaterniond q((1.0 + c) / norm, -b / norm, a / norm, 0.0);
			const Eigen::Quaterniond q_yaw(std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
			desired_odom.q = q * q_yaw;
			desired_odom.w(0) = syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot / (c + 1.0);
			desired_odom.w(1) = cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot / (c + 1.0);
			desired_odom.w(2) = (b * a_dot - a * b_dot) / (1.0 + c) + yaw_dot;
		} else {
			const double norm = std::sqrt(2.0 * (1.0 - c));
			const Eigen::Quaterniond q(-b / norm, (1.0 - c) / norm, 0.0, a / norm);
			yaw += 2.0 * std::atan2(a, b);
			const Eigen::Quaterniond q_yaw(std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
			desired_odom.q = q * q_yaw;
			syaw = std::sin(yaw);
			cyaw = std::cos(yaw);

			desired_odom.w(0) = syaw * a_dot + cyaw * b_dot - (a * syaw + b * cyaw) * c_dot / (c - 1.0);
			desired_odom.w(1) = cyaw * a_dot - syaw * b_dot - (a * cyaw - b * syaw) * c_dot / (c - 1.0);
			desired_odom.w(2) = (b * a_dot - a * b_dot) / (c - 1.0) + yaw_dot;
		}
	}

	void limitErr(Eigen::Vector3d &err, double low, double upper)
	{
		err(0) = std::max(std::min(err(0), upper), low);
		err(1) = std::max(std::min(err(1), upper), low);
		err(2) = std::max(std::min(err(2), upper), low);
	}

public:
	SE3_HOPF_CONTROLLER() = default;
	~SE3_HOPF_CONTROLLER() = default;

	void init(double hover_percent, double max_hover_percent)
	{
		hover_percent_ = hover_percent;
		max_hover_percent_ = max_hover_percent;
		T_a_ = gravity_ / hover_percent_;
		grav_vec_ << 0.0, 0.0, gravity_;
		last_err_p_.setZero();
		last_err_v_.setZero();
		last_err_a_.setZero();
		have_last_err_ = false;
	}

	void setup(
		const Eigen::Vector3d &kp_p,
		const Eigen::Vector3d &kp_v,
		const Eigen::Vector3d &kp_a,
		const Eigen::Vector3d &kp_q,
		const Eigen::Vector3d &kp_w,
		const Eigen::Vector3d &kd_p,
		const Eigen::Vector3d &kd_v,
		const Eigen::Vector3d &kd_a,
		const Eigen::Vector3d &kd_q,
		const Eigen::Vector3d &kd_w,
		double limit_err_p,
		double limit_err_v,
		double limit_err_a,
		double limit_d_err_p,
		double limit_d_err_v,
		double limit_d_err_a)
	{
		Kp_p_ = kp_p;
		Kp_v_ = kp_v;
		Kp_a_ = kp_a;
		Kp_q_ = kp_q;
		Kp_w_ = kp_w;
		Kd_p_ = kd_p;
		Kd_v_ = kd_v;
		Kd_a_ = kd_a;
		Kd_q_ = kd_q;
		Kd_w_ = kd_w;
		limit_err_p_ = limit_err_p;
		limit_err_v_ = limit_err_v;
		limit_err_a_ = limit_err_a;
		limit_d_err_p_ = limit_d_err_p;
		limit_d_err_v_ = limit_d_err_v;
		limit_d_err_a_ = limit_d_err_a;
	}

	bool calControl(
		const Odom_Data_t &odom_data,
		const Imu_Data_t &imu_data,
		Desired_State_t desired_state,
		Controller_Output_t &output)
	{
		const auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration<double>(now - odom_data.rcv_stamp).count() > 0.1) {
			return false;
		}

		Eigen::Vector3d err_p = odom_data.p - desired_state.p;
		limitErr(err_p, -limit_err_p_, limit_err_p_);
		if (!have_last_err_) {
			last_err_p_ = err_p;
		}
		Eigen::Vector3d d_err_p = err_p - last_err_p_;
		limitErr(d_err_p, -limit_d_err_p_, limit_d_err_p_);
		desired_state.v = desired_state.v - Kp_p_.asDiagonal() * err_p - Kd_p_.asDiagonal() * d_err_p;

		Eigen::Vector3d err_v = odom_data.v - desired_state.v;
		limitErr(err_v, -limit_err_v_, limit_err_v_);
		if (!have_last_err_) {
			last_err_v_ = err_v;
		}
		Eigen::Vector3d d_err_v = err_v - last_err_v_;
		limitErr(d_err_v, -limit_d_err_v_, limit_d_err_v_);
		desired_state.a = desired_state.a - Kp_v_.asDiagonal() * err_v - Kd_v_.asDiagonal() * d_err_v + grav_vec_;

		const Eigen::Vector3d a_world = odom_data.q.toRotationMatrix() * imu_data.a;
		Eigen::Vector3d err_a = a_world - desired_state.a;
		limitErr(err_a, -limit_err_a_, limit_err_a_);
		if (!have_last_err_) {
			last_err_a_ = err_a;
		}
		Eigen::Vector3d d_err_a = err_a - last_err_a_;
		limitErr(d_err_a, -limit_d_err_a_, limit_d_err_a_);
		desired_state.j = desired_state.j - Kp_a_.asDiagonal() * err_a - Kd_a_.asDiagonal() * d_err_a;

		last_err_p_ = err_p;
		last_err_v_ = err_v;
		last_err_a_ = err_a;
		have_last_err_ = true;

		const double thr = desired_state.a.transpose() * (odom_data.q * Eigen::Vector3d::UnitZ());
		output.thrust = std::clamp(thr / T_a_, 0.0, 1.0);

		Odom_Data_t desired_odom;
		computeFlatInput_Hopf_Fibration(desired_state, desired_odom);
		output.q = imu_data.q * odom_data.q.inverse() * desired_odom.q;

		const Eigen::Quaterniond err_q = odom_data.q.inverse() * desired_odom.q;
		Eigen::Vector3d err_br;
		if (err_q.w() >= 0.0) {
			err_br.x() = Kp_q_(0) * err_q.x();
			err_br.y() = Kp_q_(1) * err_q.y();
			err_br.z() = Kp_q_(2) * err_q.z();
		} else {
			err_br.x() = -Kp_q_(0) * err_q.x();
			err_br.y() = -Kp_q_(1) * err_q.y();
			err_br.z() = -Kp_q_(2) * err_q.z();
		}

		output.bodyrates = desired_odom.w + err_br;

		const Eigen::Matrix3d world_rotation = Odom_Data_t::nedEnuRotation();
		const Eigen::Matrix3d body_rotation = Odom_Data_t::frdFluRotation();
		output.q = Eigen::Quaterniond(
			world_rotation.transpose() * output.q.toRotationMatrix() * body_rotation.transpose());
		output.q.normalize();
		output.bodyrates = body_rotation.transpose() * output.bodyrates;

		timed_thrust_.push(std::make_pair(now, output.thrust));
		while (timed_thrust_.size() > 100) {
			timed_thrust_.pop();
		}
		return true;
	}

	bool estimateTa(const Eigen::Vector3d &est_a)
	{
		const auto t_now = std::chrono::steady_clock::now();
		while (!timed_thrust_.empty()) {
			const auto t_t = timed_thrust_.front();
			const double time_passed = std::chrono::duration<double>(t_now - t_t.first).count();
			if (time_passed > 0.045) {
				timed_thrust_.pop();
				continue;
			}
			if (time_passed < 0.035) {
				return false;
			}

			const double thr = t_t.second;
			timed_thrust_.pop();

			const double gamma = 1.0 / (rho_ + thr * P_ * thr);
			const double K = gamma * P_ * thr;
			T_a_ = T_a_ + K * (est_a(2) - thr * T_a_);
			P_ = (1.0 - K * thr) * P_ / rho_;
			T_a_ = std::max(T_a_, gravity_ / max_hover_percent_);
			return true;
		}
		return false;
	}
};

#endif
