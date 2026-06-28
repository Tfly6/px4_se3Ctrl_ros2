#ifndef SE3_CONTROLLER_UTILS_HPP
#define SE3_CONTROLLER_UTILS_HPP

#include <cmath>

#include <Eigen/Dense>

namespace utils
{

inline double fromQuaternion2yaw(const Eigen::Quaterniond &q)
{
	return std::atan2(
		2.0 * (q.x() * q.y() + q.w() * q.z()),
		q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
}

inline Eigen::Vector3d quat2euler(const Eigen::Quaterniond &q)
{
	return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

inline Eigen::Quaterniond euler2quat(double roll, double pitch, double yaw)
{
	return Eigen::Quaterniond(
		Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

inline double atan2_dot(double y, double x, double y_dot, double x_dot)
{
	if (x == 0.0 && y == 0.0) {
		return 0.0;
	}

	return (y_dot * x - y * x_dot) / (x * x + y * y);
}

}  // namespace utils

#endif
