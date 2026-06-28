#include "se3_hopf/se3_ctrl.h"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Se3HopfCtrl>());
	rclcpp::shutdown();
	return 0;
}
