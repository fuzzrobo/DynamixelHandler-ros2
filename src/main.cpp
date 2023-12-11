#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_handler/msg/dynamixel_cmd.hpp"
#include "dynamixel_handler_ROS2.h"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DynamixelHandler>());
	rclcpp::shutdown();
	return 0;
}
