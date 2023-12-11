#include "dynamixel_handler_ROS2.h"

DynamixelHandler::DynamixelHandler(rclcpp::NodeOptions options)
: 	Node("dynamixel_handler", options),
	// Node parameters.
	param_(declare_parameter<bool>("denoise_input_images", false)),
	params_{
		declare_parameter<double>("gyro_noise_density", 0.000244),
		declare_parameter<double>("gyro_random_walk", 0.000019393),
		declare_parameter<double>("accel_noise_density", 0.001862),
		declare_parameter<double>("accel_random_walk", 0.003),
		declare_parameter<double>("calibration_frequency", 200.0)},
	param_array_{
		declare_parameter<double>("gyro_noise_density", 0.000244),
		declare_parameter<double>("gyro_random_walk", 0.000019393)
	}	
{

}

DynamixelHandler::~DynamixelHandler()
{
}
