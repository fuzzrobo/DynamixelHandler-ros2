#ifndef DYNAMIXEL_HANDLER_H_
#define DYNAMIXEL_HANDLER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dynamixel_serialport.h"
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp" // 今はいらんかも

using namespace std::chrono_literals;

// namespace dynamixel
// {

struct Params
{
  double gyroscope_noise_density;       // rad / (s * srqt(Hz))
  double gyroscope_random_walk;         // rad / (s ^ 2 * srqt(Hz))
  double accelerometer_noise_density;   // m / (s ^ 2 * srqt(Hz))
  double accelerometer_random_walk;     // m / (s ^ 3 * srqt(Hz))
  double calibration_frequency;
};

class  DynamixelHandler : public rclcpp::Node
{
public:
	
  	explicit  DynamixelHandler(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
  	virtual ~ DynamixelHandler();

private:
	bool param_;
	Params params_;
	double param_array_[2];
	// Path to directory to store the dump data. It will be used when enable_debug_mode_ is true.
	const std::string debug_dump_path_;

	// Subscribers
	// message_filters::Subscriber<ImageType> left_image_sub_;
	// const rclcpp::Subscription<ImageType>::SharedPtr imu_sub_;

	// // Publishers: Visual SLAM
	// const rclcpp::Publisher<VisualSlamStatusType>::SharedPtr visual_slam_status_pub_;

	// void CallbackReset(
	// 	const std::shared_ptr<SrvReset::Request> req,
	// 	std::shared_ptr<SrvReset::Response> res);

	// // SaveMap Action
	// rclcpp_action::Server<ActionSaveMap>::SharedPtr save_map_server_;
	// using GoalHandleSaveMap = rclcpp_action::ServerGoalHandle<ActionSaveMap>;
	// rclcpp_action::CancelResponse CallbackSaveMapCancel(
	// 	const std::shared_ptr<GoalHandleSaveMap> goal_handle);

	// // Callback function for images
	// void ReadImageData(
	// 	const ImageType::ConstSharedPtr & msg_left_img,
	// 	const CameraInfoType::ConstSharedPtr & msg_left_camera_info,
	// 	const ImageType::ConstSharedPtr & msg_right_img,
	// 	const CameraInfoType::ConstSharedPtr & msg_right_camera_info);

	// // Callback function for interleaved imu and image messages
	// void ComputePose(
	// 	const std::vector<ImageType::ConstSharedPtr> imu_msgs,
	// 	const std::pair<ImageType::ConstSharedPtr, ImageType::ConstSharedPtr> image_pair);

	// struct VisualSlamImpl;
	// std::unique_ptr<VisualSlamImpl> impl_;
};

// }  // namespace dynamixel

#endif  // DYNAMIXEL_HANDLER_H_