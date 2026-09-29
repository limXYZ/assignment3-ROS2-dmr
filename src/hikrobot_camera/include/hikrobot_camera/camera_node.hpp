#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode();

private:
  void captureLoop();
  void grabFrame();

  bool connectCamera();
  bool configureCamera();
  void disconnectCamera();

  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters);

  void * camera_handle_{nullptr};

  std::string selected_serial_;
  std::string selected_ip_;

  std::thread capture_thread_;
  std::atomic<bool> running_{false};
  std::mutex sdk_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  int consecutive_failures_{0};

  std::size_t frame_count_{0};
  std::chrono::steady_clock::time_point fps_start_time_{
    std::chrono::steady_clock::now()};
  double actual_fps_{0.0};
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
