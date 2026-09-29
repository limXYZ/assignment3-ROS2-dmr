#include "hikrobot_camera/camera_node.hpp"
#include "MvCameraControl.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace hikrobot_camera
{
namespace
{

std::string ipToString(unsigned int ip)
{
  return
    std::to_string((ip >> 24) & 0xFF) + "." +
    std::to_string((ip >> 16) & 0xFF) + "." +
    std::to_string((ip >> 8) & 0xFF) + "." +
    std::to_string(ip & 0xFF);
}

std::string deviceSerial(const MV_CC_DEVICE_INFO * device)
{
  if (device == nullptr) {
    return "";
  }

  if (device->nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char *>(
      device->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }

  if (device->nTLayerType == MV_GIGE_DEVICE) {
    return reinterpret_cast<const char *>(
      device->SpecialInfo.stGigEInfo.chSerialNumber);
  }

  return "";
}

std::string deviceIp(const MV_CC_DEVICE_INFO * device)
{
  if (device == nullptr || device->nTLayerType != MV_GIGE_DEVICE) {
    return "";
  }

  return ipToString(device->SpecialInfo.stGigEInfo.nCurrentIp);
}

}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  // -------------------- ROS parameters --------------------
  declare_parameter<double>("exposure", 5000.0);     // microseconds
  declare_parameter<double>("gain", 0.0);
  declare_parameter<double>("frame_rate", 60.0);     // FPS

  declare_parameter<std::vector<std::string>>(
    "known_serials", std::vector<std::string>{});

  declare_parameter<std::string>("selected_serial", "");
  declare_parameter<std::string>("selected_ip", "");
  declare_parameter<std::string>("image_topic", "/image_raw");
  declare_parameter<std::string>("pixel_format", "BayerRG8");
  declare_parameter<std::string>("frame_id", "camera");

  const auto known_serials =
    get_parameter("known_serials").as_string_array();

  selected_serial_ =
    get_parameter("selected_serial").as_string();

  selected_ip_ =
    get_parameter("selected_ip").as_string();

  const std::string image_topic =
    get_parameter("image_topic").as_string();

  image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(
    get_logger(), "Publishing images on topic: %s", image_topic.c_str());

  // -------------------- Enumerate USB + GigE cameras --------------------
  MV_CC_DEVICE_INFO_LIST device_list{};

  const int enum_ret =
    MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);

  if (enum_ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to enumerate MVS cameras, error code: 0x%x",
      enum_ret);
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Found %u MVS camera(s).", device_list.nDeviceNum);

  if (device_list.nDeviceNum == 0) {
    RCLCPP_ERROR(get_logger(), "No USB or GigE MVS camera found.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Available cameras（如果要改，请在camera.yaml的selected_serial中修改（暂时没实现在终端中选择的功能））:");

  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    MV_CC_DEVICE_INFO * device = device_list.pDeviceInfo[i];
    if (device == nullptr) {
      continue;
    }

    const std::string serial = deviceSerial(device);
    const bool registered =
      std::find(
        known_serials.begin(), known_serials.end(), serial) !=
      known_serials.end();

    if (device->nTLayerType == MV_USB_DEVICE) {
      RCLCPP_INFO(
        get_logger(),
        "[%u] type=USB serial=%s %s",
        i,
        serial.c_str(),
        registered ? "[registered]" : "[unregistered]");
    } else if (device->nTLayerType == MV_GIGE_DEVICE) {
      const std::string ip = deviceIp(device);
      RCLCPP_INFO(
        get_logger(),
        "[%u] type=GigE serial=%s ip=%s %s",
        i,
        serial.c_str(),
        ip.c_str(),
        registered ? "[registered]" : "[unregistered]");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[%u] Unsupported transport layer type: 0x%x",
        i,
        device->nTLayerType);
    }
  }

  // Exactly one selection method must be used.
  if (selected_serial_.empty() && selected_ip_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Neither selected_serial nor selected_ip was specified. "
      "Choose exactly one camera selection method.");
    return;
  }

  if (!selected_serial_.empty() && !selected_ip_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Both selected_serial and selected_ip were specified. "
      "Choose only one selection method.");
    return;
  }

  if (!selected_serial_.empty()) {
    const bool selected_registered =
      std::find(
        known_serials.begin(),
        known_serials.end(),
        selected_serial_) != known_serials.end();

    if (selected_registered) {
      RCLCPP_INFO(
        get_logger(),
        "Selected registered camera by serial: %s",
        selected_serial_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Selected camera %s is not registered in known_serials. "
        "Using it for this session only.",
        selected_serial_.c_str());
    }
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Selected GigE camera by IP: %s",
      selected_ip_.c_str());
  }

  // -------------------- Connect + configure --------------------
  if (!connectCamera()) {
    if (!selected_serial_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to connect selected camera serial=%s.",
        selected_serial_.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to connect selected camera ip=%s.",
        selected_ip_.c_str());
    }
    return;
  }

  if (!configureCamera()) {
    RCLCPP_ERROR(get_logger(), "Failed to configure camera.");
    disconnectCamera();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    const int grab_ret = MV_CC_StartGrabbing(camera_handle_);

    if (grab_ret != MV_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to start grabbing, error code: 0x%x",
        grab_ret);
      // Unlock before disconnectCamera() to avoid locking the same mutex twice.
      // The actual cleanup is done immediately below after leaving this scope.
      camera_handle_ = camera_handle_;
    } else {
      RCLCPP_INFO(get_logger(), "Camera started grabbing.");
      consecutive_failures_ = 0;
      frame_count_ = 0;
      fps_start_time_ = std::chrono::steady_clock::now();

      parameter_callback_handle_ =
        add_on_set_parameters_callback(
        std::bind(
          &CameraNode::onParametersSet,
          this,
          std::placeholders::_1));

      running_ = true;
      capture_thread_ =
        std::thread(&CameraNode::captureLoop, this);
      return;
    }
  }

  disconnectCamera();
}

CameraNode::~CameraNode()
{
  running_ = false;

  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  disconnectCamera();
}

rcl_interfaces::msg::SetParametersResult CameraNode::onParametersSet(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(sdk_mutex_);

  if (camera_handle_ == nullptr) {
    result.successful = false;
    result.reason = "Camera is not connected.";
    return result;
  }

  for (const auto & param : parameters) {
    if (param.get_name() == "exposure") {
      const double requested = param.as_double();
      MVCC_FLOATVALUE info{};

      int ret =
        MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &info);

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Failed to get ExposureTime range.";
        return result;
      }

      if (requested < info.fMin || requested > info.fMax) {
        result.successful = false;
        result.reason =
          "Exposure is out of camera range [" +
          std::to_string(info.fMin) + ", " +
          std::to_string(info.fMax) + "] us.";
        return result;
      }

      ret =
        MV_CC_SetFloatValue(
        camera_handle_,
        "ExposureTime",
        static_cast<float>(requested));

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Failed to set camera exposure.";
        return result;
      }

      MVCC_FLOATVALUE verify{};
      ret =
        MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &verify);

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Exposure was set but readback failed.";
        return result;
      }

      RCLCPP_INFO(
        get_logger(),
        "Exposure dynamically updated: requested=%.2f us, actual=%.2f us",
        requested,
        verify.fCurValue);
    } else if (param.get_name() == "gain") {
      const double requested = param.as_double();
      MVCC_FLOATVALUE info{};

      int ret = MV_CC_GetFloatValue(camera_handle_, "Gain", &info);

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Failed to get Gain range.";
        return result;
      }

      if (requested < info.fMin || requested > info.fMax) {
        result.successful = false;
        result.reason =
          "Gain is out of camera range [" +
          std::to_string(info.fMin) + ", " +
          std::to_string(info.fMax) + "].";
        return result;
      }

      ret =
        MV_CC_SetFloatValue(
        camera_handle_,
        "Gain",
        static_cast<float>(requested));

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Failed to set camera gain.";
        return result;
      }

      MVCC_FLOATVALUE verify{};
      ret = MV_CC_GetFloatValue(camera_handle_, "Gain", &verify);

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Gain was set but readback failed.";
        return result;
      }

      RCLCPP_INFO(
        get_logger(),
        "Gain dynamically updated: requested=%.2f, actual=%.2f",
        requested,
        verify.fCurValue);
    } else if (param.get_name() == "frame_rate") {
      const double requested = param.as_double();
      MVCC_FLOATVALUE info{};

      int ret =
        MV_CC_GetFloatValue(
        camera_handle_, "AcquisitionFrameRate", &info);

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Failed to get frame rate range.";
        return result;
      }

      if (requested < info.fMin || requested > info.fMax) {
        result.successful = false;
        result.reason =
          "Frame rate is out of camera range [" +
          std::to_string(info.fMin) + ", " +
          std::to_string(info.fMax) + "] FPS.";
        return result;
      }

      ret =
        MV_CC_SetFloatValue(
        camera_handle_,
        "AcquisitionFrameRate",
        static_cast<float>(requested));

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Failed to set camera frame rate.";
        return result;
      }

      MVCC_FLOATVALUE verify{};
      ret =
        MV_CC_GetFloatValue(
        camera_handle_, "AcquisitionFrameRate", &verify);

      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "Frame rate was set but readback failed.";
        return result;
      }

      RCLCPP_INFO(
        get_logger(),
        "Frame rate dynamically updated: requested=%.2f FPS, actual=%.2f FPS",
        requested,
        verify.fCurValue);
    } else if (
      param.get_name() == "pixel_format" ||
      param.get_name() == "selected_serial" ||
      param.get_name() == "selected_ip" ||
      param.get_name() == "image_topic" ||
      param.get_name() == "frame_id" ||
      param.get_name() == "known_serials")
    {
      result.successful = false;
      result.reason =
        "Parameter '" + param.get_name() +
        "' is startup-only and cannot be changed at runtime.";
      return result;
    }
  }

  return result;
}

void CameraNode::captureLoop()
{
  while (running_) {
    grabFrame();
  }
}

void CameraNode::grabFrame()
{
  MV_FRAME_OUT frame{};
  int frame_ret = MV_E_NODATA;

  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);

    if (camera_handle_ == nullptr) {
      frame_ret = MV_E_NODATA;
    } else {
      frame_ret =
        MV_CC_GetImageBuffer(camera_handle_, &frame, 1000);
    }
  }

  if (frame_ret == MV_OK) {
    ++frame_count_;
    consecutive_failures_ = 0;

    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - fps_start_time_).count();

    if (elapsed >= 1.0) {
      actual_fps_ =
        static_cast<double>(frame_count_) / elapsed;

      RCLCPP_INFO(
        get_logger(),
        "Actual receive/pipeline FPS: %.2f",
        actual_fps_);

      frame_count_ = 0;
      fps_start_time_ = now;
    }

    cv::Mat bayer(
      frame.stFrameInfo.nHeight,
      frame.stFrameInfo.nWidth,
      CV_8UC1,
      frame.pBufAddr);

    cv::Mat bgr;
    cv::cvtColor(bayer, bgr, cv::COLOR_BayerBG2BGR);

    // cvtColor has copied/converted the pixels into bgr, so release the SDK
    // buffer as early as possible.
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      if (camera_handle_ != nullptr) {
        const int free_ret =
          MV_CC_FreeImageBuffer(camera_handle_, &frame);

        if (free_ret != MV_OK) {
          RCLCPP_WARN(
            get_logger(),
            "Failed to free image buffer, error code: 0x%x",
            free_ret);
        }
      }
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = this->now();
    image_msg.header.frame_id =
      get_parameter("frame_id").as_string();

    image_msg.height =
      static_cast<sensor_msgs::msg::Image::_height_type>(bgr.rows);
    image_msg.width =
      static_cast<sensor_msgs::msg::Image::_width_type>(bgr.cols);
    image_msg.encoding = "bgr8";
    image_msg.is_bigendian = false;
    image_msg.step =
      static_cast<sensor_msgs::msg::Image::_step_type>(bgr.cols * 3);

    image_msg.data.assign(
      bgr.data,
      bgr.data + bgr.total() * bgr.elemSize());

    image_pub_->publish(std::move(image_msg));
    return;
  }

  ++consecutive_failures_;

  if (
    static_cast<unsigned int>(frame_ret) ==
    static_cast<unsigned int>(MV_E_NODATA))
  {
    RCLCPP_WARN(
      get_logger(),
      "Image acquisition timeout (%d consecutive failures).",
      consecutive_failures_);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to get image buffer, error code: 0x%x "
      "(%d consecutive failures).",
      frame_ret,
      consecutive_failures_);
  }

  if (consecutive_failures_ < 3) {
    return;
  }

  RCLCPP_ERROR(
    get_logger(),
    "Camera appears disconnected. Starting reconnect.");

  disconnectCamera();

  while (running_) {
    if (connectCamera() && configureCamera()) {
      int grab_ret = MV_E_NODATA;

      {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        grab_ret = MV_CC_StartGrabbing(camera_handle_);
      }

      if (grab_ret == MV_OK) {
        RCLCPP_INFO(
          get_logger(),
          "Camera reconnected successfully; configuration restored.");

        consecutive_failures_ = 0;
        frame_count_ = 0;
        fps_start_time_ = std::chrono::steady_clock::now();
        break;
      }

      RCLCPP_WARN(
        get_logger(),
        "Failed to restart grabbing, error code: 0x%x",
        grab_ret);
    }

    disconnectCamera();

    RCLCPP_WARN(
      get_logger(),
      "Reconnect failed. Retrying in 1 second...");

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

bool CameraNode::connectCamera()
{
  MV_CC_DEVICE_INFO_LIST device_list{};

  const int enum_ret =
    MV_CC_EnumDevices(
    MV_USB_DEVICE | MV_GIGE_DEVICE,
    &device_list);

  if (enum_ret != MV_OK) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to enumerate cameras, error code: 0x%x",
      enum_ret);
    return false;
  }

  MV_CC_DEVICE_INFO * selected_device = nullptr;
  std::string matched_serial;
  std::string matched_ip;

  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    MV_CC_DEVICE_INFO * device = device_list.pDeviceInfo[i];
    if (device == nullptr) {
      continue;
    }

    const std::string serial = deviceSerial(device);

    if (!selected_serial_.empty()) {
      // Serial selection is supported for both USB and GigE devices.
      if (serial == selected_serial_) {
        selected_device = device;
        matched_serial = serial;
        matched_ip = deviceIp(device);
        break;
      }
    } else if (!selected_ip_.empty()) {
      // IP selection only makes sense for GigE devices.
      if (device->nTLayerType != MV_GIGE_DEVICE) {
        continue;
      }

      const std::string ip = deviceIp(device);
      if (ip == selected_ip_) {
        selected_device = device;
        matched_serial = serial;
        matched_ip = ip;
        break;
      }
    }
  }

  if (selected_device == nullptr) {
    if (!selected_serial_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Camera serial=%s is not available.",
        selected_serial_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "GigE camera ip=%s is not available.",
        selected_ip_.c_str());
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);

    const int create_ret =
      MV_CC_CreateHandle(&camera_handle_, selected_device);

    if (create_ret != MV_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to create camera handle, error code: 0x%x",
        create_ret);
      camera_handle_ = nullptr;
      return false;
    }

    const int open_ret = MV_CC_OpenDevice(camera_handle_);

    if (open_ret != MV_OK) {
      const unsigned int error_code =
        static_cast<unsigned int>(open_ret);

      if (
        error_code ==
          static_cast<unsigned int>(MV_E_RESOURCE_IN_USE) ||
        error_code ==
          static_cast<unsigned int>(MV_E_DEV_BUSY) ||
        error_code ==
          static_cast<unsigned int>(MV_E_BUSY))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to open selected camera: device is busy or already in use "
          "(error code: 0x%x).",
          error_code);
      } else if (
        error_code ==
          static_cast<unsigned int>(MV_E_ACCESS_DENIED) ||
        error_code ==
          static_cast<unsigned int>(MV_E_DEV_ACCESS_DENIED))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to open selected camera: access denied "
          "(error code: 0x%x).",
          error_code);
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to open selected camera, error code: 0x%x",
          error_code);
      }

      MV_CC_DestroyHandle(camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }
  }

  if (selected_device->nTLayerType == MV_GIGE_DEVICE) {
    RCLCPP_INFO(
      get_logger(),
      "Camera connected: type=GigE serial=%s ip=%s",
      matched_serial.c_str(),
      matched_ip.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Camera connected: type=USB serial=%s",
      matched_serial.c_str());
  }

  return true;
}

bool CameraNode::configureCamera()
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);

  if (camera_handle_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "Cannot configure: camera is not connected.");
    return false;
  }

  const std::string pixel_format =
    get_parameter("pixel_format").as_string();

  // The current ROS conversion path is implemented and tested for BayerRG8.
  if (pixel_format != "BayerRG8") {
    RCLCPP_ERROR(
      get_logger(),
      "Unsupported pixel_format: %s. Currently only BayerRG8 is supported.",
      pixel_format.c_str());
    return false;
  }

  int ret =
    MV_CC_SetEnumValueByString(
    camera_handle_, "PixelFormat", pixel_format.c_str());

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to set PixelFormat to %s, error code: 0x%x",
      pixel_format.c_str(),
      ret);
    return false;
  }

  // Free-running acquisition. Some models may not expose TriggerMode;
  // if so, continue and report the limitation.
  ret =
    MV_CC_SetEnumValueByString(
    camera_handle_, "TriggerMode", "Off");

  if (ret != MV_OK) {
    RCLCPP_WARN(
      get_logger(),
      "Could not set TriggerMode=Off (error code: 0x%x). "
      "Continuing with the camera's current trigger setting.",
      ret);
  }

  // Exposure
  MVCC_FLOATVALUE exposure_info{};
  ret =
    MV_CC_GetFloatValue(
    camera_handle_, "ExposureTime", &exposure_info);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to get ExposureTime range, error code: 0x%x",
      ret);
    return false;
  }

  const double exposure =
    get_parameter("exposure").as_double();

  if (
    exposure < exposure_info.fMin ||
    exposure > exposure_info.fMax)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Exposure %.2f us is out of range [%.2f, %.2f] us.",
      exposure,
      exposure_info.fMin,
      exposure_info.fMax);
    return false;
  }

  ret =
    MV_CC_SetEnumValueByString(
    camera_handle_, "ExposureAuto", "Off");

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to disable ExposureAuto, error code: 0x%x",
      ret);
    return false;
  }

  ret =
    MV_CC_SetFloatValue(
    camera_handle_,
    "ExposureTime",
    static_cast<float>(exposure));

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to set ExposureTime to %.2f us, error code: 0x%x",
      exposure,
      ret);
    return false;
  }

  MVCC_FLOATVALUE exposure_verify{};
  ret =
    MV_CC_GetFloatValue(
    camera_handle_, "ExposureTime", &exposure_verify);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Exposure set succeeded but readback failed, error code: 0x%x",
      ret);
    return false;
  }

  // Gain
  MVCC_FLOATVALUE gain_info{};
  ret = MV_CC_GetFloatValue(camera_handle_, "Gain", &gain_info);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to get Gain range, error code: 0x%x",
      ret);
    return false;
  }

  const double gain = get_parameter("gain").as_double();

  if (gain < gain_info.fMin || gain > gain_info.fMax) {
    RCLCPP_ERROR(
      get_logger(),
      "Gain %.2f is out of range [%.2f, %.2f].",
      gain,
      gain_info.fMin,
      gain_info.fMax);
    return false;
  }

  ret =
    MV_CC_SetEnumValueByString(camera_handle_, "GainAuto", "Off");

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to disable GainAuto, error code: 0x%x",
      ret);
    return false;
  }

  ret =
    MV_CC_SetFloatValue(
    camera_handle_, "Gain", static_cast<float>(gain));

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to set Gain to %.2f, error code: 0x%x",
      gain,
      ret);
    return false;
  }

  MVCC_FLOATVALUE gain_verify{};
  ret = MV_CC_GetFloatValue(camera_handle_, "Gain", &gain_verify);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Gain set succeeded but readback failed, error code: 0x%x",
      ret);
    return false;
  }

  // White balance is useful for the Bayer->BGR output, but failure is not
  // fatal because support varies by camera/model.
  ret =
    MV_CC_SetEnumValueByString(
    camera_handle_, "BalanceWhiteAuto", "Once");

  if (ret != MV_OK) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to set BalanceWhiteAuto=Once, error code: 0x%x",
      ret);
  }

  // Frame rate
  ret =
    MV_CC_SetBoolValue(
    camera_handle_, "AcquisitionFrameRateEnable", true);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to enable AcquisitionFrameRate, error code: 0x%x",
      ret);
    return false;
  }

  MVCC_FLOATVALUE fps_info{};
  ret =
    MV_CC_GetFloatValue(
    camera_handle_, "AcquisitionFrameRate", &fps_info);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to get AcquisitionFrameRate range, error code: 0x%x",
      ret);
    return false;
  }

  const double frame_rate =
    get_parameter("frame_rate").as_double();

  if (
    frame_rate < fps_info.fMin ||
    frame_rate > fps_info.fMax)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Frame rate %.2f FPS is out of range [%.2f, %.2f] FPS.",
      frame_rate,
      fps_info.fMin,
      fps_info.fMax);
    return false;
  }

  ret =
    MV_CC_SetFloatValue(
    camera_handle_,
    "AcquisitionFrameRate",
    static_cast<float>(frame_rate));

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to set AcquisitionFrameRate to %.2f FPS, error code: 0x%x",
      frame_rate,
      ret);
    return false;
  }

  MVCC_FLOATVALUE fps_verify{};
  ret =
    MV_CC_GetFloatValue(
    camera_handle_, "AcquisitionFrameRate", &fps_verify);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Frame rate set succeeded but readback failed, error code: 0x%x",
      ret);
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Camera configuration restored: "
    "pixel_format=%s, exposure requested=%.2f actual=%.2f us, "
    "gain requested=%.2f actual=%.2f, "
    "frame_rate requested=%.2f actual=%.2f FPS",
    pixel_format.c_str(),
    exposure,
    exposure_verify.fCurValue,
    gain,
    gain_verify.fCurValue,
    frame_rate,
    fps_verify.fCurValue);

  return true;
}

void CameraNode::disconnectCamera()
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);

  if (camera_handle_ == nullptr) {
    return;
  }

  // These cleanup calls are intentionally best-effort.
  MV_CC_StopGrabbing(camera_handle_);
  MV_CC_CloseDevice(camera_handle_);
  MV_CC_DestroyHandle(camera_handle_);
  camera_handle_ = nullptr;

  RCLCPP_WARN(
    get_logger(),
    "Camera disconnected and resources released.");
}

}  // namespace hikrobot_camera
