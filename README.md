# hikrobot_camera

HIKROBOT MVS SDK 的 ROS 2 Humble 封装。

## 已实现的功能

-枚举 USB3 Vision 和 GigE Vision 两种 MVS 相机。

按序列号选择 USB/GigE 相机，或按 IPv4 地址选择 GigE 相机。

报告未发现设备的情况，并区分常见的设备被占用和访问被拒绝错误。

使用 rclcpp::SensorDataQoS()（Best Effort）发布 sensor_msgs/msg/Image。

当前图像路径：相机 BayerRG8 → OpenCV BGR → ROS bgr8。

发布宽度、高度、编码、字节序、步长、完整图像数据、frame_id 和时间戳。

支持曝光、增益和帧率参数，带设备范围校验和读回验证。

曝光、增益和帧率可在运行时修改。

设备选择、话题、frame ID、像素格式和注册表仅在启动时设置。

检测到连续采集失败后，释放相机、重试连接、重新应用有效配置并重新开始采集。

关闭时释放 SDK 资源。

分别记录配置的/读回的帧率和实测的接收/流水线 FPS。

## 参数

selected_serial 和 selected_ip 互斥，必须恰好有一个非空。

known_serials：已注册/已知相机序列号列表。这是信息性的，不会阻止使用未注册的序列号。

selected_serial：按序列号选择 USB 或 GigE 相机。

selected_ip：按 IPv4 地址选择 GigE 相机。

image_topic：ROS 图像话题，默认 /image_raw。

frame_id：ROS 图像 header 的 frame ID，默认 camera。

pixel_format：目前仅接受 BayerRG8，因为这是已实现并测试过的转换路径。

exposure：曝光时间，单位微秒。节点在应用前会查询相机当前的有效范围。

gain：相机原生增益值。节点在应用前会查询相机当前的有效范围。

frame_rate：配置的采集帧率，单位 FPS。节点在应用前会查询相机当前的有效范围。

超出范围的曝光/增益/帧率更新会被拒绝。启动时配置失败会阻止采集启动。运行时更新会先写入设备，然后读回验证。

## 时间戳语义

Image.header.stamp 在 SDK 帧接收并转换完成后，用 ROS 节点时钟（this->now()）生成。因此它是主机侧的 ROS 时间戳，不是相机硬件的曝光时间戳。

Image.header.frame_id 来自 frame_id 参数。

对于发布的 bgr8 图像：

height 和 width 来自接收到的帧。

encoding = "bgr8"。

is_bigendian = false。

step = width * 3。

data.size() = height * step。

## 帧率语义

Three values should not be confused:

1.frame_rate 是请求的相机采集帧率。

2.配置日志中的 actual 值是从相机节点 AcquisitionFrameRate 读回的值。

3.Actual receive/pipeline FPS 是本进程在接收、转换和发布帧期间测得的。它可能低于配置的相机帧率，因为 Bayer 转 BGR、内存拷贝、CPU 调度和 ROS 发布都会消耗时间。

ros2 topic hz /image_raw 测量的是订阅者观察到的 ROS 话题吞吐量，可能又不一样。

## 启动

构建并 source 工作空间后：

```zsh
export ROS_DOMAIN_ID=42
ros2 launch hikrobot_camera camera.launch.py
```

使用不同的 YAML 文件：

```zsh
ros2 launch hikrobot_camera camera.launch.py params_file:=/absolute/path/to/camera.yaml
```

## 运行时参数示例

```zsh
ros2 param set /hikrobot_camera exposure 8000.0
ros2 param set /hikrobot_camera gain 5.0
ros2 param set /hikrobot_camera frame_rate 50.0
```

启动时-only 的参数在运行时修改会被有意拒绝。

## RViz2

添加一个 Image 显示，选择 /image_raw，并使用 Best Effort 可靠性和 Volatile 持久性，以匹配传感器数据发布者。

## 重连机制

连续三次采集失败后，节点将相机视为断开。它释放 SDK 资源并每秒重试。一旦所选相机再次可用，节点重新打开它，重新应用像素格式、触发模式（如果支持）、曝光、增益、白平衡和帧率，然后重新开始采集。

## 注意事项 / 限制

- 这里使用的 Bayer 转换是 cv::COLOR_BayerBG2BGR，是根据当前相机/SDK 图像输出测试后选择的。

GigE 枚举和选择是基于已安装的 MVS SDK 结构体（stGigEInfo、nCurrentIp、chSerialNumber）实现的。如果测试时没有可用的物理 GigE 相机，请说明 GigE 路径已通过编译测试但未经过硬件验证。

当前设计在同一个采集线程中执行采集、Bayer 转换和 ROS 发布。这简单且健壮，但转换/发布可能限制实测吞吐量低于配置的相机帧率。

## 作者：邓铭儒
## 学号：2264413009
## 日期：2026-9-26