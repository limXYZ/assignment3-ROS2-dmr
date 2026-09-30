# RoboMaster assignment3 ROS2
这份仓库提供一个基础工程，供你在 Ubuntu 22.04 / ROS 2 Humble 上，基于海康机器人 MVS SDK 完成相机功能包。

## 开始

1. 点击 GitHub 页面右上角的 **Fork**，将仓库复制到你的账号下。
2. 在你的 Fork 页面点击 **Code**，复制地址并克隆到本地：

   ```bash
   # 将下面的地址替换为你的 Fork 地址
   git clone https://github.com/limXYZ/assignment3-ROS2-dmr
   cd robomaster-camera-assignment
   ```

3. 阅读 [ROS 2 教程](docs/ROS2Tutorial.md) 和 [作业要求](docs/assignment.md)，按下面的步骤构建并启动工程。
4. 在自己的仓库中完成开发，提交并推送改动，最后提交你的 GitHub 仓库链接。

[AGENTS.md](AGENTS.md) 用于约束 AI 助手的帮助范围：你可以用 AI 理解概念和分析问题，核心实现需要自己完成。

## 仓库结构

```text
robomaster-camera-assignment/          # 同时作为 colcon 工作空间
├── AGENTS.md                         # AI 助教规范
├── README.md
├── docs/ROS2Tutorial.md              # ROS 2 教程
├── docs/assignment.md                # 作业要求
└── src/hikrobot_camera/              # ROS 2 功能包
    ├── package.xml                   # 包信息与依赖
    ├── CMakeLists.txt                # 构建与安装配置
    ├── include/hikrobot_camera/
    │   └── camera_node.hpp          # 节点声明
    ├── src/
    │   ├── main.cpp                 # 程序入口
    │   └── camera_node.cpp          # 在这里开始实现
    ├── launch/camera.launch.py       # 启动文件
    ├── config/camera.yaml           # 参数配置
    ├── cmake/                       # 可按需添加 SDK 查找模块
    └── test/                        # 可按需添加测试
```

## 环境与依赖
1. ROS 2 Humble
先安装 ROS 2 Humble 与开发工具，确保 `ros2`、`colcon` 和 `rosdep` 可用。
```bash
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs \
                 ros-humble-launch ros-humble-launch-ros \
                 ros-humble-ament-index-python
```
2. 海康 MVS SDK（厂商依赖，需手动安装）
从 海康机器人下载中心 下载 Linux 版 MVS SDK 的 .deb 文件，然后
```bash
sudo dpkg -i MVS_SDK_xxx.deb
sudo apt install -f   # 若提示依赖缺失
ls /opt/MVS/          # 确认安装成功
```
安装后应存在：

头文件：/opt/MVS/include/MvCameraControl.h

库文件：/opt/MVS/lib/64/libMvCameraControl.so

CMakeLists.txt 中已通过 add_library(mvs_sdk SHARED IMPORTED) 手动链接该 SDK，并通过 INSTALL_RPATH 让可执行文件自动找到 .so，无需手动设置 LD_LIBRARY_PATH。

## 启动

构建并 source 工作空间后：

```bash
export ROS_DOMAIN_ID=42
ros2 launch hikrobot_camera camera.launch.py
```
请注意：连接相机时要在assignment3-ROS2/src/hikrobot_camera/config/camera.yaml中添加相机serial或ip

使用不同的 YAML 文件：

```bash
ros2 launch hikrobot_camera camera.launch.py params_file:=/absolute/path/to/camera.yaml
```


## 在 RViz2 中查看图像
```bash
rviz2
```
1.点击 Add → 选 Image → OK
2.展开 Image → Topic 选 /image_raw
3.QoS → Reliability 改为 Best Effort
4.即可看到相机画面

## 完成与提交

从 `camera_node.cpp` 的 TODO 开始，按 [作业要求](docs/assignment.md) 完成相机功能。你可以增加源文件或 SDK 封装类，并相应更新构建配置。

完成后：

- 更新 README，说明 SDK 及依赖的安装方式、如何编译启动、有哪些可配置参数。如果有未完成的功能或已知问题，简单注明即可。
- 将源代码、Launch 和参数配置推送到你的 Fork, 然后提交仓库链接到 2719850558@qq.com，格式为：第三次作业-班级-姓名（第三次作业-自动化2305-周湛昊）


---

## 在这里解释你的项目
1. 如何编译：
# 安装 ROS 2 依赖
```bash
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs \
                 ros-humble-launch ros-humble-launch-ros \
                 ros-humble-ament-index-python
```
# 安装 MVS SDK（见上文"环境与依赖"）

# 编译

在新终端中进入仓库根目录，运行：

```bash
source /opt/ros/humble/setup.bash   # 使用 Zsh 时改为 setup.zsh
# 仅当系统尚未初始化 rosdep 时执行一次：sudo rosdep init
colcon build --packages-select hikrobot_camera
source install/setup.bash           # Zsh 时改为 setup.zsh
```
编译成功后，install/hikrobot_camera/lib/hikrobot_camera/camera_node 即为可执行文件。

2. 运行方式：
```bash
ros2 launch hikrobot_camera camera.launch.py
```


# 已实现的功能

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

# 参数

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

## 运行时参数修改示例
```bash
ros2 param set /hikrobot_camera exposure 8000.0
ros2 param set /hikrobot_camera gain 5.0
ros2 param set /hikrobot_camera frame_rate 50.0
```

启动时-only 的参数在运行时修改会被有意拒绝。

# 时间戳语义

Image.header.stamp 在 SDK 帧接收并转换完成后，用 ROS 节点时钟（this->now()）生成。因此它是主机侧的 ROS 时间戳，不是相机硬件的曝光时间戳。

Image.header.frame_id 来自 frame_id 参数。

对于发布的 bgr8 图像：

height 和 width 来自接收到的帧。

encoding = "bgr8"。

is_bigendian = false。

step = width * 3。

data.size() = height * step。

# 帧率语义

Three values should not be confused:

1.frame_rate 是请求的相机采集帧率。

2.配置日志中的 actual 值是从相机节点 AcquisitionFrameRate 读回的值。

3.Actual receive/pipeline FPS 是本进程在接收、转换和发布帧期间测得的。它可能低于配置的相机帧率，因为 Bayer 转 BGR、内存拷贝、CPU 调度和 ROS 发布都会消耗时间。

ros2 topic hz /image_raw 测量的是订阅者观察到的 ROS 话题吞吐量，可能又不一样。
# 重连机制

连续三次采集失败后，节点将相机视为断开。它释放 SDK 资源并每秒重试。一旦所选相机再次可用，节点重新打开它，重新应用像素格式、触发模式（如果支持）、曝光、增益、白平衡和帧率，然后重新开始采集。

# 注意事项 / 限制

- 连接相机时要在assignment3-ROS2/src/hikrobot_camera/config/camera.yaml中添加相机serial或ip，暂时未实现在终端中匹配并回写到camera.yaml的功能

- 这里使用的 Bayer 转换是 cv::COLOR_BayerBG2BGR，是根据当前相机/SDK 图像输出测试后选择的。

- GigE 枚举和选择是基于已安装的 MVS SDK 结构体（stGigEInfo、nCurrentIp、chSerialNumber）实现的。如果测试时没有可用的物理 GigE 相机，请说明 GigE 路径已通过编译测试但未经过硬件验证。

- 当前设计在同一个采集线程中执行采集、Bayer 转换和 ROS 发布。这简单且健壮，但转换/发布可能限制实测吞吐量低于配置的相机帧率。


## 作者：邓铭儒
## 学号：2264413009
## 日期：2026-9-29