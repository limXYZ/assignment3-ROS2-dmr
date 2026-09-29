# hikrobot_camera

ROS 2 Humble wrapper for the HIKROBOT MVS SDK.

## Implemented behavior

- Enumerates both USB3 Vision and GigE Vision MVS cameras.
- Selects a USB/GigE camera by serial number, or a GigE camera by IPv4 address.
- Reports missing devices and distinguishes common busy/in-use and access-denied open failures.
- Publishes `sensor_msgs/msg/Image` using `rclcpp::SensorDataQoS()` (Best Effort).
- Current image path: camera `BayerRG8` -> OpenCV BGR -> ROS `bgr8`.
- Publishes width, height, encoding, endianness, step, full image data, `frame_id`, and timestamp.
- Supports exposure, gain and frame-rate parameters with device-range validation and readback.
- Exposure, gain and frame rate can be changed at runtime.
- Device selection, topic, frame ID, pixel format and registry are startup-only.
- Detects repeated acquisition failures, releases the camera, retries connection, reapplies valid configuration and restarts acquisition.
- Releases SDK resources on shutdown.
- Logs configured/read-back frame rate separately from measured receive/pipeline FPS.

## Parameters

`selected_serial` and `selected_ip` are mutually exclusive. Exactly one must be non-empty.

- `known_serials`: list of registered/known camera serial numbers. This is informational and does not prevent using an unregistered serial.
- `selected_serial`: selects either a USB or GigE camera by serial number.
- `selected_ip`: selects a GigE camera by IPv4 address.
- `image_topic`: ROS image topic, default `/image_raw`.
- `frame_id`: ROS image header frame ID, default `camera`.
- `pixel_format`: currently only `BayerRG8` is accepted because that is the implemented/tested conversion path.
- `exposure`: exposure time in microseconds. The node queries the camera's current valid range before applying it.
- `gain`: camera-native gain value. The node queries the camera's current valid range before applying it.
- `frame_rate`: configured acquisition frame rate in FPS. The node queries the camera's current valid range before applying it.

Out-of-range exposure/gain/frame-rate updates are rejected. Startup configuration failure prevents acquisition from starting. Runtime updates are written to the device and then read back.

## Timestamp semantics

`Image.header.stamp` is generated with the ROS node clock (`this->now()`) after the SDK frame has been received and converted. It is therefore a host-side ROS timestamp, **not** the camera hardware exposure timestamp.

`Image.header.frame_id` comes from the `frame_id` parameter.

For the published `bgr8` image:

- `height` and `width` come from the received frame.
- `encoding = "bgr8"`.
- `is_bigendian = false`.
- `step = width * 3`.
- `data.size() = height * step`.

## Frame-rate semantics

Three values should not be confused:

1. `frame_rate` is the requested camera acquisition rate.
2. The configuration log's `actual` value is the value read back from the camera node `AcquisitionFrameRate`.
3. `Actual receive/pipeline FPS` is measured by this process while receiving, converting and publishing frames. It can be lower than the configured camera rate because Bayer-to-BGR conversion, memory copies, CPU scheduling and ROS publication consume time.

`ros2 topic hz /image_raw` measures subscriber-observed ROS topic throughput and can differ again.

## Launch

Build and source the workspace, then:

```zsh
export ROS_DOMAIN_ID=42
ros2 launch hikrobot_camera camera.launch.py
```

To use a different YAML file:

```zsh
ros2 launch hikrobot_camera camera.launch.py params_file:=/absolute/path/to/camera.yaml
```

## Runtime parameter examples

```zsh
ros2 param set /hikrobot_camera exposure 8000.0
ros2 param set /hikrobot_camera gain 5.0
ros2 param set /hikrobot_camera frame_rate 50.0
```

Startup-only parameters are intentionally rejected if changed at runtime.

## RViz2

Add an **Image** display, select `/image_raw`, and use **Best Effort** reliability with **Volatile** durability to match the sensor-data publisher.

## Reconnection

After three consecutive acquisition failures, the node treats the camera as disconnected. It releases SDK resources and retries every second. Once the selected camera is available again, the node reopens it, reapplies pixel format, trigger mode (when supported), exposure, gain, white balance and frame rate, then restarts grabbing.

## Notes / limitations

- The Bayer conversion used here is `cv::COLOR_BayerBG2BGR`, selected from testing with the current camera/SDK image output.
- GigE enumeration and selection are implemented from the installed MVS SDK structures (`stGigEInfo`, `nCurrentIp`, `chSerialNumber`). If no physical GigE camera is available during testing, document that the GigE path was compile-tested but not hardware-validated.
- The current design performs acquisition, Bayer conversion and ROS publication in one capture thread. This is simple and robust, but conversion/publication can limit measured throughput below the configured camera FPS.
