# Astra Pro Plus RGB Camera

独立 ROS 2 Humble C++ RGB 功能包。它直接通过 OpenCV/V4L2 读取 Astra Pro Plus 的
`2bc5:050f` UVC 模组，不依赖 Orbbec SDK、OpenNI 或官方相机 ROS 包。

采集与发布运行在不同线程，发布端只取最新一帧，避免慢订阅者导致相机采集积压。

## 依赖与构建

```bash
sudo apt install libopencv-dev ros-humble-rclcpp ros-humble-sensor-msgs

cd ~/gu_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select astra_rgb_camera
source install/setup.bash
```

## 启动

```bash
ros2 launch astra_rgb_camera astra_rgb_camera.launch.py
```

节点会自动查找 USB ID `2bc5:050f` 对应的 `/dev/video*`，并发布：

```text
/front_camera/image_raw  sensor_msgs/msg/Image  bgr8
```

查看图像：

```bash
ros2 run rqt_image_view rqt_image_view /front_camera/image_raw
```

需要固定视频节点或修改分辨率时：

```bash
ros2 launch astra_rgb_camera astra_rgb_camera.launch.py \
  device:=/dev/video0 width:=640 height:=480 fps:=30.0
```

默认使用 `reliable` QoS。只用于图像查看或允许丢帧时，可降低传输开销：

```bash
ros2 launch astra_rgb_camera astra_rgb_camera.launch.py \
  fps:=30.0 qos_reliability:=best_effort
```

如果提示无权限，确认当前用户能读写对应 `/dev/video*`，或将用户加入 `video` 组后重新登录。
