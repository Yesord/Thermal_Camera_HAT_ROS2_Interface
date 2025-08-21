# 简单介绍
本项目是基于微雪的 Thermal Camera HAT 的 USB 版本的在 ARM64 板子上采样的 ROS2 接口。已在 jetson orin nx 测试成功。

# 功能
1. 发布可视化热分布图 topic /thermal_camera/image_raw
2. 发布视场内最高温度和最低温度 /thermal_camera/min_max_temp

# 快速开始
1. 配置相关依赖

参考[网址](https://www.waveshare.net/wiki/Thermal_Camera_HAT)

or 执行 requirements.txt

```bash
pip install -r requirements.txt
```

2. 构建节点
```bash
cd dev_ws
colcon build
```

3. 运行节点
```bash
cd dev_ws
source install/setup.bash
ros2 launch src/thermal_camera_node/launch/thermal_camera_launch.py
```
or

```bash
cd dev_ws
source install/setup.bash
ros2 run thermal_camera_node thermal_camera_publisher
```

or 

```bash
sudo chmod +x ./start_thermal_camera_node.sh
./start_thermal_camera_node.sh
```

# 参数配置
在 [launch](src/thermal_camera_node/launch/thermal_camera_launch.py) 文件中可以更改。

| 参数名                                      | 默认值    | 说明                       |
|---------------------------------------------|----------|----------------------------|
| image_width                                | 80       | 图像宽度                   |
| image_height                               | 62       | 图像高度                   |
| frame_rate                                 | 25       | 帧率（Hz）                 |
| encoding                                   | bgr8     | ROS图像编码格式            |
| temporal_filter_enable                     | True     | 是否启用时域滤波           |
| rolling_average_filter_enable              | False    | 是否启用滑动平均滤波       |
| median_filter_enable                       | False    | 是否启用中值滤波           |
| median_filter_ksize5_enable                | False    | 是否启用5x5中值滤波        |
| temporal_filter_strength                   | 85       | 时域滤波强度               |
| offset_corr                                | 0.0      | 温度偏移修正（单位K）      |
| sens_factor                                | 100      | 灵敏度因子                 |
| stream_enable                              | True     | 是否连续采集               |
| start_with_header_enable                   | True     | 是否带帧头                 |
| rolling_average_temperature_minimum_frame_size | 10   | 最小温度滑动窗口帧数       |
| rolling_average_temperature_maximum_frame_size | 10   | 最大温度滑动窗口帧数       |
| use_opencv_filter                          | True     | 是否使用OpenCV滤波         |

# rviz2 测试

![rviz2 测试](https://imgbed.yesord.top/file/github/1753774880194_微信截图_20250729153847.png)
