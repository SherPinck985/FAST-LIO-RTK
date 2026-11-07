# FAST_LIO_RTK

基于 FAST-LIO2 的激光雷达、IMU 与 RTK 融合建图项目。在原有迭代卡尔曼滤波框架中加入 RTK 位置观测，通过天线杆臂补偿和 ENU 坐标对齐，为建图提供绝对位置约束。关闭 RTK 后可按原 FAST-LIO 流程运行，ROS 包名仍为 `fast_lio`。

## 环境与编译

依赖 ROS 1 / catkin、PCL >= 1.8、Eigen >= 3.3.4、Python 开发库、`livox_ros_driver` 和 GeographicLib。编译前请先安装并 source 雷达驱动工作空间。

```bash
sudo apt install libgeographic-dev
```

将本仓库放入 `~/catkin_ws/src/FAST_LIO_RTK`，然后执行：

```bash
cd ~/catkin_ws/src/FAST_LIO_RTK
git submodule update --init --recursive
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行（以 Livox MID360 为例）

修改 `config/mid360.yaml` 中的雷达、IMU 话题和标定外参，以及以下 RTK 配置：

- `rtk/topic`：RTK 输入话题，默认 `/rtk/fix`，消息类型为 `sensor_msgs/NavSatFix`。
- `rtk/extrinsic_T`：IMU 原点指向天线参考点的向量，在 IMU 坐标系中表达，单位为米。
- `rtk/extrinsic_R`：天线坐标系到 IMU 坐标系的旋转矩阵，按行展开；位置融合主要使用平移杆臂。
- `rtk/align_distance`、`rtk/align_min_samples`：对齐所需的水平移动距离和最少配对数，默认 5 米、10 组。
- `rtk/max_std`：允许的位置标准差上限，默认 1 米。

开启 RTK 融合：

```bash
roslaunch fast_lio mapping_mid360.launch rtk:=true
```

另行启动雷达、IMU 和 RTK 驱动，或播放包含这些话题的 rosbag。RTK 输入需提供有效的测量时间戳、经纬度、椭球高及位置协方差，传感器时间应同步，`common/time_sync_en` 保持 `false`。

启动时先保持静止完成 IMU 初始化，再水平移动以完成 ENU 对齐。对齐成功后，`/cloud_registered` 发布 `enu` 坐标系下的降采样点云；RViz 的 Fixed Frame 需设为 `enu`。当前 RTK 模式关闭里程计、路径、机体系点云发布及 PCD 保存。

仅运行激光惯性建图：

```bash
roslaunch fast_lio mapping_mid360.launch rtk:=false
```

## 致谢

本项目基于 [FAST-LIO](https://github.com/hku-mars/FAST_LIO)、[ikd-Tree](https://github.com/hku-mars/ikd-Tree) 和 [IKFoM](https://github.com/hku-mars/IKFoM) 开发，感谢原作者的开源贡献。许可证见 [LICENSE](LICENSE)。
