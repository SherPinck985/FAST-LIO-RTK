# 帧内 IMU 轨迹记录：由 common_lib.h::set_pose6d 构造，供 ImuProcess 去畸变使用。
# 虽由 ROS 消息生成器生成类型，当前工程没有发布独立 Pose6D 轨迹话题。

# 相对当前扫描起始参考时刻的时间，秒；不是点云 curvature 使用的毫秒。
float64  offset_time
# 轨迹段使用的世界系加速度，包含当前重力补偿。
float64[3] acc
# 去偏后的 IMU 系角速度，供段内旋转补偿。
float64[3] gyr
# IMU 原点的世界系速度。
float64[3] vel
# IMU 原点的世界系位置。
float64[3] pos
# IMU -> 世界的旋转矩阵，9 个元素按行保存。
float64[9] rot
