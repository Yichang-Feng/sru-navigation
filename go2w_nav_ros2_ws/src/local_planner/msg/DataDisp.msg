# 原规划诊断接口：header.stamp 为本轮状态机的 ROS 时间。
std_msgs/Header header

# 保留原有通用诊断槽位；当前没有为 a～e 赋予具体指标，默认值为 0。
float64 a
float64 b
float64 c
float64 d
float64 e
