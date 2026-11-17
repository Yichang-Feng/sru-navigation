# 优化后的 B 样条轨迹；控制点位于 grid_map.frame_id 指定的规划坐标系。
# 本消息没有 Header；跟踪端必须与规划器约定相同坐标系。
# 样条次数（当前为 3，即三次 B 样条）。
int32 order
# 单调递增的局部轨迹编号；停车轨迹也使用新编号。
int64 traj_id
# ROS 时间：该轨迹的起始执行时间。
builtin_interfaces/Time start_time

# B 样条节点向量，单位 s；求值必须结合次数和控制点，不能把控制点当路径点。
float64[] knots
# 三维位置控制点，单位 m。
geometry_msgs/Point[] pos_pts

# 保留的偏航样条字段；当前规划发布流程没有填充，路径朝向由位置切线推导。
float64[] yaw_pts
# 偏航节点间隔，单位 s；未填充时为 0。
float64 yaw_dt
