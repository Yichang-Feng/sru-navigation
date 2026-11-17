#ifndef LOCAL_PLANNER_BSPLINE_PATH_H
#define LOCAL_PLANNER_BSPLINE_PATH_H

#include <cmath>
#include <stdexcept>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/path.hpp>
#include <local_planner/bspline_opt/uniform_bspline.h>

namespace local_planner
{

/**
 * 将已经优化完成的 B 样条采样成几何路径，供 path_follow 跟踪。
 * 控制点不是轨迹上的点，必须调用 De Boor 求值；采样保留完整起点和终点。
 * sample_dt 单位为秒，仅决定采样密度；Path 不携带可执行的速度、加速度约束。
 * 位置使用规划坐标系，四元数仅表达水平切线的偏航角，不额外改变路径高度。
 */
inline nav_msgs::msg::Path sampleBsplinePath(
    UniformBspline &spline, const std::string &frame_id,
    const builtin_interfaces::msg::Time &stamp, double sample_dt)
{
  if (frame_id.empty() || !std::isfinite(sample_dt) || sample_dt <= 0.0)
    throw std::invalid_argument("local path frame and sample interval must be valid");

  const double duration = spline.getTimeSum();
  const double count = std::ceil(duration / sample_dt);
  // 限制采样开销，防止错误参数使节点申请过大的路径数组。
  if (!std::isfinite(duration) || duration <= 0.0 || !std::isfinite(count) || count > 10000.0)
    throw std::invalid_argument("local path duration or sample count is invalid");
  const auto segments = std::max<std::size_t>(1, static_cast<std::size_t>(count));
  auto velocity = spline.getDerivative();
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(segments + 1);
  for (std::size_t i = 0; i <= segments; ++i)
  {
    const double t = std::min(duration, static_cast<double>(i) * sample_dt);
    const Eigen::Vector3d point = spline.evaluateDeBoorT(t);
    if (!point.allFinite())
      throw std::invalid_argument("local path contains a non-finite position");
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    path.poses.push_back(pose);
  }

  double last_yaw = 0.0;
  for (std::size_t i = 0; i < path.poses.size(); ++i)
  {
    const double t = std::min(duration, static_cast<double>(i) * sample_dt);
    Eigen::Vector3d tangent = velocity.evaluateDeBoorT(t);
    if (!tangent.allFinite())
      throw std::invalid_argument("local path contains a non-finite tangent");
    // 起止速度可能为零，此时用相邻采样点估计切线；完全静止时使用单位四元数。
    if (tangent.head<2>().squaredNorm() < 1e-12)
    {
      const auto &a = path.poses[i == 0 ? i : i - 1].pose.position;
      const auto &b = path.poses[i + 1 < path.poses.size() ? i + 1 : i].pose.position;
      tangent = Eigen::Vector3d(b.x - a.x, b.y - a.y, b.z - a.z);
    }
    if (tangent.head<2>().squaredNorm() >= 1e-12)
      last_yaw = std::atan2(tangent.y(), tangent.x());
    auto &q = path.poses[i].pose.orientation;
    q.z = std::sin(last_yaw / 2.0);
    q.w = std::cos(last_yaw / 2.0);
  }
  return path;
}

}  // namespace local_planner
#endif  // LOCAL_PLANNER_BSPLINE_PATH_H
