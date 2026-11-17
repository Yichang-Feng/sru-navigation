// 重规划初值的有界采样：拒绝退化数据、过期轨迹和过大点列，避免循环细分耗尽内存。
#ifndef LOCAL_PLANNER_INITIAL_PATH_SAMPLING_H
#define LOCAL_PLANNER_INITIAL_PATH_SAMPLING_H

#include <algorithm>
#include <cmath>
#include <vector>
#include <local_planner/plan_manage/plan_container.hpp>

namespace local_planner
{

// 先计算有界区间数，再按整数索引采样，避免浮点步长下溢或 t += dt 不再前进。
inline bool initialSampleSegments(double duration, double dt, std::size_t &segments)
{
  if (!std::isfinite(duration) || duration <= 0.0 || !std::isfinite(dt) ||
      dt < UniformBspline::kMinSampleInterval)
    return false;
  const double count = std::ceil(duration / dt);
  if (!std::isfinite(count) || count < 1.0 ||
      count >= static_cast<double>(UniformBspline::kMaxParameterizationPoints))
    return false;
  segments = static_cast<std::size_t>(count);
  return true;
}

// 多项式初值最多细分 16 次；只有点数、间距、有限性全部满足时才交给 QR 拟合。
inline bool samplePolynomialInitialPath(
    PolynomialTraj &trajectory, double duration, double spacing, double &dt,
    std::vector<Eigen::Vector3d> &points)
{
  points.clear();
  if (!std::isfinite(spacing) || spacing <= 0.0)
    return false;
  for (int attempt = 0; attempt < 16; ++attempt, dt /= 1.5)
  {
    std::size_t segments = 0;
    if (!initialSampleSegments(duration, dt, segments))
      return false;
    const double actual_dt = duration / static_cast<double>(segments);
    if (actual_dt < UniformBspline::kMinSampleInterval)
      return false;
    std::vector<Eigen::Vector3d> sampled;
    sampled.reserve(segments + 1);
    bool too_far = false;
    for (std::size_t i = 0; i <= segments; ++i)
    {
      const Eigen::Vector3d point = trajectory.evaluate(
          duration * (static_cast<double>(i) / segments));
      if (!point.allFinite())
        return false;
      if (!sampled.empty() && (point - sampled.back()).norm() > spacing * 1.5)
      {
        too_far = true;
        break;
      }
      sampled.push_back(point);
    }
    if (!too_far && sampled.size() >= 7)
    {
      points.swap(sampled);
      dt = actual_dt;
      return true;
    }
  }
  return false;
}

// 沿折线弧长直接确定采样数量；跳过重复点，不使用 size()-2 或无上限步长缩减。
inline bool resampleInitialPolyline(
    const std::vector<Eigen::Vector3d> &input, double spacing,
    std::vector<Eigen::Vector3d> &output)
{
  output.clear();
  if (input.size() < 2 || input.size() > UniformBspline::kMaxParameterizationPoints ||
      !std::isfinite(spacing) || spacing <= 0.0)
    return false;
  std::vector<Eigen::Vector3d> points;
  std::vector<double> arc;
  for (const auto &point : input)
  {
    if (!point.allFinite())
      return false;
    if (points.empty())
    {
      points.push_back(point);
      arc.push_back(0.0);
      continue;
    }
    const double length = (point - points.back()).norm();
    if (!std::isfinite(length))
      return false;
    if (length <= 1e-9)
      continue;
    const double distance = arc.back() + length;
    if (!std::isfinite(distance))
      return false;
    points.push_back(point);
    arc.push_back(distance);
  }
  if (points.size() < 2)
    return false;
  const double segments = std::max(6.0, std::ceil(arc.back() / spacing));
  if (!std::isfinite(segments) || segments >= UniformBspline::kMaxParameterizationPoints)
    return false;
  const auto count = static_cast<std::size_t>(segments);
  output.reserve(count + 1);
  std::size_t segment = 1;
  for (std::size_t i = 0; i <= count; ++i)
  {
    const double distance = arc.back() * (static_cast<double>(i) / count);
    while (segment + 1 < arc.size() && arc[segment] < distance)
      ++segment;
    const double ratio = std::clamp(
        (distance - arc[segment - 1]) / (arc[segment] - arc[segment - 1]), 0.0, 1.0);
    output.push_back((1.0 - ratio) * points[segment - 1] + ratio * points[segment]);
  }
  output.front() = points.front();
  output.back() = points.back();
  return true;
}

// 复用旧轨迹前验证时间范围，包含旧轨迹终点；无法复用时由调用方改用多项式初值。
inline bool samplePreviousInitialPath(
    LocalTrajData &previous, double elapsed, double dt, const Eigen::Vector3d &target,
    const Eigen::Vector3d &target_velocity, double max_velocity, double spacing,
    std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &derivatives)
{
  points.clear();
  derivatives.clear();
  if (previous.traj_id_ <= 0 || !std::isfinite(elapsed) || elapsed < 0.0 ||
      !std::isfinite(previous.duration_) || elapsed >= previous.duration_ ||
      !target.allFinite() || !target_velocity.allFinite() ||
      !std::isfinite(max_velocity) || max_velocity <= 0.0)
    return false;
  std::size_t segments = 0;
  if (!initialSampleSegments(previous.duration_ - elapsed, dt, segments))
    return false;
  std::vector<Eigen::Vector3d> sampled;
  sampled.reserve(segments + 1);
  for (std::size_t i = 0; i <= segments; ++i)
  {
    const double t = i == segments ? previous.duration_ : std::min(previous.duration_, elapsed + i * dt);
    const Eigen::Vector3d point = previous.position_traj_.evaluateDeBoorT(t);
    if (!point.allFinite())
      return false;
    sampled.push_back(point);
  }
  const double extension_time = (sampled.back() - target).norm() / max_velocity * 2.0;
  if (!std::isfinite(extension_time))
    return false;
  if (extension_time > dt)
  {
    if (!initialSampleSegments(extension_time, dt, segments) ||
        segments > UniformBspline::kMaxParameterizationPoints - sampled.size())
      return false;
    auto extension = PolynomialTraj::one_segment_traj_gen(
        sampled.back(), previous.velocity_traj_.evaluateDeBoorT(previous.duration_),
        previous.acceleration_traj_.evaluateDeBoorT(previous.duration_),
        target, target_velocity, Eigen::Vector3d::Zero(), extension_time);
    for (std::size_t i = 1; i <= segments; ++i)
      sampled.push_back(extension.evaluate(i == segments ? extension_time : std::min(extension_time, i * dt)));
  }
  else
  {
    if (sampled.size() >= UniformBspline::kMaxParameterizationPoints)
      return false;
    sampled.push_back(target);
  }
  if (!resampleInitialPolyline(sampled, spacing, points))
    return false;
  derivatives = {previous.velocity_traj_.evaluateDeBoorT(elapsed), target_velocity,
                 previous.acceleration_traj_.evaluateDeBoorT(elapsed), Eigen::Vector3d::Zero()};
  for (const auto &derivative : derivatives)
    if (!derivative.allFinite())
    {
      points.clear();
      derivatives.clear();
      return false;
    }
  return true;
}

}  // namespace local_planner
#endif  // LOCAL_PLANNER_INITIAL_PATH_SAMPLING_H
