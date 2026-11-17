// B 样条采样单测：检查真正曲线位置、终点、朝向、坐标系和异常参数。
#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include <local_planner/traj_utils/bspline_path.h>

TEST(BsplinePath, SamplesCurveWithExactEndpointsAndFrame)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i)
  {
    points(0, i) = i;
    points(1, i) = 2.0 * i;
    points(2, i) = 0.5 + 0.1 * i;
  }
  local_planner::UniformBspline spline(points, 3, 0.5);
  // 使用非均匀节点，验证转换遵循真实节点向量，而不是控制点折线。
  Eigen::VectorXd knots = spline.getKnot();
  knots(4) += 0.1;
  spline.setKnot(knots);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  auto path = local_planner::sampleBsplinePath(spline, "world", stamp, 0.17);
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_EQ(path.header.frame_id, "world");
  EXPECT_EQ(path.header.stamp.sec, 12);
  const auto end = spline.evaluateDeBoorT(spline.getTimeSum());
  EXPECT_NEAR(path.poses.back().pose.position.x, end.x(), 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.z, end.z(), 1e-9);
  EXPECT_NE(path.poses.front().pose.position.x, points(0, 0));
  for (std::size_t i = 0; i < path.poses.size(); ++i)
  {
    const auto &pose = path.poses[i];
    const auto expected = spline.evaluateDeBoorT(std::min(i * 0.17, spline.getTimeSum()));
    EXPECT_NEAR(pose.pose.position.x, expected.x(), 1e-9);
    EXPECT_NEAR(pose.pose.position.y, expected.y(), 1e-9);
    EXPECT_NEAR(pose.pose.position.z, expected.z(), 1e-9);
    EXPECT_EQ(pose.header, path.header);
    const auto &q = pose.pose.orientation;
    EXPECT_NEAR(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, 1e-9);
    EXPECT_NEAR(2.0 * std::atan2(q.z, q.w), std::atan2(2.0, 1.0), 1e-9);
  }
}

TEST(BsplinePath, ZeroTangentKeepsValidOrientation)
{
  auto points = Eigen::MatrixXd::Ones(3, 6).eval();
  local_planner::UniformBspline spline(points, 3, 0.5);
  auto path = local_planner::sampleBsplinePath(spline, "world", {}, 0.2);
  for (const auto &pose : path.poses)
  {
    EXPECT_NEAR(pose.pose.position.x, 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);
  }
}

TEST(BsplinePath, RejectsInvalidSamplingAndNonfinitePositions)
{
  auto points = Eigen::MatrixXd::Zero(3, 6).eval();
  local_planner::UniformBspline spline(points, 3, 0.5);
  EXPECT_THROW(local_planner::sampleBsplinePath(spline, "", {}, 0.05), std::invalid_argument);
  for (double dt : {0.0, -0.1, 1e-10, std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(local_planner::sampleBsplinePath(spline, "world", {}, dt), std::invalid_argument);
  points(0, 2) = std::numeric_limits<double>::quiet_NaN();
  local_planner::UniformBspline invalid(points, 3, 0.5);
  EXPECT_THROW(local_planner::sampleBsplinePath(invalid, "world", {}, 0.05), std::invalid_argument);
}
