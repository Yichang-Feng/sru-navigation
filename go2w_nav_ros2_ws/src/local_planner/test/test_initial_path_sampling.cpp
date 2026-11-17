// 回归：过期旧轨迹、零弧长和极小步长必须有限次退出，不能越界或持续扩张点列。
#include <limits>
#include <gtest/gtest.h>
#include <local_planner/plan_manage/initial_path_sampling.h>

namespace
{
local_planner::LocalTrajData previousTrajectory(bool stationary = false)
{
  local_planner::LocalTrajData result;
  Eigen::MatrixXd controls = Eigen::MatrixXd::Zero(3, 7);
  if (!stationary)
    for (int i = 0; i < controls.cols(); ++i) controls(0, i) = 0.2 * i;
  result.position_traj_ = local_planner::UniformBspline(controls, 3, 0.2);
  result.velocity_traj_ = result.position_traj_.getDerivative();
  result.acceleration_traj_ = result.velocity_traj_.getDerivative();
  result.duration_ = result.position_traj_.getTimeSum();
  result.traj_id_ = 1;
  return result;
}
}

TEST(InitialPathSampling, RejectsInvalidAndUnboundedSampleCounts)
{
  std::size_t segments = 0;
  EXPECT_TRUE(local_planner::initialSampleSegments(1.0, 0.1, segments));
  EXPECT_EQ(segments, 10u);
  for (double dt : {0.0, -0.1, 1e-20, std::numeric_limits<double>::quiet_NaN()})
    EXPECT_FALSE(local_planner::initialSampleSegments(1.0, dt, segments));
  EXPECT_FALSE(local_planner::initialSampleSegments(1e100, 0.1, segments));
  EXPECT_FALSE(local_planner::initialSampleSegments(0.0, 0.1, segments));
}

TEST(InitialPathSampling, RejectsDegeneratePolylineAndBoundsMemory)
{
  std::vector<Eigen::Vector3d> result;
  EXPECT_FALSE(local_planner::resampleInitialPolyline({}, 0.2, result));
  EXPECT_FALSE(local_planner::resampleInitialPolyline({Eigen::Vector3d::Zero()}, 0.2, result));
  EXPECT_FALSE(local_planner::resampleInitialPolyline(
      std::vector<Eigen::Vector3d>(10, Eigen::Vector3d::Zero()), 0.2, result));
  EXPECT_FALSE(local_planner::resampleInitialPolyline(
      std::vector<Eigen::Vector3d>(513, Eigen::Vector3d::Zero()), 0.2, result));
  EXPECT_FALSE(local_planner::resampleInitialPolyline(
      {Eigen::Vector3d::Zero(), Eigen::Vector3d(1e100, 0, 0)}, 0.2, result));
  EXPECT_TRUE(result.empty());
}

TEST(InitialPathSampling, ResamplesRepeatedPointsWithoutZeroDivision)
{
  std::vector<Eigen::Vector3d> input = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX()};
  std::vector<Eigen::Vector3d> result;
  ASSERT_TRUE(local_planner::resampleInitialPolyline(input, 0.2, result));
  EXPECT_EQ(result.size(), 7u);
  EXPECT_TRUE(result.front().isApprox(input.front()));
  EXPECT_TRUE(result.back().isApprox(input.back()));
  for (const auto &point : result) EXPECT_TRUE(point.allFinite());
}

TEST(InitialPathSampling, RejectsExpiredOldTrajectoryAndHandlesItsLastInterval)
{
  auto previous = previousTrajectory();
  std::vector<Eigen::Vector3d> points, derivatives;
  const Eigen::Vector3d target(1.2, 0.0, 0.0);
  for (double elapsed : {-0.1, previous.duration_, previous.duration_ + 1.0,
                         std::numeric_limits<double>::quiet_NaN()})
    EXPECT_FALSE(local_planner::samplePreviousInitialPath(
        previous, elapsed, 0.2, target, Eigen::Vector3d::Zero(), 1.0, 0.2, points, derivatives));
  ASSERT_TRUE(local_planner::samplePreviousInitialPath(
      previous, previous.duration_ - 1e-6, 0.2, target, Eigen::Vector3d::Zero(),
      1.0, 0.2, points, derivatives));
  EXPECT_GE(points.size(), 7u);
  EXPECT_LE(points.size(), local_planner::UniformBspline::kMaxParameterizationPoints);
  EXPECT_TRUE(points.back().isApprox(target));
  EXPECT_EQ(derivatives.size(), 4u);
}

TEST(InitialPathSampling, ThousandsOfStationaryRetriesRemainBounded)
{
  auto previous = previousTrajectory(true);
  std::vector<Eigen::Vector3d> points, derivatives;
  for (int i = 0; i < 4000; ++i)
  {
    ASSERT_FALSE(local_planner::samplePreviousInitialPath(
        previous, previous.duration_ - 1e-6, 0.2, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), 1.0, 0.2, points, derivatives));
    ASSERT_TRUE(points.empty());
  }
}

TEST(InitialPathSampling, SamplesLoggedShortHighSpeedBoundaryWithLimits)
{
  const Eigen::Vector3d start(2.67, 4.12, -0.0543), target(2.66, 4.35, -0.0553);
  const Eigen::Vector3d velocity(0.0615, 0.998, 0.00279), target_velocity(-0.0984, 0.994, 0.00408);
  const double duration = std::sqrt((start - target).norm() / 0.5);
  auto polynomial = PolynomialTraj::one_segment_traj_gen(
      start, velocity, Eigen::Vector3d::Zero(), target, target_velocity, Eigen::Vector3d::Zero(), duration);
  double dt = 0.24;
  std::vector<Eigen::Vector3d> points;
  ASSERT_TRUE(local_planner::samplePolynomialInitialPath(polynomial, duration, 0.2, dt, points));
  EXPECT_GE(points.size(), 7u);
  EXPECT_LE(points.size(), local_planner::UniformBspline::kMaxParameterizationPoints);
  EXPECT_TRUE(points.front().isApprox(start, 1e-6));
  EXPECT_TRUE(points.back().isApprox(target, 1e-6));
  EXPECT_GE(dt, local_planner::UniformBspline::kMinSampleInterval);
  dt = 1e-20;
  EXPECT_FALSE(local_planner::samplePolynomialInitialPath(polynomial, duration, 0.2, dt, points));
  EXPECT_TRUE(points.empty());
}
