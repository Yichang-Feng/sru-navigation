// 验证样条求值及导数，作为局部路径采样的数学基础。
#include <gtest/gtest.h>
#include <limits>

#include <Eigen/Core>
#include <local_planner/bspline_opt/uniform_bspline.h>

TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  local_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = local_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}

TEST(UniformBspline, ParameterizationRejectsInvalidInputBeforeAllocation)
{
  std::vector<Eigen::Vector3d> points(7, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> derivatives(4, Eigen::Vector3d::Zero());
  Eigen::MatrixXd output = Eigen::MatrixXd::Ones(3, 7);
  EXPECT_FALSE(local_planner::UniformBspline::parameterizeToBspline(0.1, points, {}, output));
  EXPECT_EQ(output.size(), 0);
  for (double dt : {0.0, -0.1, 1e-20, std::numeric_limits<double>::quiet_NaN()})
    EXPECT_FALSE(local_planner::UniformBspline::parameterizeToBspline(dt, points, derivatives, output));
  points.resize(local_planner::UniformBspline::kMaxParameterizationPoints + 1);
  EXPECT_FALSE(local_planner::UniformBspline::parameterizeToBspline(0.1, points, derivatives, output));
  points.assign(7, Eigen::Vector3d::Zero());
  points[3].x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(local_planner::UniformBspline::parameterizeToBspline(0.1, points, derivatives, output));
  EXPECT_EQ(output.size(), 0);
}

TEST(UniformBspline, SharedQrParameterizationPreservesLineAndEndpointDerivatives)
{
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 7; ++i) points.emplace_back(i * 0.2, 0.0, 0.5);
  std::vector<Eigen::Vector3d> derivatives = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX(),
                                            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  Eigen::MatrixXd controls;
  ASSERT_TRUE(local_planner::UniformBspline::parameterizeToBspline(0.2, points, derivatives, controls));
  local_planner::UniformBspline spline(controls, 3, 0.2);
  auto velocity = spline.getDerivative();
  EXPECT_TRUE(spline.evaluateDeBoorT(0.0).isApprox(points.front(), 1e-6));
  EXPECT_TRUE(spline.evaluateDeBoorT(1.2).isApprox(points.back(), 1e-6));
  EXPECT_TRUE(velocity.evaluateDeBoorT(0.0).isApprox(Eigen::Vector3d::UnitX(), 1e-6));
}
