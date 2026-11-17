// 验证参考平台和下降段的高度插值。
#include <gtest/gtest.h>
#include <local_planner/path_searching/reference_height.h>

TEST(ReferenceHeight, KeepsPlatformBeforeDescending)
{
  const std::vector<Eigen::Vector3d> path{
      {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
      {1.5, 0.0, -0.3}, {2.0, 0.0, -0.6}};
  EXPECT_NEAR(referenceHeightAtXY(path, {0.5, 0.0}), 0.0, 1e-9);
  EXPECT_NEAR(referenceHeightAtXY(path, {1.25, 0.0}), -0.15, 1e-9);
  EXPECT_NEAR(referenceHeightAtXY(path, {1.75, 0.1}), -0.45, 1e-9);
}
