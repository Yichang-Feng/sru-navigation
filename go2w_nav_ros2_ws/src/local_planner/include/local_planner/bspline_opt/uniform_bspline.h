// 中文导读：B 样条数学工具：构造节点向量、De Boor 求值、求导以及可行性检查和时间重分配。

#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace local_planner
{
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  class UniformBspline
  {
  private:
    // 样条控制点矩阵；当前实现每列是一个三维控制点（3×N）。
    // 行对应坐标分量，列对应控制点序号。
    // 当前规划使用三维坐标，不支持直接把路径点作为控制点输出。
    // 例如 N 个三维控制点的矩阵尺寸为 3×N。
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    Eigen::VectorXd u_; // knots vector
    double interval_;   // knot span \delta t

    Eigen::MatrixXd getDerivativeControlPoints();

    double limit_vel_, limit_acc_, feasibility_tolerance_; // physical limits and feasibility tolerance

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // 按均匀节点间隔初始化 B 样条。
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // 节点向量、控制点和有效时间区间的读写接口。

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);

    // 求值及求导：位置样条的一阶、二阶导数分别为速度和加速度。

    Eigen::VectorXd evaluateDeBoor(const double &u) const;                                               // use u \in [up, u_mp]
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) const { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    UniformBspline getDerivative();

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    // 密集 QR 的内存/时间开销随点数增长；局部拟合最多允许 512 个采样点。
    static constexpr std::size_t kMaxParameterizationPoints = 512;
    static constexpr double kMinSampleInterval = 1e-4;
    // 无效或超限输入返回 false，并清空输出，调用者不得继续构造样条。
    static bool parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* 检查物理可行性并按比例调整时间。 */

    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    bool checkFeasibility(double &ratio, bool show = false);
    void lengthenTime(const double &ratio);

    /* 轨迹长度、速度、加速度和 jerk 统计。 */

    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace local_planner
#endif
