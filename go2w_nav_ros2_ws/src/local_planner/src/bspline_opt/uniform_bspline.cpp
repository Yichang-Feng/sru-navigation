// 中文导读：B 样条数学工具：构造节点向量、De Boor 求值、求导以及可行性检查和时间重分配。

#include "local_planner/bspline_opt/uniform_bspline.h"
#include <cmath>

namespace local_planner
{

  // 由控制点、样条次数和节点间隔构造 B 样条。
  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

  // 初始化节点向量；当前控制点矩阵为 3×N，每列一个控制点。
  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;
    p_ = order;
    interval_ = interval;

    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {

      if (i <= p_)
      {
        u_(i) = double(-p_ + i) * interval_;
      }
      else if (i > p_ && i <= m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  // 替换节点向量，可用于表示非均匀时间分布。
  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  // 读取完整节点向量。
  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  // 返回可求值的样条参数区间，不包含端部外延节点。
  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    um = u_(p_);
    um_p = u_(m_ - p_);

    return true;
  }

  // 读取控制点矩阵；控制点一般不在实际轨迹上。
  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  // 使用 De Boor 算法求值；evaluateDeBoorT 会把从零开始的时间转为节点参数。
  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u) const
  {

    double ub = min(max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* deBoor's alg */
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  // 由相邻控制点差分与节点跨度生成导数样条的控制点。
  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  // 构造导数 B 样条，用于求速度、加速度和切线方向。
  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  // 读取初始均匀节点间隔，单位 s。
  double UniformBspline::getInterval() { return interval_; }

  // 设置速度、加速度限值及可行性容差。
  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    feasibility_tolerance_ = tolerance;
  }

  // 检查控制点差分对应的速度、加速度，并估计所需时间拉伸倍率。
  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility and insert points */
    double max_vel = -1.0;
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
          fabs(vel(2)) > enlarged_vel_lim)
      {

        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_vel = max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i)
    {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
          fabs(acc(2)) > enlarged_acc_lim)
      {

        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_acc = max(max_acc, fabs(acc(j)));
        }
      }
    }

    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  // 通过拉长节点时间降低速度与加速度，不直接改变空间控制点。
  void UniformBspline::lengthenTime(const double &ratio)
  {
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc = delta_t / double(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  // void UniformBspline::recomputeInit() {}

  // 由采样位置点和端点速度/加速度条件求解 B 样条控制点。
  bool UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    ctrl_pts.resize(0, 0);
    // 所有校验均在分配密集矩阵之前执行；异常输入不能进入 QR 或访问边界导数。
    if (!std::isfinite(ts) || ts < kMinSampleInterval || point_set.size() < 4 ||
        point_set.size() > kMaxParameterizationPoints || start_end_derivative.size() != 4)
      return false;
    for (const auto &point : point_set)
      if (!point.allFinite()) return false;
    for (const auto &derivative : start_end_derivative)
      if (!derivative.allFinite()) return false;

    const Eigen::Index K = static_cast<Eigen::Index>(point_set.size());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);
    Eigen::MatrixXd rhs(K + 4, 3);
    const Eigen::RowVector3d position_row(1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0);
    const Eigen::RowVector3d velocity_row(-1.0 / (2.0 * ts), 0.0, 1.0 / (2.0 * ts));
    const Eigen::RowVector3d acceleration_row(1.0 / ts / ts, -2.0 / ts / ts, 1.0 / ts / ts);
    for (Eigen::Index i = 0; i < K; ++i)
    {
      A.block<1, 3>(i, i) = position_row;
      rhs.row(i) = point_set[i].transpose();
    }
    A.block<1, 3>(K, 0) = velocity_row;
    A.block<1, 3>(K + 1, K - 1) = velocity_row;
    A.block<1, 3>(K + 2, 0) = acceleration_row;
    A.block<1, 3>(K + 3, K - 1) = acceleration_row;
    for (Eigen::Index i = 0; i < 4; ++i)
      rhs.row(K + i) = start_end_derivative[i].transpose();

    // 三个坐标共享同一次 QR 分解，避免对相同矩阵重复分配和分解三遍。
    const Eigen::MatrixXd solution = A.colPivHouseholderQr().solve(rhs);
    if (!solution.allFinite())
      return false;
    ctrl_pts = solution.transpose();
    return true;
  }

  // 读取有效样条区间的总时长，单位 s。
  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  // 对轨迹离散采样估计空间弧长，单位 m。
  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  // 计算三阶导数的积分代价，用于评价轨迹平滑程度。
  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  // 采样速度曲线，统计平均与最大速度，单位 m/s。
  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  // 采样加速度曲线，统计平均与最大加速度，单位 m/s²。
  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace local_planner
