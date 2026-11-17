// 中文导读：规划算法管理器：生成参考初值，调用 A* 与样条优化，检查可行性并保存位置/速度/加速度轨迹。
// 数据约定：位置 m，时间 s，速度 m/s；地图和局部轨迹使用 grid_map.frame_id。

// #include <fstream>
#include <local_planner/plan_manage/planner_manager.h>
#include <local_planner/path_searching/reference_height.h>
#include <local_planner/plan_manage/initial_path_sampling.h>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace local_planner
{
  namespace
  {
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }

    bool followReferencePath(std::vector<Eigen::Vector3d> &points,
                             const std::vector<Eigen::Vector3d> &reference,
                             const Eigen::Vector3d &start, const Eigen::Vector3d &target,
                             const double spacing)
    {
      if (reference.size() < 2 || reference.size() > UniformBspline::kMaxParameterizationPoints ||
          points.size() > UniformBspline::kMaxParameterizationPoints ||
          !std::isfinite(spacing) || spacing <= 0.0)
        return false;
      for (const auto &point : reference)
        if (!point.allFinite()) return false;
      std::vector<double> arc(reference.size(), 0.0);
      for (size_t i = 1; i < reference.size(); ++i)
        arc[i] = arc[i - 1] + (reference[i] - reference[i - 1]).norm();
      if (!std::isfinite(arc.back()) || arc.back() < 1e-6)
        return false;

      const double requested = std::ceil(arc.back() / std::max(spacing, 0.05)) + 1.0;
      if (!std::isfinite(requested) || requested > UniformBspline::kMaxParameterizationPoints)
        return false;
      const size_t count = std::max<size_t>(7, std::max(points.size(),
          static_cast<size_t>(requested)));
      points.resize(count);
      size_t segment = 1;
      const Eigen::Vector3d start_offset = start - reference.front();
      for (size_t i = 0; i < count; ++i)
      {
        const double distance = arc.back() * static_cast<double>(i) / static_cast<double>(count - 1);
        while (segment + 1 < arc.size() && arc[segment] < distance)
          ++segment;
        const double length = arc[segment] - arc[segment - 1];
        const double ratio = length > 1e-9 ? (distance - arc[segment - 1]) / length : 0.0;
        points[i] = reference[segment - 1] + ratio * (reference[segment] - reference[segment - 1]);
        // Connect the current world start to the reference without moving the first point.
        const double blend = std::max(0.0, 1.0 - distance / 0.5);
        points[i] += blend * blend * start_offset;
      }
      points.front() = start;
      points.back() = target;
      return true;
    }
  } // namespace

  // SECTION interfaces for setup and query

  // 构造规划管理器；实际地图与优化器在 initPlanModules 中创建。
  LocalPlannerManager::LocalPlannerManager() {}

  LocalPlannerManager::~LocalPlannerManager() { std::cout << "des manager" << std::endl; }

  // 读取规划限制，创建滑动地图、样条优化器和 A* 搜索器。
  void LocalPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);

    local_data_.traj_id_ = 0;
    local_data_.duration_ = 0.0;
    local_data_.start_time_ = node_->now();
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  // 生成局部轨迹初值并避障优化，检查动力学可行性后保存新轨迹。
  bool LocalPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                                        const std::vector<Eigen::Vector3d> &reference_path_world)
  {
    try
    {
      return reboundReplanImpl(start_pt, start_vel, start_acc, local_target_pt,
                               local_target_vel, flag_polyInit, flag_randomPolyTraj, reference_path_world);
    }
    catch (const std::bad_alloc &)
    {
      resource_failure_ = true;
      RCLCPP_ERROR(node_->get_logger(), "[DIAG][RESOURCE] replan allocation failed; request stop and wait for a new path");
    }
    catch (const std::length_error &)
    {
      resource_failure_ = true;
      RCLCPP_ERROR(node_->get_logger(), "[DIAG][RESOURCE] replan container size invalid; request stop and wait for a new path");
    }
    continuous_failures_count_++;
    return false;
  }

  bool LocalPlannerManager::reboundReplanImpl(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                            Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                            Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                                            const std::vector<Eigen::Vector3d> &reference_path_world)
  {

    static int count = 0;
    const int manager_attempt = count;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if (!start_pt.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
        !local_target_pt.allFinite() || !local_target_vel.allFinite() ||
        !std::isfinite(pp_.ctrl_pt_dist) || !std::isfinite(pp_.max_vel_) ||
        !std::isfinite(pp_.max_acc_) || !std::isfinite(pp_.planning_horizon_) ||
        pp_.planning_horizon_ <= 0.0 || pp_.ctrl_pt_dist <= 0.0 ||
        pp_.max_vel_ <= 0.0 || pp_.max_acc_ <= 0.0)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[DIAG][MANAGER id=%d] invalid_input start_finite=%d target_finite=%d "
                   "velocity_finite=%d ctrl_dist=%.6f max_vel=%.6f max_acc=%.6f",
                   manager_attempt, start_pt.allFinite(), local_target_pt.allFinite(),
                   start_vel.allFinite() && local_target_vel.allFinite(),
                   pp_.ctrl_pt_dist, pp_.max_vel_, pp_.max_acc_);
      continuous_failures_count_++;
      return false;
    }

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "[DIAG][MANAGER id=%d] target_too_close distance=%.3f",
                           manager_attempt, (start_pt - local_target_pt).norm());
      continuous_failures_count_++;
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    /*** STEP 1: INIT：旧轨迹最多复用一次，失败则仅尝试一次有界多项式初始化。 ***/
    double ts = pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    bool reused = false;
    if (!flag_polyInit && local_data_.traj_id_ > 0)
    {
      const double elapsed = (node_->now() - local_data_.start_time_).seconds();
      reused = samplePreviousInitialPath(local_data_, elapsed, ts, local_target_pt,
                                        local_target_vel, pp_.max_vel_, pp_.ctrl_pt_dist,
                                        point_set, start_end_derivatives);
      if (reused && point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3.0)
        reused = false;
      if (!reused)
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[DIAG][INIT] previous trajectory unusable; using bounded polynomial fallback");
    }
    if (!reused)
    {
      // 不继承旧细分轮次的步长，防止失败重试把 ts 持续缩小至零。
      ts = pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2;
      const double distance = (start_pt - local_target_pt).norm();
      const double time = pp_.max_vel_ * pp_.max_vel_ / pp_.max_acc_ > distance
          ? std::sqrt(distance / pp_.max_acc_)
          : (distance - pp_.max_vel_ * pp_.max_vel_ / pp_.max_acc_) / pp_.max_vel_ +
                2.0 * pp_.max_vel_ / pp_.max_acc_;
      if (!std::isfinite(time) || time <= 0.0)
      {
        continuous_failures_count_++;
        return false;
      }
      PolynomialTraj trajectory;
      if (!flag_randomPolyTraj)
      {
        trajectory = PolynomialTraj::one_segment_traj_gen(
            start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
            Eigen::Vector3d::Zero(), time);
      }
      else
      {
        Eigen::Vector3d horizon = (start_pt - local_target_pt).cross(Eigen::Vector3d::UnitZ());
        // 垂直路线与 Z 轴叉积为零，不能 normalized() 后把 NaN 传给优化器。
        if (horizon.norm() < 1e-8) horizon = Eigen::Vector3d::UnitX();
        horizon.normalize();
        const Eigen::Vector3d vertical = (start_pt - local_target_pt).cross(horizon).normalized();
        const double scale = -0.978 / (continuous_failures_count_ + 0.989) + 0.989;
        const Eigen::Vector3d midpoint = (start_pt + local_target_pt) / 2.0 +
            (static_cast<double>(rand()) / RAND_MAX - 0.5) * distance * horizon * 0.8 * scale +
            (static_cast<double>(rand()) / RAND_MAX - 0.5) * distance * vertical * 0.4 * scale;
        Eigen::MatrixXd positions(3, 3);
        positions.col(0) = start_pt;
        positions.col(1) = midpoint;
        positions.col(2) = local_target_pt;
        const Eigen::VectorXd durations = Eigen::VectorXd::Constant(2, time / 2.0);
        trajectory = PolynomialTraj::minSnapTraj(
            positions, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), durations);
      }
      if (!samplePolynomialInitialPath(trajectory, time, pp_.ctrl_pt_dist, ts, point_set))
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                             "[DIAG][INIT] bounded_sampling_failed duration=%.6f dt=%.6f max_points=%zu",
                             time, ts, UniformBspline::kMaxParameterizationPoints);
        continuous_failures_count_++;
        return false;
      }
      start_end_derivatives = {start_vel, local_target_vel, start_acc, trajectory.evaluateAcc(time)};
    }

    const bool have_reference = reference_path_world.size() >= 2;
    if (have_reference && flag_randomPolyTraj)
    {
      // Preserve the randomized lateral escape, but use the route's height.
      for (auto &point : point_set)
        point(2) = referenceHeightAtXY(reference_path_world, point.head<2>());
      point_set.front()(2) = start_pt(2);
      point_set.back() = local_target_pt;
    }
    else if (have_reference)
    {
      if (!followReferencePath(point_set, reference_path_world,
                               start_pt, local_target_pt, pp_.ctrl_pt_dist))
      {
        RCLCPP_WARN(node_->get_logger(), "[DIAG][INIT] reference path is invalid or exceeds the sampling limit");
        continuous_failures_count_++;
        return false;
      }
    }
    if (!have_reference)
      applyLinearZReference(point_set, start_pt(2), local_target_pt(2));
    bspline_optimizer_rebound_->a_star_->setHeightReference(have_reference ? reference_path_world
                                                                           : std::vector<Eigen::Vector3d>{});

    Eigen::MatrixXd ctrl_pts;
    if (!UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts))
    {
      RCLCPP_WARN(node_->get_logger(), "[DIAG][INIT] parameterization rejected samples=%zu dt=%.6f",
                   point_set.size(), ts);
      continuous_failures_count_++;
      return false;
    }

    if (ctrl_pts.rows() != 3 || ctrl_pts.cols() < 7 || !ctrl_pts.allFinite())
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[DIAG][MANAGER id=%d] invalid_parameterization samples=%zu derivatives=%zu "
                   "ctrl_shape=%ldx%ld finite=%d ts=%.6f",
                   manager_attempt, point_set.size(), start_end_derivatives.size(),
                   static_cast<long>(ctrl_pts.rows()), static_cast<long>(ctrl_pts.cols()),
                   ctrl_pts.allFinite(), ts);
      continuous_failures_count_++;
      return false;
    }

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "[DIAG][MANAGER id=%d] optimize_failed ctrl_points=%ld samples=%zu "
                           "ts=%.4f init_ms=%.3f consecutive_failures=%d",
                           manager_attempt, static_cast<long>(ctrl_pts.cols()), point_set.size(), ts,
                           t_init * 1000.0, continuous_failures_count_);
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "[DIAG][MANAGER id=%d] refine_or_dynamics_failed refine_ok=%d "
                           "duration=%.3f ts=%.4f consecutive_failures=%d",
                           manager_attempt, flag_step_2_success, pos.getTimeSum(), ts,
                           continuous_failures_count_);
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    RCLCPP_INFO(node_->get_logger(),
                "[DIAG][MANAGER id=%d] success traj_id=%d duration=%.3f ctrl_points=%ld "
                "init_ms=%.3f optimize_ms=%.3f refine_ms=%.3f",
                manager_attempt, local_data_.traj_id_, local_data_.duration_,
                static_cast<long>(pos.getControlPoint().cols()), t_init * 1000.0,
                t_opt * 1000.0, t_refine * 1000.0);
    return true;
  }

  // 以当前位置构造静止控制点，生成停车轨迹。
  bool LocalPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  // 沿外部参考点生成分段参考轨迹，不进行全局地图搜索。
  bool LocalPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  // 时间拉伸后重新参数化并精修，使轨迹尽量贴合原几何形状。
  bool LocalPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    if (!reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc))
      return false;

    traj = UniformBspline(ctrl_pts, 3, ts);

    const auto segments = static_cast<std::size_t>(ctrl_pts.cols() - 3);
    const double duration = traj.getTimeSum();
    bspline_optimizer_rebound_->ref_pts_.clear();
    bspline_optimizer_rebound_->ref_pts_.reserve(segments + 1);
    for (std::size_t i = 0; i <= segments; ++i)
      bspline_optimizer_rebound_->ref_pts_.push_back(
          traj.evaluateDeBoorT(duration * (static_cast<double>(i) / segments)));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  // 保存位置、速度、加速度样条，更新起始时间、时长和轨迹编号。
  void LocalPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  // 离散检查速度/加速度；最终阈值为管理器限值加对应附加容差。
  bool LocalPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;
    if (!std::isfinite(duration) || duration <= 0.0 || duration / sample_dt > 10000.0 ||
        !std::isfinite(vel_limit) || vel_limit <= 0.0 ||
        !std::isfinite(acc_limit) || acc_limit <= 0.0)
      return false;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (!vel.allFinite() || vel.norm() > vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (!acc.allFinite() || acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;
  }

  // 在新时间尺度下重采样并重新拟合控制点。
  bool LocalPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    ctrl_pts.resize(0, 0);
    if (!std::isfinite(ratio) || ratio < 1.0)
      return false;
    const double original_duration = bspline.getTimeSum();
    const Eigen::Index segments = bspline.getControlPoint().cols() - 3;
    if (!std::isfinite(original_duration) || original_duration <= 0.0 || segments < 1 ||
        segments + 1 > static_cast<Eigen::Index>(UniformBspline::kMaxParameterizationPoints))
      return false;
    bspline.lengthenTime(ratio);
    const double duration = bspline.getTimeSum();
    dt = duration / static_cast<double>(segments);
    if (!std::isfinite(duration) || !std::isfinite(dt) || dt < UniformBspline::kMinSampleInterval)
      return false;
    time_inc = duration - original_duration;
    vector<Eigen::Vector3d> samples;
    samples.reserve(static_cast<std::size_t>(segments + 1));
    // 不再使用 duration + 1e-4 的浮点循环，dt 很小时固定误差项会产生海量额外采样。
    for (Eigen::Index i = 0; i <= segments; ++i)
      samples.push_back(bspline.evaluateDeBoorT(duration * (static_cast<double>(i) / segments)));
    return UniformBspline::parameterizeToBspline(dt, samples, start_end_derivative, ctrl_pts);
  }

} // namespace local_planner
