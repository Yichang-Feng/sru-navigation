// 中文导读：规划算法管理器：生成参考初值，调用 A* 与样条优化，检查可行性并保存位置/速度/加速度轨迹。
// 数据约定：位置 m，时间 s，速度 m/s；地图和局部轨迹使用 grid_map.frame_id。

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <local_planner/bspline_opt/bspline_optimizer.h>
#include <local_planner/bspline_opt/uniform_bspline.h>
#include <local_planner/plan_env/grid_map.h>
#include <local_planner/plan_manage/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <local_planner/traj_utils/planning_visualization.h>

namespace local_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class LocalPlannerManager
  {
    // SECTION stable
  public:
    LocalPlannerManager();
    ~LocalPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* 规划接口：参考拟合、局部避障优化和停车轨迹。 */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                       const std::vector<Eigen::Vector3d> &reference_path_world = {});
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis = nullptr);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

    // 由状态机消费资源异常，进入停车等待；普通动力学失败仍按原策略处理。
    bool takeResourceFailure()
    {
      const bool failed = resource_failure_;
      resource_failure_ = false;
      return failed;
    }

  private:
    rclcpp::Node *node_{nullptr};
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continuous_failures_count_{0};
    bool resource_failure_{false};
    bool reboundReplanImpl(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                           Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit,
                           bool flag_randomPolyTraj, const std::vector<Eigen::Vector3d> &reference_path_world);

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);
    bool checkDynamicFeasibility(UniformBspline position_traj);

    bool reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<LocalPlannerManager> Ptr;

    // !SECTION
  };
} // namespace local_planner

#endif
