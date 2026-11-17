// 中文导读：局部重规划状态机：接收 /global_path，根据机身里程计和占据地图规划，并处理 TF 修正与紧急停车。
// 数据约定：位置 m，时间 s，速度 m/s；地图和局部轨迹使用 grid_map.frame_id。

#ifndef LOCAL_PLANNER_REPLAN_FSM_H_
#define LOCAL_PLANNER_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <local_planner/bspline_opt/bspline_optimizer.h>
#include <local_planner/plan_env/grid_map.h>
#include <local_planner/msg/bspline.hpp>
#include <local_planner/msg/data_disp.hpp>
#include <local_planner/plan_manage/planner_manager.h>
#include <local_planner/traj_utils/planning_visualization.h>

using std::vector;

namespace local_planner
{

  class LocalReplanFSM
  {

  private:
    /* 状态机：输入始终为外部参考路径，状态名称中的 TARGET 指该路径的目标。 */
    enum FSM_EXEC_STATE
    {
      INIT,            // 等待机身里程计及首条有效参考路径。
      WAIT_TARGET,     // 等待新的有效参考路径。
      GEN_NEW_TRAJ,    // 从当前实测位置生成新轨迹。
      REPLAN_TRAJ,     // 执行中局部重规划，或定位 TF 修正后的重新规划。
      EXEC_TRAJ,       // 监控当前轨迹进度和周期重规划条件。
      EMERGENCY_STOP   // 轨迹失效或碰撞风险导致停车。
    };

    /* 规划模块与可视化工具。 */
    LocalPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    local_planner::msg::DataDisp data_disp_;

    /* 参数：距离 m、时间 s；具体用途和默认值见 config/planner.yaml。 */
    double no_replan_thresh_, replan_thresh_;
    double planning_horizon_;
    double emergency_time_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;
    std::string reference_frame_id_;
    double reference_tf_max_age_{0.3}, reference_tf_grace_{0.5};
    double reference_tf_jump_translation_{0.2}, reference_tf_jump_yaw_{5.0 * 3.14159265358979323846 / 180.0};

    /* 当前输入状态、重规划失败统计和 TF 恢复状态。 */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    uint64_t plan_attempt_id_{0};
    rclcpp::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d start_pt_, start_vel_, start_acc_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> local_reference_path_world_;
    Eigen::Vector3d reference_end_pt_{Eigen::Vector3d::Zero()};
    Eigen::Isometry3d planning_reference_to_world_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d last_reference_to_world_{Eigen::Isometry3d::Identity()};
    bool have_last_reference_transform_{false};
    bool reference_tf_stop_{false};
    bool reference_tf_jump_failed_{false};
    bool reference_jump_replan_{false};
    rclcpp::Time reference_tf_missing_since_{0, 0, RCL_ROS_TIME};

    bool flag_escape_emergency_;

    /* ROS 接口：全局路径输入，B 样条与 /local_path 输出。 */
    rclcpp::Node *node_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Publisher<local_planner::msg::Bspline>::SharedPtr bspline_pub_;
    // 几何路径输出：可靠、非持久化；定时刷新供 path_follow 的路径超时检测使用。
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::TimerBase::SharedPtr local_path_timer_;
    nav_msgs::msg::Path sampled_local_path_;
    double local_path_sample_dt_{0.05};  // B 样条采样时间间隔，单位秒。
    bool local_path_active_{false};     // 停车或轨迹过期后必须重新规划才能恢复。
    rclcpp::Publisher<local_planner::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;

    /* 内部辅助函数：规划、TF 处理、碰撞检查和路径输出。 */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, LocalReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool getLocalTarget();
    bool lookupReferenceTransform(Eigen::Isometry3d &reference_to_world);
    bool monitorReferenceTransform();
    void finishProcess();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    void updateSampledLocalPath();  // 新轨迹生成时采样一次，后续心跳复用点列。
    void publishLocalPath();        // 正常发布路径；无有效轨迹时发布空路径要求停车。

    /* ROS 回调：输入消息、规划时钟和安全检查。 */
    void execFSMCallback();
    void checkCollisionCallback();
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);


  public:
    LocalReplanFSM(/* args */)
    {
    }
    ~LocalReplanFSM()
    {
    }

    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace local_planner

#endif
