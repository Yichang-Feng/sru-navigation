// 中文导读：局部重规划状态机：接收 /global_path，根据机身里程计和占据地图规划，并处理 TF 修正与紧急停车。
// 数据约定：位置 m，时间 s，速度 m/s；地图和局部轨迹使用 grid_map.frame_id。


#include <local_planner/plan_manage/local_replan_fsm.h>
#include <local_planner/plan_manage/reference_path_utils.h>
#include <local_planner/plan_manage/reference_frame_utils.h>
#include <local_planner/traj_utils/bspline_path.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tf2/time.h>
#include <tf2/exceptions.h>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace local_planner
{

  // 初始化 TF、规划模块、话题接口和定时器；唯一导航输入为 /global_path。
  void LocalReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.0);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    reference_tf_max_age_ = load_parameter<double>(node_, "fsm.reference_tf_max_age", 0.3);
    reference_tf_grace_ = load_parameter<double>(node_, "fsm.reference_tf_grace", 0.5);
    reference_tf_jump_translation_ = load_parameter<double>(node_, "fsm.reference_tf_jump_translation", 0.2);
    reference_tf_jump_yaw_ = load_parameter<double>(node_, "fsm.reference_tf_jump_yaw_deg", 5.0) * M_PI / 180.0;
    if (reference_tf_max_age_ <= 0.0 || reference_tf_grace_ <= 0.0 ||
        reference_tf_jump_translation_ <= 0.0 || reference_tf_jump_yaw_ <= 0.0)
      throw std::runtime_error("reference TF limits must be positive");

    /* 创建规划算法模块与可视化工具。 */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new LocalPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* 注册规划、安全检查和输入消息回调。 */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&LocalReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&LocalReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&LocalReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&LocalReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<local_planner::msg::Bspline>("planning/bspline", 10);
    local_path_sample_dt_ = load_parameter<double>(node_, "local_path.sample_dt", 0.05);
    const double path_rate = load_parameter<double>(node_, "local_path.publish_rate", 10.0);
    if (!std::isfinite(local_path_sample_dt_) || local_path_sample_dt_ <= 0.0 ||
        !std::isfinite(path_rate) || path_rate < 1.0 || path_rate > 100.0)
      throw std::runtime_error("local path sample_dt must be positive and publish_rate within [1, 100] Hz");
    local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/local_path", rclcpp::QoS(1));
    local_path_timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / path_rate)),
        std::bind(&LocalReplanFSM::publishLocalPath, this));
    data_disp_pub_ = node_->create_publisher<local_planner::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/global_path", 1, std::bind(&LocalReplanFSM::pathCallback, this, std::placeholders::_1));

  }

  // 在输入路径坐标系中拟合参考轨迹，局部选点时再映射到规划坐标系。
  bool LocalReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(), "Reference path requires at least two points");
      return false;
    }

    reference_end_pt_ = waypoints.back();
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i,
                                       reference_frame_id_);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        reference_waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0, reference_frame_id_);
    visualization_->displayGoalPoint(reference_end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3,
                                     static_cast<int>(waypoints.size()) - 1, reference_frame_id_);

    return true;
  }

  // 校验外部路径并替换参考路线；无里程计、空路径或无效点不会触发规划。
  void LocalReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty global_path; ignoring");
      return;
    }

    if (!have_odom_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No odometry yet; ignoring global_path");
      return;
    }

    if (msg->header.frame_id.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring global_path with an empty frame_id");
      return;
    }
    for (const auto &pose : msg->poses)
    {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != msg->header.frame_id)
      {
        RCLCPP_WARN(node_->get_logger(), "Ignoring global_path with mixed pose frames");
        return;
      }
    }

    std::vector<Eigen::Vector3d> waypoints;
    std::string path_error;
    if (!prepareReferenceWaypoints(*msg, body_height_, 0.5, waypoints, &path_error))
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring global_path: %s", path_error.c_str());
      return;
    }

    reference_frame_id_ = msg->header.frame_id;
    have_last_reference_transform_ = false;
    reference_jump_replan_ = false;
    trigger_ = true;
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      if (reference_tf_jump_failed_)
      {
        reference_tf_jump_failed_ = false;
        reference_tf_stop_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "NEW_PATH");
      }
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  // 保存机身位置、速度和姿态，供规划起点及碰撞方向估计使用。
  void LocalReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;


    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();

  }

  // 接收外部执行暂停反馈；冻结轨迹计时并撤销几何跟踪路径。
  void LocalReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    go2_execution_frozen_ = msg->data;
    // 冻结期间不允许几何跟踪器继续追逐旧路径，恢复后可继续当前有效轨迹。
    publishLocalPath();
  }

  // 新 B 样条生成后采样几何路径，后续心跳复用该点列。
  void LocalReplanFSM::updateSampledLocalPath()
  {
    try
    {
      sampled_local_path_ = sampleBsplinePath(
          planner_manager_->local_data_.position_traj_, self_inflation_frame_id_,
          node_->now(), local_path_sample_dt_);
      local_path_active_ = true;
    }
    catch (const std::exception &error)
    {
      local_path_active_ = false;
      RCLCPP_ERROR(node_->get_logger(), "Cannot sample local path: %s", error.what());
    }
    publishLocalPath();
  }

  // 发布有效局部路径的心跳；等待、停车、冻结或过期时输出空路径。
  void LocalReplanFSM::publishLocalPath()
  {
    updateLocalTrajTimeFreeze();
    const auto &info = planner_manager_->local_data_;
    const auto now = node_->now();
    // 只对仍有效的轨迹刷新心跳，不能用新时间戳把已过期的轨迹伪装为有效路径。
    // 首次规划前 start_time_/duration_ 尚未赋值，不读取这些轨迹字段。
    if (local_path_active_)
    {
      const double elapsed = (now - info.start_time_).seconds();
      if (elapsed < -0.05 || elapsed > info.duration_ ||
          exec_state_ == EMERGENCY_STOP || exec_state_ == WAIT_TARGET)
        local_path_active_ = false;
    }
    if (!local_path_active_ || go2_execution_frozen_)
    {
      nav_msgs::msg::Path stop;
      stop.header.frame_id = self_inflation_frame_id_;
      stop.header.stamp = now;
      local_path_pub_->publish(stop);  // path_follow 收到空路径会立即发布零速度。
      return;
    }
    // 保留完整几何轨迹而不按计划时间裁掉前段，允许跟踪器以自己的速度投影进度。
    sampled_local_path_.header.stamp = now;
    for (auto &pose : sampled_local_path_.poses)
      pose.header = sampled_local_path_.header;
    local_path_pub_->publish(sampled_local_path_);
  }

  // 冻结时把轨迹起始时间顺延，防止暂停期间计划进度继续推进。
  void LocalReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  // 由当前机身四元数提取水平朝向，供双圆柱碰撞模型使用。
  double LocalReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  // 由轨迹线段估计偏航，退化线段使用机身当前朝向。
  double LocalReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  // 显示沿机身前后方向放置的两个碰撞圆柱。
  void LocalReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  // 统一切换状态并记录诊断日志；停车/等待状态同步清空局部路径。
  void LocalReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    if (new_state == EMERGENCY_STOP || new_state == WAIT_TARGET)
      publishLocalPath();  // 状态进入停车/等待时同步撤销路径，不等下一次心跳。
    if (pre_s != static_cast<int>(new_state))
      RCLCPP_INFO(node_->get_logger(),
                  "[DIAG][FSM] trigger=%s from=%s to=%s traj_id=%d replan_fail=%d "
                  "odom=(%.3f,%.3f,%.3f) speed=%.3f",
                  pos_call.c_str(), state_str[pre_s].c_str(), state_str[int(new_state)].c_str(),
                  planner_manager_->local_data_.traj_id_, replan_fail_count_,
                  odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), odom_vel_.norm());
  }

  std::pair<int, LocalReplanFSM::FSM_EXEC_STATE> LocalReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  // 输出当前状态，用于诊断输入等待和规划过程。
  void LocalReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  // 周期推进初始化、规划、执行、重规划和紧急停车状态。
  void LocalReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();
    if (!monitorReferenceTransform())
      return;

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for /global_path." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        if (reference_jump_replan_)
        {
          reference_tf_stop_ = true;
          reference_tf_jump_failed_ = true;
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          changeFSMExecState(EMERGENCY_STOP, "TF_JUMP_FAIL");
        }
        else
        {
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      bool success;
      if (reference_jump_replan_)
      {
        // A localization correction invalidates the old trajectory's predicted
        // state. Start the replacement from the measured world-frame state.
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
        success = callReboundReplan(true, false);
      }
      else
        success = planFromCurrentTraj();
      if (success)
      {
        reference_jump_replan_ = false;
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        if (reference_jump_replan_)
        {
          reference_tf_stop_ = true;
          reference_tf_jump_failed_ = true;
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          changeFSMExecState(EMERGENCY_STOP, "TF_JUMP_FAIL");
        }
        else
        {
          replan_fail_count_++;
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (reference_tf_stop_)
      {
        if (!reference_tf_jump_failed_ && odom_vel_.norm() < 0.1)
        {
          reference_tf_stop_ = false;
          reference_jump_replan_ = false;
          have_new_target_ = true;
          changeFSMExecState(GEN_NEW_TRAJ, "TF_RECOVERED");
        }
        break;
      }

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  // 连续规划失败达到阈值时停车，并等待新的参考路径。
  void LocalReplanFSM::finishProcess()
  {
    if (planner_manager_->takeResourceFailure())
    {
      // 资源异常不继续高频重试；立即撤销 /local_path，并等待外部重新下发参考路径。
      callEmergencyStop(odom_pos_);
      have_target_ = false;
      trigger_ = false;
      flag_escape_emergency_ = false;
      replan_fail_count_ = 0;
      changeFSMExecState(WAIT_TARGET, "RESOURCE_FAILURE");
      return;
    }
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  // 从当前计划位置及导数状态重规划，逐步尝试不同初始化策略。
  bool LocalReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);


    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);
    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
          return false;
      }
    }

    return true;

  }

  // 使用实测位置作为新轨迹起点，条件允许时继承当前轨迹的速度/加速度。
  void LocalReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  // 检查近期轨迹碰撞；优先重规划，无法避让且碰撞临近时停车。
  void LocalReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (reference_tf_stop_)
      return;
    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* 检查当前轨迹近期区段的占据碰撞。 */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      const double collision_yaw = estimateYawFromSegment(pos, pos_next);
      const int collision_occ = map->getInflateOccupancy(pos, collision_yaw);
      if (collision_occ)
      {
        Eigen::Vector3d map_origin, map_size;
        map->getRegion(map_origin, map_size);
        RCLCPP_WARN(node_->get_logger(),
                    "[DIAG][COLLISION] traj_id=%d t_cur=%.3f hit_t=%.3f time_to_hit=%.3f "
                    "occ=%d pos=(%.3f,%.3f,%.3f) yaw=%.3f odom=(%.3f,%.3f,%.3f) "
                    "map_min=(%.3f,%.3f,%.3f) map_max=(%.3f,%.3f,%.3f)",
                    info->traj_id_, t_cur, t, t - t_cur, collision_occ,
                    pos.x(), pos.y(), pos.z(), collision_yaw,
                    odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
                    map_origin.x(), map_origin.y(), map_origin.z(),
                    map_origin.x() + map_size.x(), map_origin.y() + map_size.y(),
                    map_origin.z() + map_size.z());
        if (reference_tf_missing_since_.nanoseconds() != 0)
        {
          reference_tf_stop_ = true;
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          changeFSMExecState(EMERGENCY_STOP, "TF_MISSING_COLLISION");
          return;
        }
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  // 选取局部目标、优化 B 样条，成功后同时输出样条与几何路径。
  bool LocalReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    const uint64_t attempt = ++plan_attempt_id_;
    if (!getLocalTarget())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "[DIAG][PLAN id=%llu] target_selection_failed state=%d odom=(%.3f,%.3f,%.3f)",
                           static_cast<unsigned long long>(attempt), static_cast<int>(exec_state_),
                           odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
      return false;
    }

    const bool use_poly_init = have_new_target_ || flag_use_poly_init;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                         "[DIAG][PLAN id=%llu] begin state=%d poly_init=%d random=%d "
                         "start=(%.3f,%.3f,%.3f) target=(%.3f,%.3f,%.3f) "
                         "start_speed=%.3f target_speed=%.3f ref_points=%zu",
                         static_cast<unsigned long long>(attempt), static_cast<int>(exec_state_),
                         use_poly_init, flag_randomPolyTraj,
                         start_pt_.x(), start_pt_.y(), start_pt_.z(),
                         local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z(),
                         start_vel_.norm(), local_target_vel_.norm(), local_reference_path_world_.size());

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
                                       use_poly_init, flag_randomPolyTraj,
                                       local_reference_path_world_);
    have_new_target_ = false;

    if (!plan_success)
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "[DIAG][PLAN id=%llu] failed state=%d start=(%.3f,%.3f,%.3f) "
                           "target=(%.3f,%.3f,%.3f)",
                           static_cast<unsigned long long>(attempt), static_cast<int>(exec_state_),
                           start_pt_.x(), start_pt_.y(), start_pt_.z(),
                           local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z());

    if (plan_success)
    {
      last_reference_to_world_ = planning_reference_to_world_;
      have_last_reference_transform_ = true;

      auto info = &planner_manager_->local_data_;

      /* 发布原 B 样条接口；Path 使用相同位置样条另行求值采样。 */
      local_planner::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);
      updateSampledLocalPath();

      RCLCPP_INFO(node_->get_logger(),
                  "[DIAG][PLAN id=%llu] published traj_id=%d duration=%.3f control_points=%zu "
                  "start=(%.3f,%.3f,%.3f) target=(%.3f,%.3f,%.3f)",
                  static_cast<unsigned long long>(attempt), info->traj_id_, info->duration_,
                  bspline.pos_pts.size(), start_pt_.x(), start_pt_.y(), start_pt_.z(),
                  local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z());

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  // 生成定点停车 B 样条，并以空 Path 通知 path_follow 立即停止。
  bool LocalReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* 发布原 B 样条接口；Path 使用相同位置样条另行求值采样。 */
    local_planner::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);
    local_path_active_ = false;
    publishLocalPath();  // 停车 B 样条对应空 Path，避免跟踪器驶向一个旧的停车点。
    RCLCPP_ERROR(node_->get_logger(),
                 "[DIAG][STOP] published emergency trajectory traj_id=%d stop=(%.3f,%.3f,%.3f) "
                 "state=%d odom_speed=%.3f",
                 info->traj_id_, stop_pos.x(), stop_pos.y(), stop_pos.z(),
                 static_cast<int>(exec_state_), odom_vel_.norm());

    return true;
  }

  // 查找参考路径到规划坐标系的 TF，并检查其时龄。
  bool LocalReplanFSM::lookupReferenceTransform(Eigen::Isometry3d &reference_to_world)
  {
    reference_to_world.setIdentity();
    if (reference_frame_id_ == self_inflation_frame_id_)
      return true;
    if (reference_frame_id_.empty())
      return false;
    try
    {
      const auto transform = tf_buffer_->lookupTransform(
          self_inflation_frame_id_, reference_frame_id_, tf2::TimePointZero);
      const double age = (node_->now() - rclcpp::Time(
          transform.header.stamp, node_->get_clock()->get_clock_type())).seconds();
      if (!referenceTransformFresh(age, reference_tf_max_age_))
        return false;
      reference_to_world = referenceToWorld(transform);
      return true;
    }
    catch (const tf2::TransformException &error)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Reference TF unavailable: %s", error.what());
      return false;
    }
  }

  // 监控 TF 丢失及跳变，决定宽限、停车或按新定位重新规划。
  bool LocalReplanFSM::monitorReferenceTransform()
  {
    if (!have_target_)
      return true;

    Eigen::Isometry3d current;
    if (!lookupReferenceTransform(current))
    {
      const rclcpp::Time now = node_->now();
      if (reference_tf_missing_since_.nanoseconds() == 0)
        reference_tf_missing_since_ = now;
      if (referenceTfGraceExpired((now - reference_tf_missing_since_).seconds(),
                                  reference_tf_grace_) &&
          !reference_tf_stop_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Reference TF missing for %.2f s; stopping",
                     reference_tf_grace_);
        reference_tf_stop_ = true;
        callEmergencyStop(odom_pos_);
        flag_escape_emergency_ = false;
        changeFSMExecState(EMERGENCY_STOP, "TF_TIMEOUT");
      }
      return false;
    }

    reference_tf_missing_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    end_pt_ = current * reference_end_pt_;
    if (exec_state_ == EXEC_TRAJ && have_last_reference_transform_ &&
        referenceTransformJumped(last_reference_to_world_, current, odom_pos_,
                                 reference_tf_jump_translation_, reference_tf_jump_yaw_))
    {
      reference_jump_replan_ = true;
      changeFSMExecState(REPLAN_TRAJ, "TF_CHANGED");
    }
    return true;
  }

  // 沿参考轨迹选择前视目标，调整占据目标并生成本轮世界坐标参考点列。
  bool LocalReplanFSM::getLocalTarget()
  {
    local_reference_path_world_.clear();
    Eigen::Isometry3d reference_to_world = Eigen::Isometry3d::Identity();
    if (!lookupReferenceTransform(reference_to_world))
      return false;
    planning_reference_to_world_ = reference_to_world;
    end_pt_ = reference_to_world * reference_end_pt_;
    const Eigen::Vector3d reference_start = reference_to_world.inverse() * start_pt_;
    auto toWorld = [&](const Eigen::Vector3d &reference_point) {
      return reference_to_world * reference_point;
    };
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    for (double t = 0.0; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist_to_start = (pos_t - reference_start).norm();
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }

    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj);
    local_target_pt_ = end_pt_;

    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      total_dist += (pos_t - prev_pos).norm();
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = toWorld(pos_t);
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      const Eigen::Vector3d original_target = local_target_pt_;
      const int original_occupancy = targetOccupancy(original_target);
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= duration; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= duration)
        {
          Eigen::Vector3d pt = toWorld(planner_manager_->global_data_.getPosition(t_forward));
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = toWorld(planner_manager_->global_data_.getPosition(t_backward));
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[DIAG][TARGET] adjusted occupancy=%d original=(%.3f,%.3f,%.3f) "
                             "adjusted=(%.3f,%.3f,%.3f) projection_t=%.3f target_t=%.3f",
                             original_occupancy, original_target.x(), original_target.y(), original_target.z(),
                             local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z(),
                             t_proj, adjusted_t);
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[DIAG][TARGET] no_free_target occupancy=%d target=(%.3f,%.3f,%.3f) "
                             "projection_t=%.3f target_t=%.3f duration=%.3f",
                             original_occupancy, original_target.x(), original_target.y(), original_target.z(),
                             t_proj, target_t, duration);
        return false;
      }
      if (!target_found && target_t < duration)
      {
        reference_end_pt_ = planner_manager_->global_data_.getPosition(target_t);
        planner_manager_->global_data_.global_duration_ = target_t;
        end_pt_ = local_target_pt_;
      }
    }

    const double braking_distance = max_acc > 1e-6
        ? (max_vel * max_vel) / (2.0 * max_acc)
        : std::numeric_limits<double>::infinity();
    if ((end_pt_ - local_target_pt_).norm() < braking_distance)
    {
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      local_target_vel_ = reference_to_world.linear() * local_target_vel_;
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
    // Keep the whole local reference in world using this planning cycle's TF.
    // A target moved during occupancy adjustment must use its adjusted time.
    const double first_t = std::min(t_proj, target_t);
    const double interval = std::max(0.02, std::min(t_step, (target_t - first_t) / 40.0));
    for (double t = first_t; t < target_t; t += interval)
      local_reference_path_world_.push_back(toWorld(planner_manager_->global_data_.getPosition(t)));
    local_reference_path_world_.push_back(local_target_pt_);
    return true;
  }

} // namespace local_planner
