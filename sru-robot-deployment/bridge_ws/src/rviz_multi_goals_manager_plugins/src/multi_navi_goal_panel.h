#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QColor>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#ifdef HAVE_ROVER_MSGS
#include <rover_msgs/msg/rover_goal_status.hpp>
#endif

namespace rviz_multi_goals_manager_plugins
{

class MultiNaviGoalsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MultiNaviGoalsPanel(QWidget * parent = nullptr);

  // 把里程计话题和到点半径存进 rviz 配置，重启后不用重填
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

public Q_SLOTS:
  void setMaxNumGoal(const QString & maxNumGoal);
  void writePose(geometry_msgs::msg::Pose pose);
  void markPose(const geometry_msgs::msg::PoseStamped::SharedPtr & pose);
  void deleteMark();

protected Q_SLOTS:
  void changeEvent();
  void initPoseTable();
  void updatePoseTable();
  void startNavi();
  void cancelNavi();

  void goalCntCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  void odomCB(const nav_msgs::msg::Odometry::SharedPtr odom);

#ifdef HAVE_ROVER_MSGS
  void statusGlobalPoseCB(const rover_msgs::msg::RoverGoalStatus::SharedPtr poseStatus);
  void statusLocalPoseCB(const rover_msgs::msg::RoverGoalStatus::SharedPtr poseStatus);
#endif

  void checkCycle();
  void toggleTerrainMode();
  void onToleranceChanged();
  void onOdomTopicChanged();
  void onSaveRoute();
  void onLoadRoute();
  void onDemoStart();
  void publishGoalArray();
  void completeNavi();
  void cycleNavi();
  void spinOnce();

  bool checkGoal(const std::vector<action_msgs::msg::GoalStatus> & status_list);

#ifdef HAVE_ROVER_MSGS
  bool checkGoalStatus(
    rover_msgs::msg::RoverGoalStatus globalStatus,
    rover_msgs::msg::RoverGoalStatus localStatus);
#endif

protected:
  // 发布 curGoalIdx_ 指向的目标点，并把该点置为“正在导航”状态
  void publishCurrentGoal();
  // 在当前目标点画/擦到点判定圈
  void markCurrentGoal();
  void clearCurrentGoalMark();
  void subscribeOdom(const QString & topic);
  // 巡视路线的存取（纯文本，可手工编辑）
  bool saveRoute(const QString & path);
  bool loadRoute(const QString & path);
  // 到达当前目标点的唯一入口，内部做边沿检测，一次到达只推进一次
  void onGoalReached();
  // 结束整个导航任务并复位按钮/状态
  void finishNavi();
  void resetRowColors();
  void highlightRow(int row, const QColor & color);

  QLineEdit * output_maxNumGoal_editor_;
  QPushButton * output_maxNumGoal_button_;
  QPushButton * output_reset_button_;
  QPushButton * output_startNavi_button_;
  QPushButton * output_cancel_button_;
  QPushButton * terrain_mode_button_;
  QTableWidget * poseArray_table_;
  QCheckBox * cycle_checkbox_;
  QDoubleSpinBox * arrival_tolerance_spin_;
  QLineEdit * odom_topic_editor_;
  QPushButton * save_route_button_;
  QPushButton * load_route_button_;
  QPushButton * demo_button_;

  QString output_maxNumGoal_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr startNavi_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr terrain_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goal_array_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_3d_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

#ifdef HAVE_ROVER_MSGS
  rclcpp::Subscription<rover_msgs::msg::RoverGoalStatus>::SharedPtr global_goal_status_sub_;
  rclcpp::Subscription<rover_msgs::msg::RoverGoalStatus>::SharedPtr local_goal_status_sub_;
  rover_msgs::msg::RoverGoalStatus globalGoalStatus_;
  rover_msgs::msg::RoverGoalStatus localGoalStatus_;
#endif

  int maxNumGoal_;
  int curGoalIdx_ = 0;   // 当前正在导航的目标点下标
  int cycleCnt_ = 0;     // 已完成的往返段数（每掉一次头 +1）
  int cycleDir_ = 1;     // 往返方向：+1 顺着列表走，-1 倒着走
  bool permit_ = false;  // 导航任务进行中
  bool cycle_ = false;
  bool arrived_ = false;
  bool terrain_mode_ = true;
  bool start_navi_ = false;

  bool navigating_ = false;   // 当前目标点已发出、尚未判定到达
  int arrival_hits_ = 0;      // 连续满足到点距离的帧数
  bool frame_warned_ = false;
  bool has_odom_ = false;
  std::string odom_topic_;
  QString route_path_;
  std::chrono::steady_clock::time_point last_odom_wall_;
  geometry_msgs::msg::Pose current_goal_;

  geometry_msgs::msg::PoseArray pose_array_;
};

}  // namespace rviz_multi_goals_manager_plugins

#endif  // MULTI_NAVI_GOAL_PANEL_H
