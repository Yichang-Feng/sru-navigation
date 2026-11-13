#include "multi_navi_goal_panel.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <QDebug>
#include <QDir>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QTableWidget>
#include <QHeaderView>
#include <QPixmap>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QBrush>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rviz_multi_goals_manager_plugins
{

namespace
{
// 连续多少帧落在到点半径内才判定到达，避免定位抖动误触发
constexpr int kArrivalConfirmCount = 3;
const QColor kOddLapColor(255, 69, 0);     // 奇数圈：橙
const QColor kEvenLapColor(100, 149, 237); // 偶数圈：蓝

// ---- 目标点箭头的外观，按需调这里 ----
constexpr double kArrowLength = 1.2;   // 箭头总长(m)
constexpr double kArrowShaft  = 0.18;  // 杆直径(m)
constexpr double kArrowHead   = 0.35;  // 箭头直径(m)
constexpr double kArrowLift   = 0.5;   // 抬离地面的高度(m)，避免被点云盖住
// 亮品红，和 rviz_goal_tool_plugins 的 3D Nav Goal 拖拽箭头同色
constexpr float kArrowR = 1.0f;
constexpr float kArrowG = 0.1f;
constexpr float kArrowB = 0.9f;
constexpr float kArrowA = 1.0f;

// ---- 序号标签 ----
constexpr double kTextSize = 0.45;  // 字高(m)
constexpr double kTextLift = 0.45;  // 在箭头之上再抬多少(m)

// ---- 当前目标点的到点判定圈 ----
constexpr double kDiscLift = 0.05;       // 离地高度(m)
constexpr double kDiscThickness = 0.05;  // 圆盘厚度(m)
constexpr float kDiscR = 0.1f;           // 亮绿
constexpr float kDiscG = 1.0f;
constexpr float kDiscB = 0.3f;
constexpr float kDiscA = 0.45f;          // 半透明，不挡住底下的点云

constexpr char kDefaultOdomTopic[] = "/lio/robo/odom";
constexpr double kOdomTimeoutSec = 3.0;  // 超过这么久没收到里程计就报警

// 巡视路线默认存放位置，展演时无需再选文件
QString defaultRoutePath()
{
  return QDir::homePath() + "/.ros/multi_goals_route.txt";
}
}  // namespace

MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget * parent)
: rviz_common::Panel(parent)
, maxNumGoal_(1)
{
  // ROS2 节点在 onInitialize 中通过 context 获取，构造时先创建临时节点
  node_ = rclcpp::Node::make_shared("multi_navi_goal_panel");

  if (this->parentWidget()) {
    this->parentWidget()->setWindowTitle("多点导航");
  }
  this->setName("多点导航");

  // 三个入口都往同一个列表里加点：
  //   /goal      —— 自定义/桥接来的目标点
  //   /goal_pose —— RViz2 内置 "2D Goal Pose" 工具
  //   /goal_3d   —— 本仓库 rviz_goal_tool_plugins 的 "3D Nav Goal" 工具（带高度）
  auto add_goal = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goalCntCB(msg);
  };
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal", 1, add_goal);
  goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 1, add_goal);
  goal_3d_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_3d", 1, add_goal);

  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("cur_goal", 1);
  // 队列要能装下一次 markPose 的箭头+文字，以及当前目标点的判定圈
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
  startNavi_pub_ = node_->create_publisher<std_msgs::msg::Bool>("startNavi", 1);
  terrain_mode_pub_ = node_->create_publisher<std_msgs::msg::Bool>("terrain_mode_switch", 1);
  goal_array_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("multi_goal_list", 1);

#ifdef HAVE_ROVER_MSGS
  global_goal_status_sub_ = node_->create_subscription<rover_msgs::msg::RoverGoalStatus>(
    "/cur_global_goal_status", 1,
    [this](const rover_msgs::msg::RoverGoalStatus::SharedPtr msg) {
      statusGlobalPoseCB(msg);
    });
  local_goal_status_sub_ = node_->create_subscription<rover_msgs::msg::RoverGoalStatus>(
    "/cur_local_goal_status", 1,
    [this](const rover_msgs::msg::RoverGoalStatus::SharedPtr msg) {
      statusLocalPoseCB(msg);
    });
#endif

  // ---- UI ----
  QVBoxLayout * root_layout = new QVBoxLayout;

  QHBoxLayout * maxNumGoal_layout = new QHBoxLayout;
  output_maxNumGoal_button_ = new QPushButton("多点模式");
  output_maxNumGoal_button_->setToolTip("切换到多点导航模式");
  maxNumGoal_layout->addWidget(output_maxNumGoal_button_);

  QLabel * image_label = new QLabel();
  image_label->setAlignment(Qt::AlignCenter);
  try {
    std::string pkg_share =
      ament_index_cpp::get_package_share_directory("rviz_multi_goals_manager_plugins");
    QPixmap pixmap(QString::fromStdString(pkg_share + "/image/logo.png"));
    if (!pixmap.isNull()) {
      image_label->setPixmap(pixmap.scaled(300, 300, Qt::KeepAspectRatio));
    } else {
      image_label->setText("无法加载图片");
    }
  } catch (const std::exception &) {
    image_label->setText("包路径未找到");
  }
  maxNumGoal_layout->addWidget(image_label);
  root_layout->addLayout(maxNumGoal_layout);

  cycle_checkbox_ = new QCheckBox("开启循环");
  cycle_checkbox_->setToolTip("往返循环：走到最后一个点后原路返回，1→2→3→2→1→2… 无限往返");
  root_layout->addWidget(cycle_checkbox_);

  QHBoxLayout * odom_layout = new QHBoxLayout;
  QLabel * odom_label = new QLabel("里程计话题");
  odom_topic_editor_ = new QLineEdit(kDefaultOdomTopic);
  odom_topic_editor_->setToolTip("用于到点判定的 nav_msgs/Odometry 话题，回车生效");
  odom_layout->addWidget(odom_label);
  odom_layout->addWidget(odom_topic_editor_);
  root_layout->addLayout(odom_layout);
  subscribeOdom(odom_topic_editor_->text());

  QHBoxLayout * tolerance_layout = new QHBoxLayout;
  QLabel * tolerance_label = new QLabel("到点半径(m)");
  arrival_tolerance_spin_ = new QDoubleSpinBox;
  arrival_tolerance_spin_->setRange(0.1, 5.0);
  arrival_tolerance_spin_->setSingleStep(0.1);
  arrival_tolerance_spin_->setValue(0.75);
  arrival_tolerance_spin_->setToolTip("机器人与当前目标点水平距离小于该值即判定到达");
  tolerance_layout->addWidget(tolerance_label);
  tolerance_layout->addWidget(arrival_tolerance_spin_);
  root_layout->addLayout(tolerance_layout);

  terrain_mode_button_ = new QPushButton("地形模式");
  terrain_mode_button_->setToolTip("切换步态模式（地形模式/普通模式）");
  root_layout->addWidget(terrain_mode_button_);

  poseArray_table_ = new QTableWidget;
  initPoseTable();
  poseArray_table_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  root_layout->addWidget(poseArray_table_);

  QHBoxLayout * route_layout = new QHBoxLayout;
  save_route_button_ = new QPushButton("保存路线");
  save_route_button_->setToolTip("把当前目标点列表存成路线文件");
  load_route_button_ = new QPushButton("加载路线");
  load_route_button_->setToolTip("从路线文件读回目标点列表");
  demo_button_ = new QPushButton("一键展演");
  demo_button_->setToolTip("加载巡视路线并立即开始往返巡视");
  demo_button_->setStyleSheet(
    "QPushButton { background-color: #E91E63; color: white; font-weight: bold; }");
  route_layout->addWidget(save_route_button_);
  route_layout->addWidget(load_route_button_);
  route_layout->addWidget(demo_button_);
  root_layout->addLayout(route_layout);

  QHBoxLayout * manipulate_layout = new QHBoxLayout;
  output_reset_button_ = new QPushButton("重置列表");
  output_reset_button_->setToolTip("清空目标点列表");
  manipulate_layout->addWidget(output_reset_button_);

  output_startNavi_button_ = new QPushButton("开始导航");
  output_startNavi_button_->setToolTip("启动导航任务");
  manipulate_layout->addWidget(output_startNavi_button_);
  manipulate_layout->setSpacing(10);
  manipulate_layout->setContentsMargins(5, 5, 5, 5);
  root_layout->addLayout(manipulate_layout);

  setLayout(root_layout);

  // 到点判定依赖里程计回调，spin 周期决定判定的时间分辨率
  QTimer * output_timer = new QTimer(this);
  output_timer->start(100);
  setMaxNumGoal("200");

  connect(output_maxNumGoal_button_, SIGNAL(clicked()), this, SLOT(changeEvent()));
  connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
  connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
  connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
  connect(terrain_mode_button_, SIGNAL(clicked()), this, SLOT(toggleTerrainMode()));
  connect(arrival_tolerance_spin_, SIGNAL(valueChanged(double)), this, SLOT(onToleranceChanged()));
  connect(odom_topic_editor_, SIGNAL(editingFinished()), this, SLOT(onOdomTopicChanged()));
  connect(save_route_button_, SIGNAL(clicked()), this, SLOT(onSaveRoute()));
  connect(load_route_button_, SIGNAL(clicked()), this, SLOT(onLoadRoute()));
  connect(demo_button_, SIGNAL(clicked()), this, SLOT(onDemoStart()));
  connect(output_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));

  // 启动即把上次存的巡视路线摆好，展演时只需按“一键展演”。
  // 这里只加载不启动 —— 开机就让机器人自己走起来太危险。
  route_path_ = defaultRoutePath();
  if (QFileInfo::exists(route_path_)) {
    loadRoute(route_path_);
  }
}

void MultiNaviGoalsPanel::changeEvent()
{
  Q_EMIT configChanged();
}

void MultiNaviGoalsPanel::setMaxNumGoal(const QString & new_maxNumGoal)
{
  if (new_maxNumGoal != output_maxNumGoal_) {
    output_maxNumGoal_ = new_maxNumGoal;
    if (output_maxNumGoal_.isEmpty()) {
      maxNumGoal_ = 1;
    } else {
      maxNumGoal_ = output_maxNumGoal_.toInt();
    }
    Q_EMIT configChanged();
  }
}

void MultiNaviGoalsPanel::initPoseTable()
{
  curGoalIdx_ = 0;
  cycleCnt_ = 0;
  cycleDir_ = 1;
  permit_ = false;
  cycle_ = false;
  navigating_ = false;
  arrival_hits_ = 0;
  start_navi_ = false;
  poseArray_table_->clear();
  pose_array_.poses.clear();
  deleteMark();
  publishGoalArray();

  poseArray_table_->setRowCount(200);
  poseArray_table_->setColumnCount(3);
  poseArray_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  QStringList pose_header;
  pose_header << "x" << "y" << "yaw";
  poseArray_table_->setHorizontalHeaderLabels(pose_header);
  cycle_checkbox_->setCheckState(Qt::Unchecked);
}

void MultiNaviGoalsPanel::deleteMark()
{
  visualization_msgs::msg::Marker marker_delete;
  marker_delete.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_pub_->publish(marker_delete);
}

void MultiNaviGoalsPanel::updatePoseTable()
{
  poseArray_table_->setRowCount(maxNumGoal_);
  QStringList pose_header;
  pose_header << "x" << "y" << "yaw";
  poseArray_table_->setHorizontalHeaderLabels(pose_header);
  poseArray_table_->show();
}

void MultiNaviGoalsPanel::goalCntCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  if (static_cast<int>(pose_array_.poses.size()) < maxNumGoal_) {
    pose_array_.poses.push_back(pose->pose);
    pose_array_.header.frame_id = pose->header.frame_id;
    writePose(pose->pose);
    markPose(pose);
    publishGoalArray();
    RCLCPP_INFO(node_->get_logger(),
      "添加目标点，当前共 %zu 个", pose_array_.poses.size());
  } else {
    RCLCPP_ERROR(node_->get_logger(),
      "超出最大目标点数量: %d", maxNumGoal_);
  }
}

void MultiNaviGoalsPanel::writePose(geometry_msgs::msg::Pose pose)
{
  int row = static_cast<int>(pose_array_.poses.size()) - 1;
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  double yaw = tf2::getYaw(q);

  poseArray_table_->setItem(row, 0,
    new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
  poseArray_table_->setItem(row, 1,
    new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
  poseArray_table_->setItem(row, 2,
    new QTableWidgetItem(QString::number(yaw * 180.0 / M_PI, 'f', 2)));
}

void MultiNaviGoalsPanel::markPose(const geometry_msgs::msg::PoseStamped::SharedPtr & pose)
{
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = pose->header.frame_id;
  arrow.header.stamp = node_->now();
  arrow.ns = "navi_point_arrow";
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.pose = pose->pose;
  arrow.scale.x = kArrowLength;
  arrow.scale.y = kArrowShaft;
  arrow.scale.z = kArrowHead;
  arrow.pose.position.z += kArrowLift;
  arrow.color.r = kArrowR;
  arrow.color.g = kArrowG;
  arrow.color.b = kArrowB;
  arrow.color.a = kArrowA;
  const int idx = static_cast<int>(pose_array_.poses.size());
  arrow.id = idx;
  marker_pub_->publish(arrow);

  // 序号标签，堆在一起的点靠它区分先后
  visualization_msgs::msg::Marker text;
  text.header = arrow.header;
  text.ns = "navi_point_text";
  text.action = visualization_msgs::msg::Marker::ADD;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.id = idx;
  text.pose.position = pose->pose.position;
  text.pose.position.z += kArrowLift + kTextLift;
  text.pose.orientation.w = 1.0;
  text.scale.z = kTextSize;  // TEXT_VIEW_FACING 只用 scale.z 作字高
  text.color.r = 1.0f;
  text.color.g = 1.0f;
  text.color.b = 1.0f;
  text.color.a = 1.0f;
  text.text = std::to_string(idx);
  marker_pub_->publish(text);
}

// 在当前目标点画一个半径等于“到点半径”的半透明圆盘。
// 机器人进圈即判定到达，所以这个圈同时也是判定阈值的可视化。
void MultiNaviGoalsPanel::markCurrentGoal()
{
  visualization_msgs::msg::Marker disc;
  disc.header.frame_id = pose_array_.header.frame_id;
  disc.header.stamp = node_->now();
  disc.ns = "navi_current_goal";
  disc.id = 0;
  disc.action = visualization_msgs::msg::Marker::ADD;
  disc.type = visualization_msgs::msg::Marker::CYLINDER;

  disc.pose.position = current_goal_.position;
  disc.pose.position.z += kDiscLift;
  disc.pose.orientation.w = 1.0;  // 保持水平，不跟随目标点的朝向

  const double diameter = 2.0 * arrival_tolerance_spin_->value();
  disc.scale.x = diameter;
  disc.scale.y = diameter;
  disc.scale.z = kDiscThickness;

  disc.color.r = kDiscR;
  disc.color.g = kDiscG;
  disc.color.b = kDiscB;
  disc.color.a = kDiscA;
  marker_pub_->publish(disc);
}

void MultiNaviGoalsPanel::clearCurrentGoalMark()
{
  visualization_msgs::msg::Marker del;
  del.header.frame_id = pose_array_.header.frame_id;
  del.header.stamp = node_->now();
  del.ns = "navi_current_goal";
  del.id = 0;
  del.action = visualization_msgs::msg::Marker::DELETE;
  marker_pub_->publish(del);
}

// 导航中改到点半径时让圆盘跟着变，方便边看边调
void MultiNaviGoalsPanel::onToleranceChanged()
{
  if (permit_ && navigating_) {
    markCurrentGoal();
  }
}

void MultiNaviGoalsPanel::checkCycle()
{
  cycle_ = cycle_checkbox_->isChecked();
  RCLCPP_INFO(node_->get_logger(), "循环模式: %s", cycle_ ? "开启" : "关闭");
}

void MultiNaviGoalsPanel::resetRowColors()
{
  for (int r = 0; r < static_cast<int>(pose_array_.poses.size()); ++r) {
    for (int c = 0; c < 3; ++c) {
      QTableWidgetItem * item = poseArray_table_->item(r, c);
      if (item) {item->setBackground(QBrush());}
    }
  }
}

void MultiNaviGoalsPanel::highlightRow(int row, const QColor & color)
{
  for (int c = 0; c < 3; ++c) {
    QTableWidgetItem * item = poseArray_table_->item(row, c);
    if (item) {item->setBackground(color);}
  }
}

void MultiNaviGoalsPanel::publishCurrentGoal()
{
  const int size = static_cast<int>(pose_array_.poses.size());
  if (size == 0 || curGoalIdx_ < 0 || curGoalIdx_ >= size) {
    RCLCPP_ERROR(node_->get_logger(), "目标点索引 %d 越界（共 %d 个）", curGoalIdx_, size);
    return;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header = pose_array_.header;
  goal.header.stamp = node_->now();
  goal.pose = pose_array_.poses.at(curGoalIdx_);
  goal_pub_->publish(goal);

  current_goal_ = goal.pose;
  navigating_ = true;
  arrival_hits_ = 0;
  permit_ = true;

  // 只保留当前点的高亮；颜色随掉头交替，橙=正向、蓝=反向，一眼看出走的哪一程
  resetRowColors();
  highlightRow(curGoalIdx_, (cycleCnt_ % 2 == 0) ? kOddLapColor : kEvenLapColor);
  markCurrentGoal();

  RCLCPP_INFO(node_->get_logger(), "导航到目标点 %d/%d（%s）",
    curGoalIdx_ + 1, size,
    cycle_ ? (cycleDir_ > 0 ? "往返-正向" : "往返-反向") : "单程");
}

// 到达当前目标点的唯一入口。navigating_ 在这里被清掉，直到下一个点发出前
// 重复到达的状态消息都会被忽略——这就是防止索引被刷穿的边沿检测。
void MultiNaviGoalsPanel::onGoalReached()
{
  if (!permit_ || !navigating_) {return;}

  navigating_ = false;
  arrival_hits_ = 0;
  RCLCPP_INFO(node_->get_logger(), "目标点 %d 已到达", curGoalIdx_ + 1);

  if (cycle_) {
    cycleNavi();
  } else {
    completeNavi();
  }
}

void MultiNaviGoalsPanel::odomCB(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // 放在 permit_ 判断之前，好让看门狗能区分“没收到里程计”和“收到了但没在导航”
  has_odom_ = true;
  last_odom_wall_ = std::chrono::steady_clock::now();

  if (!permit_ || !navigating_) {return;}

  if (!frame_warned_ && !odom->header.frame_id.empty() &&
    !pose_array_.header.frame_id.empty() &&
    odom->header.frame_id != pose_array_.header.frame_id)
  {
    frame_warned_ = true;
    RCLCPP_WARN(node_->get_logger(),
      "定位坐标系(%s)与目标点坐标系(%s)不一致，到点判定可能不准确",
      odom->header.frame_id.c_str(), pose_array_.header.frame_id.c_str());
  }

  const double dx = odom->pose.pose.position.x - current_goal_.position.x;
  const double dy = odom->pose.pose.position.y - current_goal_.position.y;
  const double dist = std::hypot(dx, dy);
  const double tol = arrival_tolerance_spin_->value();

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "距目标点 %d 还有 %.2f m（到点半径 %.2f m，已连续 %d/%d 帧在圈内）",
    curGoalIdx_ + 1, dist, tol, arrival_hits_, kArrivalConfirmCount);

  if (dist <= tol) {
    arrival_hits_ += 1;
    if (arrival_hits_ >= kArrivalConfirmCount) {
      onGoalReached();
    }
  } else {
    arrival_hits_ = 0;
  }
}

void MultiNaviGoalsPanel::startNavi()
{
  if (output_startNavi_button_->text() != tr("开始导航")) {
    RCLCPP_INFO(node_->get_logger(), "导航已手动停止");
    finishNavi();
    return;
  }

  if (pose_array_.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "目标点列表为空，请先添加目标点");
    return;
  }

  start_navi_ = true;
  output_startNavi_button_->setText(tr("停止导航"));
  output_reset_button_->setEnabled(false);

  std_msgs::msg::Bool start_msg;
  start_msg.data = true;
  startNavi_pub_->publish(start_msg);

  curGoalIdx_ = 0;
  cycleCnt_ = 0;
  cycleDir_ = 1;
  publishCurrentGoal();
}

void MultiNaviGoalsPanel::cancelNavi()
{
  finishNavi();
}

// 结束任务的唯一出口：手动停止、走完全部点都走这里，保证 permit_/navigating_
// 一定被清掉，否则后续状态消息还会继续推进目标点。
void MultiNaviGoalsPanel::finishNavi()
{
  permit_ = false;
  navigating_ = false;
  arrival_hits_ = 0;
  start_navi_ = false;
  curGoalIdx_ = 0;
  cycleCnt_ = 0;
  cycleDir_ = 1;

  std_msgs::msg::Bool stop_msg;
  stop_msg.data = false;
  startNavi_pub_->publish(stop_msg);

  resetRowColors();
  clearCurrentGoalMark();
  output_startNavi_button_->setText(tr("开始导航"));
  output_reset_button_->setEnabled(true);
}

void MultiNaviGoalsPanel::completeNavi()
{
  const int size = static_cast<int>(pose_array_.poses.size());
  if (size == 0) {
    finishNavi();
    return;
  }

  if (curGoalIdx_ + 1 < size) {
    curGoalIdx_ += 1;
    publishCurrentGoal();
  } else {
    RCLCPP_INFO(node_->get_logger(), "所有目标点已完成");
    finishNavi();
  }
}

// 往返循环：走到列表两端就掉头原路返回。
// 3 个点的序列是 1→2→3→2→1→2→3→… 而不是绕回起点的 1→2→3→1→2→3。
void MultiNaviGoalsPanel::cycleNavi()
{
  const int size = static_cast<int>(pose_array_.poses.size());
  if (size == 0) {
    finishNavi();
    return;
  }

  if (size == 1) {
    cycleCnt_ += 1;
    publishCurrentGoal();  // 只有一个点，无处可返，原地重发
    return;
  }

  const int next = curGoalIdx_ + cycleDir_;
  if (next >= size || next < 0) {
    cycleDir_ = -cycleDir_;
    cycleCnt_ += 1;
    RCLCPP_INFO(node_->get_logger(), "到达端点，掉头%s（第 %d 段）",
      cycleDir_ > 0 ? "正向" : "反向", cycleCnt_);
  }

  curGoalIdx_ += cycleDir_;
  publishCurrentGoal();
}

#ifdef HAVE_ROVER_MSGS
void MultiNaviGoalsPanel::statusGlobalPoseCB(
  const rover_msgs::msg::RoverGoalStatus::SharedPtr poseStatus)
{
  globalGoalStatus_ = *poseStatus;
  if (globalGoalStatus_.status == 1 && start_navi_) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    startNavi_pub_->publish(msg);
  }
  if (globalGoalStatus_.status == 5) {return;}
  if (globalGoalStatus_.status == 3) {
    onGoalReached();
  }
}

void MultiNaviGoalsPanel::statusLocalPoseCB(
  const rover_msgs::msg::RoverGoalStatus::SharedPtr poseStatus)
{
  localGoalStatus_ = *poseStatus;
  arrived_ = checkGoalStatus(globalGoalStatus_, localGoalStatus_);
  if (arrived_) {
    onGoalReached();
  }
}

bool MultiNaviGoalsPanel::checkGoalStatus(
  rover_msgs::msg::RoverGoalStatus globalStatus,
  rover_msgs::msg::RoverGoalStatus /*localStatus*/)
{
  return globalStatus.status == 3;
}
#endif

bool MultiNaviGoalsPanel::checkGoal(
  const std::vector<action_msgs::msg::GoalStatus> & status_list)
{
  bool done = false;
  if (status_list.empty()) {
    RCLCPP_INFO(node_->get_logger(), "请输入导航目标点");
    return false;
  }
  for (const auto & s : status_list) {
    if (s.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      done = true;
      RCLCPP_INFO(node_->get_logger(), "目标点 %d 已到达", curGoalIdx_);
    } else if (s.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
      RCLCPP_ERROR(node_->get_logger(), "目标点 %d 无效，跳到下一个", curGoalIdx_);
      return true;
    } else if (s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
      done = false;
    } else {
      done = false;
    }
  }
  return done;
}

void MultiNaviGoalsPanel::toggleTerrainMode()
{
  terrain_mode_ = !terrain_mode_;
  std_msgs::msg::Bool msg;
  msg.data = terrain_mode_;
  terrain_mode_pub_->publish(msg);
  if (terrain_mode_) {
    terrain_mode_button_->setText("地形模式");
    terrain_mode_button_->setStyleSheet(
      "QPushButton { background-color: #4CAF50; color: white; }");
  } else {
    terrain_mode_button_->setText("普通模式");
    terrain_mode_button_->setStyleSheet(
      "QPushButton { background-color: #2196F3; color: white; }");
  }
}

void MultiNaviGoalsPanel::publishGoalArray()
{
  if (!pose_array_.poses.empty()) {
    pose_array_.header.stamp = node_->now();
    goal_array_pub_->publish(pose_array_);
  }
}

bool MultiNaviGoalsPanel::saveRoute(const QString & path)
{
  if (pose_array_.poses.empty()) {return false;}

  QDir().mkpath(QFileInfo(path).absolutePath());
  std::ofstream ofs(path.toStdString());
  if (!ofs) {return false;}

  ofs << "# rviz_multi_goals_manager_plugins 巡视路线\n";
  ofs << "# 每行一个点: x y z qx qy qz qw（可手工编辑）\n";
  ofs << "frame_id: "
      << (pose_array_.header.frame_id.empty() ? "map" : pose_array_.header.frame_id) << "\n";
  ofs << std::fixed << std::setprecision(6);
  for (const auto & p : pose_array_.poses) {
    ofs << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' '
        << p.orientation.x << ' ' << p.orientation.y << ' '
        << p.orientation.z << ' ' << p.orientation.w << '\n';
  }
  return ofs.good();
}

bool MultiNaviGoalsPanel::loadRoute(const QString & path)
{
  std::ifstream ifs(path.toStdString());
  if (!ifs) {
    RCLCPP_WARN(node_->get_logger(), "打不开路线文件: %s", path.toStdString().c_str());
    return false;
  }

  std::string frame = "map";
  std::vector<geometry_msgs::msg::Pose> poses;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    if (line.rfind("frame_id:", 0) == 0) {
      std::istringstream fs(line.substr(9));
      fs >> frame;
      continue;
    }
    std::istringstream ls(line);
    geometry_msgs::msg::Pose p;
    if (!(ls >> p.position.x >> p.position.y >> p.position.z >>
      p.orientation.x >> p.orientation.y >> p.orientation.z >> p.orientation.w))
    {
      RCLCPP_WARN(node_->get_logger(), "路线文件有无法解析的行，已跳过: %s", line.c_str());
      continue;
    }
    poses.push_back(p);
  }

  if (poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "路线文件里没有有效目标点: %s",
      path.toStdString().c_str());
    return false;
  }

  initPoseTable();  // 清空表格、点位、marker
  for (const auto & p : poses) {
    auto ps = std::make_shared<geometry_msgs::msg::PoseStamped>();
    ps->header.frame_id = frame;
    ps->header.stamp = node_->now();
    ps->pose = p;
    goalCntCB(ps);  // 复用手动下点的路径：写表格 + 发 marker + 发 multi_goal_list
  }

  route_path_ = path;
  RCLCPP_INFO(node_->get_logger(), "已加载巡视路线 %s（%zu 个点）",
    path.toStdString().c_str(), poses.size());
  return true;
}

void MultiNaviGoalsPanel::onSaveRoute()
{
  if (pose_array_.poses.empty()) {
    QMessageBox::warning(this, "保存路线", "目标点列表是空的，先下几个点再保存。");
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
    this, "保存巡视路线", route_path_.isEmpty() ? defaultRoutePath() : route_path_,
    "路线文件 (*.txt);;所有文件 (*)");
  if (path.isEmpty()) {return;}

  if (saveRoute(path)) {
    route_path_ = path;
    Q_EMIT configChanged();
    RCLCPP_INFO(node_->get_logger(), "巡视路线已保存到 %s（%zu 个点）",
      path.toStdString().c_str(), pose_array_.poses.size());
  } else {
    QMessageBox::warning(this, "保存路线", "写入失败：" + path);
  }
}

void MultiNaviGoalsPanel::onLoadRoute()
{
  const QString path = QFileDialog::getOpenFileName(
    this, "加载巡视路线", route_path_.isEmpty() ? defaultRoutePath() : route_path_,
    "路线文件 (*.txt);;所有文件 (*)");
  if (path.isEmpty()) {return;}

  if (loadRoute(path)) {
    Q_EMIT configChanged();
  } else {
    QMessageBox::warning(this, "加载路线", "读取失败或文件里没有有效目标点：" + path);
  }
}

// 展演入口：点位还没摆好就先加载，然后勾上往返循环并立刻出发
void MultiNaviGoalsPanel::onDemoStart()
{
  if (pose_array_.poses.empty() && !loadRoute(route_path_)) {
    QMessageBox::warning(this, "一键展演",
      "没有可用的巡视路线。\n请先手动下几个点，按“保存路线”存到：\n" + defaultRoutePath());
    return;
  }

  cycle_checkbox_->setChecked(true);
  cycle_ = true;

  if (output_startNavi_button_->text() == tr("开始导航")) {
    startNavi();
  } else {
    RCLCPP_INFO(node_->get_logger(), "已在巡视中，忽略重复的展演请求");
  }
}

// 重新订阅里程计。给 odom_sub_ 赋新值会释放旧订阅，等同于退订。
void MultiNaviGoalsPanel::subscribeOdom(const QString & topic)
{
  const std::string t = topic.trimmed().toStdString();
  if (t.empty()) {return;}

  odom_topic_ = t;
  has_odom_ = false;
  frame_warned_ = false;

  // BEST_EFFORT 订阅者能收 RELIABLE 和 BEST_EFFORT 两种发布方；
  // 反过来 RELIABLE 订阅者收不到 BEST_EFFORT 发布方（QoS 不兼容，一条都收不到）。
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    t, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odomCB(msg);
    });
  RCLCPP_INFO(node_->get_logger(), "里程计话题已设为: %s", t.c_str());
}

void MultiNaviGoalsPanel::onOdomTopicChanged()
{
  if (odom_topic_editor_->text().trimmed().toStdString() != odom_topic_) {
    subscribeOdom(odom_topic_editor_->text());
    Q_EMIT configChanged();
  }
}

void MultiNaviGoalsPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("OdomTopic", odom_topic_editor_->text());
  config.mapSetValue("ArrivalTolerance", arrival_tolerance_spin_->value());
  config.mapSetValue("RoutePath", route_path_);
}

void MultiNaviGoalsPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString topic;
  if (config.mapGetString("OdomTopic", &topic) && !topic.trimmed().isEmpty()) {
    odom_topic_editor_->setText(topic);
    subscribeOdom(topic);
  }

  float tolerance = 0.0f;
  if (config.mapGetFloat("ArrivalTolerance", &tolerance) && tolerance > 0.0f) {
    arrival_tolerance_spin_->setValue(tolerance);
  }

  QString route;
  if (config.mapGetString("RoutePath", &route) && !route.trimmed().isEmpty() &&
    route != route_path_ && QFileInfo::exists(route))
  {
    loadRoute(route);
  }
}

void MultiNaviGoalsPanel::spinOnce()
{
  if (!rclcpp::ok()) {return;}
  rclcpp::spin_some(node_);

  // 导航中却收不到里程计 —— 到点判定不可能触发，直接把原因喊出来
  if (!permit_ || !navigating_) {return;}

  if (!has_odom_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "导航中，但从未收到 %s。到点判定无法工作，不会推进到下一个点。"
      "请检查话题名与 QoS：ros2 topic info %s -v", odom_topic_.c_str(), odom_topic_.c_str());
    return;
  }

  const double idle = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_odom_wall_).count();
  if (idle > kOdomTimeoutSec) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "已 %.1f 秒没有收到 %s，到点判定已停摆", idle, odom_topic_.c_str());
  }
}

}  // namespace rviz_multi_goals_manager_plugins

PLUGINLIB_EXPORT_CLASS(
  rviz_multi_goals_manager_plugins::MultiNaviGoalsPanel, rviz_common::Panel)
