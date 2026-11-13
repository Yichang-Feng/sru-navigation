#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/wireless_controller.hpp"

using namespace std::chrono_literals;

class CmdVelToUnitreeBridge : public rclcpp::Node {
public:
  CmdVelToUnitreeBridge() : Node("unitree_bridge"), last_switch_time_(this->now()), last_goal_time_(this->now()) {
    // 参数
    max_speed_ = this->declare_parameter<double>("max_speed", 2.0);
    RCLCPP_INFO(this->get_logger(), "set max speed: %.3f m/s", max_speed_);

    // 发布者
    pub_request_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);
    pub_motion_enabled_ = this->create_publisher<std_msgs::msg::Bool>("/motion_enabled", 10);

    // 订阅者
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelToUnitreeBridge::cmdVelCallback, this, std::placeholders::_1));

    sub_start_nav_ = this->create_subscription<std_msgs::msg::Bool>(
      "startNavi", 10,
      std::bind(&CmdVelToUnitreeBridge::startNavCallback, this, std::placeholders::_1));

    sub_terrain_mode_ = this->create_subscription<std_msgs::msg::Bool>(
      "terrain_mode_switch", 10,
      std::bind(&CmdVelToUnitreeBridge::terrainModeCallback, this, std::placeholders::_1));

    sub_wireless_controller_ = this->create_subscription<unitree_go::msg::WirelessController>(
      "/wirelesscontroller", 10,
      std::bind(&CmdVelToUnitreeBridge::wirelessControllerCallback, this, std::placeholders::_1));

    sub_localization_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization", 10,
      std::bind(&CmdVelToUnitreeBridge::localizationCallback, this, std::placeholders::_1));

    // 延迟 2 秒设置步态模式
    init_timer_ = this->create_wall_timer(2s, std::bind(&CmdVelToUnitreeBridge::initSetTerrainOnce, this));

    RCLCPP_INFO(this->get_logger(), "unitree_bridge node started");
  }

private:
  void startNavCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    start_nav_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), start_nav_ ? "start navigation" : "stop navigation");
  }

  void terrainModeCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    terrain_mode_ = msg->data;
    setTerrainMode(terrain_mode_);
  }

  void wirelessControllerCallback(const unitree_go::msg::WirelessController::SharedPtr msg) {
    // 定义按键联合体
    typedef union {
      struct {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
      } components;
      uint16_t value;
    } xKeySwitchUnion;

    xKeySwitchUnion key;
    key.value = msg->keys;

    // 检测B键是否按下: 停止导航并刹停
    if (key.components.B == 1) {
      auto current_time = this->now();
      auto time_diff = (current_time - last_switch_time_).seconds();

      if (time_diff >= 0.5) {
        joystick_control_ = false;
        last_switch_time_ = current_time;
        std_msgs::msg::Bool msg_motion;
        msg_motion.data = false;
        pub_motion_enabled_->publish(msg_motion);
        RCLCPP_WARN(this->get_logger(), "Button B pressed: motion STOPPED (joystick_control_ = false, /motion_enabled=False)");
      }
    }

    // 检测A键是否按下: 启动/恢复导航
    if (key.components.A == 1) {
      auto current_time = this->now();
      auto time_diff = (current_time - last_switch_time_).seconds();

      if (time_diff >= 0.5) {
        joystick_control_ = true;
        start_nav_ = true;
        last_switch_time_ = current_time;
        std_msgs::msg::Bool msg_motion;
        msg_motion.data = true;
        pub_motion_enabled_->publish(msg_motion);
        RCLCPP_INFO(this->get_logger(), "Button A pressed: motion ENABLED (joystick_control_ = true, /motion_enabled=True)");
      }
    }

    // 检测Y键是否按下，发布当前位置作为goal
    if (key.components.Y == 1 && latest_localization_) {
      auto current_time = this->now();
      auto time_diff = (current_time - last_goal_time_).seconds();

      // 如果距离上次发送goal超过1秒，才允许发送新的goal
      if (time_diff >= 1.0) {
        geometry_msgs::msg::PoseStamped goal;
        goal.header = latest_localization_->header;
        goal.pose = latest_localization_->pose.pose;
        goal.pose.position.z -=0.5;
        pub_goal_->publish(goal);
        last_goal_time_ = current_time;  // 更新上次发送goal的时间
        RCLCPP_INFO(this->get_logger(), "Published goal at position: [%.2f, %.2f, %.2f]",
                    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      }
    }
  }

  void localizationCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_localization_ = msg;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "joystick control: %d, start nav: %d", joystick_control_, start_nav_);

    if (joystick_control_ && start_nav_) {
      double vx   = clamp(msg->linear.x,  -max_speed_, max_speed_);
      double vy   = clamp(msg->linear.y,  -max_speed_, max_speed_);
      double vyaw = clamp(msg->angular.z, -1.0,        1.0);

      const int api_id = 1008; // MOVE
      std::ostringstream oss;
        oss << "{\"x\":" << vx << ",\"y\":" << vy << ",\"z\":" << vyaw << "}";
        publishRequest(api_id, oss.str());
    }
    else {
      double vx   = 0;
      double vy   = 0;
      double vyaw = 0;

      const int api_id = 1008; // MOVE
      std::ostringstream oss;
      oss << "{\"x\":" << vx << ",\"y\":" << vy << ",\"z\":" << vyaw << "}";
      publishRequest(api_id, oss.str());
    }
  }

  void initSetTerrainOnce() {
    if (terrain_inited_) return;
    terrain_inited_ = true;
    setTerrainMode(terrain_mode_);
  }

  void setTerrainMode(bool terrain) {
    const int api_id = 1011; // SWITCH_GAIT
    int gait_type = terrain ? 1 : 0;
    std::ostringstream oss;
    oss << "{\"data\":" << gait_type << "}";
    publishRequest(api_id, oss.str());

    RCLCPP_INFO(this->get_logger(), "set terrain mode to %s (gait type %d)",
                terrain ? "terrain" : "normal", gait_type);
  }

  void publishRequest(int api_id, const std::string &parameter) {
    unitree_api::msg::Request req;

    req.header.identity.id = 0;
    req.header.identity.api_id = api_id;
    req.header.lease.id = 0;
    req.header.policy.priority = 0;
    req.header.policy.noreply = false;

    req.parameter = parameter;
    req.binary.clear();

    pub_request_->publish(req);
  }

  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  // 状态
  bool start_nav_ = false;
  bool joystick_control_ = true;  // 手柄控制使能标志
  bool terrain_mode_ = true;
  bool terrain_inited_ = false;
  double max_speed_ = 2.0;
  rclcpp::Time last_switch_time_;  // 记录上次B键切换的时间
  rclcpp::Time last_goal_time_;  // 记录上次Y键发送goal的时间
  nav_msgs::msg::Odometry::SharedPtr latest_localization_;  // 存储最新的定位数据

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_nav_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_terrain_mode_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr sub_wireless_controller_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_localization_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_request_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motion_enabled_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToUnitreeBridge>());
  rclcpp::shutdown();
  return 0;
}