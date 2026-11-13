#include <chrono>
#include <sstream>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include <unitree_api/msg/request.hpp>

// G1 Loco API ID
const int32_t ROBOT_API_ID_LOCO_SET_VELOCITY = 7105;

class G1CmdVelBridgeNode : public rclcpp::Node {
public:
  explicit G1CmdVelBridgeNode()
      : Node("g1_cmd_vel_bridge_node"),
        current_vx_(0.0),
        current_vy_(0.0),
        current_omega_(0.0),
        cmd_timeout_(1, 0),
        joystick_control_(true),
        last_switch_time_(this->now()) {

    // Publisher for sport request
    pub_request_ = this->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);

    // Subscribe to /cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&G1CmdVelBridgeNode::cmdVelCallback, this,
                  std::placeholders::_1));

    // Subscribe to /wirelesscontroller topic
    sub_wireless_controller_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,
        std::bind(&G1CmdVelBridgeNode::wirelessControllerCallback, this,
                  std::placeholders::_1));

    // Create a timer to send velocity commands periodically
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&G1CmdVelBridgeNode::timerCallback, this));

    // Initialize last command time
    last_cmd_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
                "G1 CmdVelBridge node started, subscribing to /cmd_vel and /wirelesscontroller");
  }

private:
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

    // 检测B键是否按下，切换控制使能
    if (key.components.B == 1) {
      auto current_time = this->now();
      auto time_diff = (current_time - last_switch_time_).seconds();

      // 如果距离上次切换超过1秒，才允许切换
      if (time_diff >= 1.0) {
        joystick_control_ = !joystick_control_;
        last_switch_time_ = current_time;
        RCLCPP_INFO(this->get_logger(), "Joystick control: %s",
                    joystick_control_ ? "ENABLED" : "DISABLED");
      }
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update current velocities from /cmd_vel
    current_vx_ = msg->linear.x;
    current_vy_ = msg->linear.y;
    current_omega_ = msg->angular.z;
    last_cmd_time_ = this->now();

    RCLCPP_DEBUG(this->get_logger(),
                 "Received cmd_vel: vx=%.2f, vy=%.2f, omega=%.2f",
                 current_vx_, current_vy_, current_omega_);
  }

  void timerCallback() {
    float vx = 0.0;
    float vy = 0.0;
    float omega = 0.0;

    // Only send actual velocities if joystick control is enabled
    if (joystick_control_) {
      // Check if cmd_vel has timed out
      auto time_since_last_cmd = this->now() - last_cmd_time_;

      if (time_since_last_cmd <= cmd_timeout_) {
        vx = current_vx_;
        vy = current_vy_;
        omega = current_omega_;
      } else {
        // Timeout - velocities remain zero
        if (current_vx_ != 0.0 || current_vy_ != 0.0 || current_omega_ != 0.0) {
          RCLCPP_WARN(this->get_logger(), "cmd_vel timeout, stopping robot");
          current_vx_ = 0.0;
          current_vy_ = 0.0;
          current_omega_ = 0.0;
        }
      }
    }

    // Send velocity command to G1
    float duration = 0.2;
    sendVelocityRequest(vx, vy, omega, duration);
  }

  void sendVelocityRequest(float vx, float vy, float omega, float duration) {
    unitree_api::msg::Request req;

    req.header.identity.id = 0;
    req.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY;
    req.header.lease.id = 0;
    req.header.policy.priority = 0;
    req.header.policy.noreply = true;  // 不需要响应

    // 构建JSON参数
    std::ostringstream oss;
    oss << "{\"velocity\":[" << vx << "," << vy << "," << omega << "],"
        << "\"duration\":" << duration << "}";
    req.parameter = oss.str();
    req.binary.clear();

    pub_request_->publish(req);
  }

  // ROS2 publisher, subscriber and timer
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_request_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr sub_wireless_controller_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Current velocity commands
  float current_vx_;
  float current_vy_;
  float current_omega_;

  // Control flags
  bool joystick_control_;  // 手柄控制使能标志

  // Timing
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_switch_time_;
  rclcpp::Duration cmd_timeout_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1CmdVelBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
