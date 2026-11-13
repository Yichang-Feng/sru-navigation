#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class SportModeSubscriber : public rclcpp::Node {
public:
  SportModeSubscriber() : Node("sport_mode_subscriber") {
    subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/lf/sportmodestate", 10, 
      std::bind(&SportModeSubscriber::topic_callback, this, _1));

    pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_lf", 10);
    
    RCLCPP_INFO(this->get_logger(), "SportModeSubscriber node started");
  }

private:
  void topic_callback(const unitree_go::msg::SportModeState::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received SportModeState message:");
    // RCLCPP_INFO(this->get_logger(), "Velocity: [%.2f, %.2f, %.2f]",
    //             msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = msg->velocity[0];
    velocity.linear.y = msg->velocity[1];
    velocity.angular.z = msg->velocity[2];
    pub_velocity_->publish(velocity);
    }

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SportModeSubscriber>());
  rclcpp::shutdown();
  return 0;
} 