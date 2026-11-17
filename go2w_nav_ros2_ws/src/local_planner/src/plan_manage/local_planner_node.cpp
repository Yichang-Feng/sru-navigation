// 中文导读：节点入口：初始化 ROS 和局部规划状态机，在单线程执行器中处理规划、建图和路径输出回调。

#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <local_planner/plan_manage/local_replan_fsm.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("local_planner_node");

  try
  {
    local_planner::LocalReplanFSM planner;
    planner.init(node.get());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(node->get_logger(), "[DIAG][CRASH] unhandled std::exception: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  catch (...)
  {
    RCLCPP_FATAL(node->get_logger(), "[DIAG][CRASH] unhandled non-standard exception");
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::shutdown();
  return 0;
}
