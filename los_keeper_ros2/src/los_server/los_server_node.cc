#include "los_keeper_ros2/los_server/los_server.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto los_server = std::make_shared<los_keeper::LosServer>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(los_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}