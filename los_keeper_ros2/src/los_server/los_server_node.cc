#include "los_keeper_ros2/los_server/los_server.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto los_server = std::make_shared<los_keeper::LosServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(los_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}