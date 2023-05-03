#include "los_server/LosServer.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<los_keeper::LosServer>());

  rclcpp::shutdown();
  return 0;
}