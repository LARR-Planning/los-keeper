#ifndef HEADER_LOS_SERVER
#define HEADER_LOS_SERVER
#include "los_keeper/wrapper/wrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <std_msgs/msg/string.hpp>

#include <chrono>
using namespace std::chrono_literals;

namespace los_keeper {
class LosServer : public rclcpp::Node {
private:
  Wrapper wrapper_;

  std::string ProcessMessage(const std::string &raw_string);

  rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr short_subscriber_;
  void ShortTopicCallback(const std_msgs::msg::String::SharedPtr msg);
  void LongTopicCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr long_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;
  struct {
    std::mutex long_topic;
    std::mutex short_topic;
  } mutex_list_;

public:
  LosServer();
  bool Update();
  void TimerCallback();
};

} // namespace los_keeper
#endif /* HEADER_LOS_SERVER */
