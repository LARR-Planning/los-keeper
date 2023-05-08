#include "los_keeper_ros2/los_server/los_server.h"

using namespace los_keeper;

void LosServer::ProcessMessage(const std_msgs::msg::String::SharedPtr msg) {
  std::thread::id this_id = std::this_thread::get_id();
  std::ostringstream oss;
  oss << std::this_thread::get_id();

  RCLCPP_INFO(this->get_logger(), "Thread  %s is processing %s...",
              oss.str().c_str(), msg->data.c_str());

  size_t num_chars = msg->data.size();
  rclcpp::sleep_for(std::chrono::seconds(num_chars));
  RCLCPP_INFO(this->get_logger(), "Thread  %s finished processing %s.",
              oss.str().c_str(), msg->data.c_str());
}

void LosServer::ShortTopicCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received short message: %s",
              msg->data.c_str());
  ProcessMessage(msg);
  this->wrapper_.SetShortString(msg->data);
};
void LosServer::LongTopicCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received short message: %s",
              msg->data.c_str());
  ProcessMessage(msg);
  this->wrapper_.SetLongString(msg->data);
};

bool LosServer::Update() { return wrapper_.Plan(); }
void LosServer::TimerCallback() {
  if (Update()) {
    auto concat_string = wrapper_.GetConcatString();
    RCLCPP_INFO(this->get_logger(), "Concat string = %s",
                concat_string.c_str());
  }
}
LosServer::LosServer() : Node("los_server_node") {
  reentrant_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions options;
  options.callback_group = reentrant_callback_group_;

  short_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/short_topic", rclcpp::QoS(10),
      std::bind(&LosServer::ShortTopicCallback, this, std::placeholders::_1),
      options);

  long_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/long_topic", rclcpp::QoS(10),
      std::bind(&LosServer::LongTopicCallback, this, std::placeholders::_1),
      options);

  timer_ =
      this->create_wall_timer(1s, std::bind(&LosServer::TimerCallback, this),
                              reentrant_callback_group_);
}
