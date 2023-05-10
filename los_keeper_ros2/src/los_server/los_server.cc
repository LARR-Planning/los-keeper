#include "los_keeper_ros2/los_server/los_server.h"

using namespace los_keeper;

void LosServer::ShortTopicCallback(const std_msgs::msg::String::SharedPtr msg) {
  auto current_time = now();
  std::string received_string =
      msg->data + " (" + std::to_string(current_time.seconds()) + ")";

  RCLCPP_INFO(this->get_logger(), "Could set short message: %s",
              received_string.c_str());
  this->wrapper_.SetShortString(received_string);
};
void LosServer::LongTopicCallback(const std_msgs::msg::String::SharedPtr msg) {
  auto current_time = now();
  std::string received_string =
      msg->data + " (" + std::to_string(current_time.seconds()) + ")";

  RCLCPP_INFO(this->get_logger(), "Could set long message: %s",
              received_string.c_str());
  this->wrapper_.SetLongString(received_string);
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
