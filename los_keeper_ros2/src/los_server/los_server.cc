#include "los_keeper_ros2/los_server/los_server.h"

using namespace los_keeper;

void LosServer::ShortTopicCallback(const std_msgs::msg::String::SharedPtr msg) {
  auto current_time = now();
  std::string received_string =
      msg->data + " (" + std::to_string(current_time.seconds()) + ")";

  std::unique_lock<std::mutex> lock(mutex_list_.short_topic, std::defer_lock);
  if (lock.try_lock()) {
    RCLCPP_INFO(this->get_logger(), "Could set short message: %s",
                received_string.c_str());
    this->wrapper_.SetShortString(received_string);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Unlock failed when setting short message: %s",
                received_string.c_str());
  }
};
void LosServer::LongTopicCallback(const std_msgs::msg::String::SharedPtr msg) {
  auto current_time = now();
  std::string received_string =
      msg->data + " (" + std::to_string(current_time.seconds()) + ")";

  std::unique_lock<std::mutex> lock(mutex_list_.short_topic, std::defer_lock);
  if (lock.try_lock()) {
    RCLCPP_INFO(this->get_logger(), "Could set long message: %s",
                received_string.c_str());
    this->wrapper_.SetLongString(received_string);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Unlock failed when setting long message: %s",
                received_string.c_str());
  }
};

bool LosServer::Update() { return wrapper_.Plan(); }
void LosServer::TimerCallback() {
  std::scoped_lock<std::mutex, std::mutex> lock(mutex_list_.long_topic,
                                                mutex_list_.short_topic);
  if (Update()) {
    auto concat_string = wrapper_.GetConcatString();
    RCLCPP_INFO(this->get_logger(), "Concat string = %s",
                concat_string.c_str());
  }
}

LosServer::LosServer() : Node("los_server_node") {

  rclcpp::SubscriptionOptions options;
  options.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  short_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/short_topic", rclcpp::QoS(10),
      std::bind(&LosServer::ShortTopicCallback, this, std::placeholders::_1),
      options);

  options.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  long_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/long_topic", rclcpp::QoS(10),
      std::bind(&LosServer::LongTopicCallback, this, std::placeholders::_1),
      options);

  timer_ =
      this->create_wall_timer(1s, std::bind(&LosServer::TimerCallback, this));
}
