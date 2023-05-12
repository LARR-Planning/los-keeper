#include <chrono>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class TestPublisherNode : public rclcpp::Node {
public:
  TestPublisherNode() : Node("test_publisher_node") {
    short_publisher_ =
        create_publisher<std_msgs::msg::String>("/short_topic", 10);
    long_publisher_ =
        create_publisher<std_msgs::msg::String>("/long_topic", 10);

    short_timer_ = create_wall_timer(10ms, [this]() { PublishShortMessage(); });

    long_timer_ = create_wall_timer(1s, [this]() { PublishLongMessage(); });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr short_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr long_publisher_;

  rclcpp::TimerBase::SharedPtr short_timer_;
  rclcpp::TimerBase::SharedPtr long_timer_;

  void PublishShortMessage() {
    std_msgs::msg::String message;
    message.data = "Short message";
    short_publisher_->publish(message);
    short_timer_->reset();
  }

  void PublishLongMessage() {
    std_msgs::msg::String message;
    message.data = "Long message with more characters";
    long_publisher_->publish(message);
    long_timer_->reset();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}