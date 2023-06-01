#include <chrono>
#include <random>

#include "los_keeper_msgs/msg/drone_state.hpp"
#include "los_keeper_msgs/msg/jerk_control_input.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

using DroneStateMsg = los_keeper_msgs::msg::DroneState;
using DroneStateMsgPublisher = rclcpp::Publisher<DroneStateMsg>::SharedPtr;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudMsgPublisher = rclcpp::Publisher<PointCloudMsg>::SharedPtr;

using RosTimer = rclcpp::TimerBase::SharedPtr;

class TestPublisherNode : public rclcpp::Node {
public:
  TestPublisherNode() : Node("test_publisher_node") {
    state_publisher_ = create_publisher<DroneStateMsg>("/los_server_node/state", 10);
    points_publisher_ = create_publisher<PointCloudMsg>("/los_server_node/points", 10);

    short_timer_ = create_wall_timer(10ms, [this]() { PublishStateMessage(); });

    long_timer_ = create_wall_timer(1s, [this]() { PublishPointsMessage(); });
  }

private:
  DroneStateMsgPublisher state_publisher_;
  PointCloudMsgPublisher points_publisher_;

  rclcpp::TimerBase::SharedPtr short_timer_;
  rclcpp::TimerBase::SharedPtr long_timer_;

  void PublishStateMessage() {
    DroneStateMsg message;
    RCLCPP_INFO(get_logger(), "publishing empty drone state.");
    state_publisher_->publish(message);
  }

  void PublishPointsMessage() {
    PointCloudMsg message;
    RCLCPP_INFO(get_logger(), "publishing empty points.");
    points_publisher_->publish(message);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}