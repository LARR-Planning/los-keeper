#include <chrono>
#include <random>

#include "los_keeper_msgs/msg/drone_state.hpp"
#include "los_keeper_msgs/msg/jerk_control_input.hpp"
#include "los_keeper_msgs/msg/object_state_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

using DroneStateMsg = los_keeper_msgs::msg::DroneState;
using DroneStateMsgPublisher = rclcpp::Publisher<DroneStateMsg>::SharedPtr;
using ObjectStateMsg = los_keeper_msgs::msg::ObjectState;
using ObjectStateMsgArray = los_keeper_msgs::msg::ObjectStateArray;
using ObjectStateMsgArrayPublisher = rclcpp::Publisher<ObjectStateMsgArray>::SharedPtr;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudMsgPublisher = rclcpp::Publisher<PointCloudMsg>::SharedPtr;

using RosTimer = rclcpp::TimerBase::SharedPtr;

class TestPublisherNode : public rclcpp::Node {
public:
  TestPublisherNode() : Node("test_publisher_node") {
    state_publisher_ = create_publisher<DroneStateMsg>("~/state", 10);
    points_publisher_ = create_publisher<PointCloudMsg>("~/points", 10);
    objects_publisher_ = create_publisher<ObjectStateMsgArray>("~/target_state_array", 10);

    short_timer_ = create_wall_timer(10ms, [this]() { PublishStateMessage(); });

    long_timer_ = create_wall_timer(1s, [this]() {
      PublishPointsMessage();
      PublishObjectsMessage();
    });
  }

private:
  DroneStateMsgPublisher state_publisher_;
  PointCloudMsgPublisher points_publisher_;
  ObjectStateMsgArrayPublisher objects_publisher_;

  rclcpp::TimerBase::SharedPtr short_timer_;
  rclcpp::TimerBase::SharedPtr long_timer_;

  void PublishStateMessage() {
    DroneStateMsg message;
    message.header.stamp = now();
    state_publisher_->publish(message);
  }

  void PublishPointsMessage() {
    PointCloudMsg message;
    // TODO(@): testing with non-empty pointcloud
    points_publisher_->publish(message);
  }

  void PublishObjectsMessage() {
    ObjectStateMsgArray array_message;
    ObjectStateMsg message;
    array_message.header.stamp = now();
    array_message.object_state_array.push_back(message);
    array_message.object_state_array.push_back(message);
    objects_publisher_->publish(array_message);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}