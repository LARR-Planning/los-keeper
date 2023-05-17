#ifndef HEADER_LOS_SERVER
#define HEADER_LOS_SERVER
#include "los_keeper/wrapper/wrapper.h"
#include "los_keeper_msgs/msg/drone_state.hpp"
#include "los_keeper_msgs/msg/jerk_control_input.hpp"
#include "los_keeper_msgs/msg/object_state.hpp"
#include "los_keeper_msgs/msg/object_state_array.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <mutex>

using namespace std::chrono_literals;
using DroneStateMsg = los_keeper_msgs::msg::DroneState;
using InputMsg = los_keeper_msgs::msg::JerkControlInput;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudSubscriber =
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using StateSubscriber = rclcpp::Subscription<DroneStateMsg>::SharedPtr;
using InputPublisher = rclcpp::Publisher<InputMsg>::SharedPtr;
using RosTimer = rclcpp::TimerBase::SharedPtr;

namespace los_keeper {
class LosServer : public rclcpp::Node {
private:
  Wrapper wrapper_;

  PointCloudSubscriber points_subscriber_;
  StateSubscriber state_subscriber_;
  InputPublisher input_publisher_;

  RosTimer timer_;
  struct {
    std::mutex pose;
    std::mutex pointcloud;
  } mutex_list_;

  void StateCallback(const DroneStateMsg::SharedPtr msg);
  void PointsCallback(const PointCloudMsg::SharedPtr msg);
  void TimerCallback();

  std::string ProcessPointCloud(const std::string &raw_string);

public:
  LosServer();
};

} // namespace los_keeper
#endif /* HEADER_LOS_SERVER */
