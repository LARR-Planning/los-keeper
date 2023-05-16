#ifndef HEADER_LOS_SERVER
#define HEADER_LOS_SERVER
#include "los_keeper/wrapper/wrapper.h"
#include "los_keeper_msgs/msg/drone_state.hpp"
#include "los_keeper_msgs/msg/jerk_control_input.hpp"
#include "los_keeper_msgs/msg/object_state.hpp"
#include "los_keeper_msgs/msg/object_state_array.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <mutex>

using namespace std::chrono_literals;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudSubscriber =
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using PoseSubscriber =
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr;
using RosTimer = rclcpp::TimerBase::SharedPtr;

namespace los_keeper {
class LosServer : public rclcpp::Node {
private:
  Wrapper wrapper_;

  PointCloudSubscriber points_subscriber_;
  PoseSubscriber pose_subscriber_;

  RosTimer timer_;
  struct {
    std::mutex pose;
    std::mutex pointcloud;
  } mutex_list_;

  void PoseCallback(const PoseStampedMsg::SharedPtr msg);
  void PointsCallback(const PointCloudMsg::SharedPtr msg);
  void TimerCallback();

  std::string ProcessPointCloud(const std::string &raw_string);

public:
  LosServer();
};

} // namespace los_keeper
#endif /* HEADER_LOS_SERVER */
