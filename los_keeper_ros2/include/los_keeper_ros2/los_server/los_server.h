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

using namespace std::chrono_literals;
using DroneStateMsg = los_keeper_msgs::msg::DroneState;
using InputMsg = los_keeper_msgs::msg::JerkControlInput;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudSubscriber = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using StateSubscriber = rclcpp::Subscription<DroneStateMsg>::SharedPtr;
using InputPublisher = rclcpp::Publisher<InputMsg>::SharedPtr;
using RosTimer = rclcpp::TimerBase::SharedPtr;

namespace los_keeper {

DroneState ConverToDroneState(const DroneStateMsg &drone_state_msg);
pcl::PointCloud<pcl::PointXYZ> ConvertToPointCloud(const PointCloudMsg &point_cloud_msg);
InputMsg ConverToInputMsg(const int drone_input);

class LosServer : public rclcpp::Node {
private:
  Wrapper wrapper_;

  PointCloudSubscriber points_subscriber_;
  StateSubscriber state_subscriber_;
  InputPublisher input_publisher_;
  RosTimer planning_timer_;
  RosTimer control_timer_;

  void DroneStateCallback(const DroneStateMsg::SharedPtr msg);
  void PointsCallback(const PointCloudMsg::SharedPtr msg);
  void PlanningTimerCallback();
  void ControlTimerCallback();

public:
  LosServer();
};

} // namespace los_keeper
#endif /* HEADER_LOS_SERVER */
