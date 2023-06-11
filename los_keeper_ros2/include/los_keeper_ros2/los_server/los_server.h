#ifndef HEADER_LOS_SERVER
#define HEADER_LOS_SERVER
#include "los_keeper/wrapper/wrapper.h"
#include "los_keeper_msgs/msg/drone_state.hpp"
#include "los_keeper_msgs/msg/jerk_control_input.hpp"
#include "los_keeper_msgs/msg/object_state.hpp"
#include "los_keeper_msgs/msg/object_state_array.hpp"
#include "los_keeper_msgs/srv/toggle_activate.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>

using namespace std::chrono_literals;
using DroneStateMsg = los_keeper_msgs::msg::DroneState;
using InputMsg = los_keeper_msgs::msg::JerkControlInput;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using ObjectStateMsg = los_keeper_msgs::msg::ObjectState;
using ObjectStateArrayMsg = los_keeper_msgs::msg::ObjectStateArray;

using PointCloudSubscriber = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using StateSubscriber = rclcpp::Subscription<DroneStateMsg>::SharedPtr;
using ObjectStateArraySubscriber = rclcpp::Subscription<ObjectStateArrayMsg>::SharedPtr;
using InputPublisher = rclcpp::Publisher<InputMsg>::SharedPtr;
using RosTimer = rclcpp::TimerBase::SharedPtr;
using ToggleActivateService = los_keeper_msgs::srv::ToggleActivate;
using ToggleActivateServer = rclcpp::Service<ToggleActivateService>::SharedPtr;

namespace los_keeper {

DroneState ConvertToDroneState(const DroneStateMsg &drone_state_msg);
pcl::PointCloud<pcl::PointXYZ> ConvertToPointCloud(const PointCloudMsg &point_cloud_msg);
std::vector<ObjectState>
ConvertToObjectStateArray(const ObjectStateArrayMsg &object_state_array_msg);
InputMsg ConvertToInputMsg(const JerkControlInput &jerk_control_input);

class LosServer : public rclcpp::Node {
private:
  Wrapper *wrapper_ptr_;

  PointCloudSubscriber points_subscriber_;
  StateSubscriber state_subscriber_;
  ObjectStateArraySubscriber structured_obstacle_state_array_subscriber_;
  ObjectStateArraySubscriber target_state_array_subscriber_;
  InputPublisher input_publisher_;

  RosTimer planning_timer_;
  RosTimer control_timer_;

  ToggleActivateServer toggle_activate_server_;

  void DroneStateCallback(const DroneStateMsg::SharedPtr msg);
  void PointsCallback(const PointCloudMsg::SharedPtr msg);
  void ObjectStateArrayCallback(const ObjectStateArrayMsg::SharedPtr msg);
  void TargetStateArrayCallback(const ObjectStateArrayMsg::SharedPtr msg);
  void PlanningTimerCallback();
  void ControlTimerCallback();
  void ToggleActivateCallback(const std::shared_ptr<ToggleActivateService::Request> reqeust,
                              std::shared_ptr<ToggleActivateService::Response> response);

public:
  LosServer(const rclcpp::NodeOptions &options_input);
};

} // namespace los_keeper
#endif /* HEADER_LOS_SERVER */
