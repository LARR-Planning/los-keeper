#include "los_keeper_ros2/los_server/los_server.h"

using namespace los_keeper;

DroneState
los_keeper::ConverToDroneState(const DroneStateMsg &drone_state_msg) {
  return DroneState();
}

pcl::PointCloud<pcl::PointXYZ>
los_keeper::ConvertToPointCloud(const PointCloudMsg &point_cloud_msg) {

  pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(point_cloud_msg, *cloud_ptr);

  // TODO(@): convert pcl
  return pcl::PointCloud<pcl::PointXYZ>();
}

InputMsg los_keeper::ConverToInputMsg(const int drone_input) {

  // TODO(@): change argument
  return InputMsg();
}

void LosServer::PlanningTimerCallback() { wrapper_.OnPlanningTimerCallback(); }

void LosServer::ControlTimerCallback() {
  auto t = now();
  auto control_input = wrapper_.GetControlInput(t.seconds());
}

void LosServer::DroneStateCallback(const DroneStateMsg::SharedPtr msg) {
  auto drone_state = ConverToDroneState(*msg);
  wrapper_.SetDroneState(drone_state);
};

void LosServer::PointsCallback(const PointCloudMsg::SharedPtr msg) {
  auto points = ConvertToPointCloud(*msg);
  wrapper_.SetPoints(points);
};

LosServer::LosServer() : Node("los_server_node") {

  rclcpp::SubscriptionOptions options;
  options.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  state_subscriber_ = create_subscription<DroneStateMsg>(
      "~/state", rclcpp::QoS(10),
      std::bind(&LosServer::DroneStateCallback, this, std::placeholders::_1),
      options);

  options.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  points_subscriber_ = create_subscription<PointCloudMsg>(
      "~/points", rclcpp::QoS(10),
      std::bind(&LosServer::PointsCallback, this, std::placeholders::_1),
      options);

  planning_timer_ = this->create_wall_timer(
      10ms, std::bind(&LosServer::PlanningTimerCallback, this));

  control_timer_ = this->create_wall_timer(
      10ms, std::bind(&LosServer::ControlTimerCallback, this));
}
