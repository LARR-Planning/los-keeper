#include "los_keeper_ros2/los_server/los_server.h"

using namespace los_keeper;

void LosServer::TimerCallback() {
  std::scoped_lock<std::mutex, std::mutex> lock(mutex_list_.pose,
                                                mutex_list_.pointcloud);
  // TODO(Jeon): getter and publish
  // wrapper_.Plan();
  // auto control_input = wrapper_.GetControlInput(now());
  // input_publisher_.publish(ConvertToInputMsg(control_input));
}

void LosServer::StateCallback(const DroneStateMsg::SharedPtr msg){};
void LosServer::PointsCallback(const PointCloudMsg::SharedPtr msg){};

LosServer::LosServer() : Node("los_server_node") {

  rclcpp::SubscriptionOptions options;
  options.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  state_subscriber_ = create_subscription<DroneStateMsg>(
      "pose", rclcpp::QoS(10),
      std::bind(&LosServer::StateCallback, this, std::placeholders::_1),
      options);

  options.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  points_subscriber_ = create_subscription<PointCloudMsg>(
      "points", rclcpp::QoS(10),
      std::bind(&LosServer::PointsCallback, this, std::placeholders::_1),
      options);

  timer_ =
      this->create_wall_timer(1s, std::bind(&LosServer::TimerCallback, this));
}
