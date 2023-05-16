#include "los_keeper_ros2/los_server/los_server.h"

using namespace los_keeper;

void LosServer::TimerCallback() {
  std::scoped_lock<std::mutex, std::mutex> lock(mutex_list_.pose,
                                                mutex_list_.pointcloud);
}

void LosServer::PoseCallback(const PoseStampedMsg::SharedPtr msg){};
void LosServer::PointsCallback(const PointCloudMsg::SharedPtr msg){};

LosServer::LosServer() : Node("los_server_node") {

  rclcpp::SubscriptionOptions options;
  options.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  pose_subscriber_ = create_subscription<PoseStampedMsg>(
      "pose", rclcpp::QoS(10),
      std::bind(&LosServer::PoseCallback, this, std::placeholders::_1),
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
