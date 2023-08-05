#ifndef HEADER_LOS_SERVER
#define HEADER_LOS_SERVER
#include "los_keeper/wrapper/wrapper.h"
#include "los_keeper_msgs/msg/accel_control_input.hpp"
#include "los_keeper_msgs/msg/chaser_state.hpp"
#include "los_keeper_msgs/msg/jerk_control_input.hpp"
#include "los_keeper_msgs/msg/object_state.hpp"
#include "los_keeper_msgs/msg/object_state_array.hpp"
#include "los_keeper_msgs/msg/velocity_control_input.hpp"
#include "los_keeper_msgs/srv/toggle_activate.hpp"

#include "los_keeper_ros2/visualization/visualizer.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>

using namespace std::chrono_literals;
using KeeperStateMsg = los_keeper_msgs::msg::ChaserState;
using InputMsg = los_keeper_msgs::msg::JerkControlInput;
using VelocityInputMsg = los_keeper_msgs::msg::VelocityControlInput;
using AccelInputMsg = los_keeper_msgs::msg::AccelControlInput;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using ObjectStateMsg = los_keeper_msgs::msg::ObjectState;
using ObjectStateArrayMsg = los_keeper_msgs::msg::ObjectStateArray;
using VisualizationMsg = visualization_msgs::msg::Marker;

using PointCloudSubscriber = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using StateSubscriber = rclcpp::Subscription<KeeperStateMsg>::SharedPtr;
using ObjectStateArraySubscriber = rclcpp::Subscription<ObjectStateArrayMsg>::SharedPtr;
using InputPublisher = rclcpp::Publisher<InputMsg>::SharedPtr;
using VelociyInputPublisher = rclcpp::Publisher<VelocityInputMsg>::SharedPtr;
using AccelInputPublihser = rclcpp::Publisher<AccelInputMsg>::SharedPtr;
using RosTimer = rclcpp::TimerBase::SharedPtr;
using ToggleActivateService = los_keeper_msgs::srv::ToggleActivate;
using ToggleActivateServer = rclcpp::Service<ToggleActivateService>::SharedPtr;

using SomeDebugInfoVisualization = VisualizationMsg;
using SomeDebugInfoPublisher = rclcpp::Publisher<VisualizationMsg>::SharedPtr;

using ObstaclePathVisPublisher = rclcpp::Publisher<ObstaclePathVisualizationMsg>::SharedPtr;
using TargetBestPathVisPublisher = rclcpp::Publisher<TargetBestPathVisualizationMsg>::SharedPtr;
using TargetSafePathVisPublisher = rclcpp::Publisher<TargetSafePathVisualizationMsg>::SharedPtr;
using TargetRawPathVisPublisher = rclcpp::Publisher<TargetSafePathVisualizationMsg>::SharedPtr;
using KeeperRawPathVisPublisher = rclcpp::Publisher<KeeperRawPathVisualizationMsg>::SharedPtr;
using KeeperSafePathVisPublisher = rclcpp::Publisher<KeeperSafePathVisualizationMsg>::SharedPtr;
using FailFlagVisPUblisher = rclcpp::Publisher<FailVisualizationMsg>::SharedPtr;

namespace los_keeper {

KeeperState ConvertToKeeperState(const KeeperStateMsg &keeper_state_msg);
pcl::PointCloud<pcl::PointXYZ> ConvertToPointCloud(const PointCloudMsg &point_cloud_msg);
std::vector<ObjectState>
ConvertToObjectStateArray(const ObjectStateArrayMsg &object_state_array_msg);
InputMsg ConvertToInputMsg(const JerkControlInput &jerk_control_input);
AccelInputMsg ConvertToAccelInputMsg(const AccelControlInput &accel_control_input);
VelocityInputMsg ConvertToVelocityInputMsg(const VelocityControlInput &velocity_control_input);

class LosServer : public rclcpp::Node {
private:
  Wrapper *wrapper_ptr_;
  Visualizer visualizer_;

  string logging_file_name_;
  PointCloudSubscriber points_subscriber_;
  StateSubscriber state_subscriber_;
  ObjectStateArraySubscriber structured_obstacle_state_array_subscriber_;
  ObjectStateArraySubscriber target_state_array_subscriber_;
  InputPublisher input_publisher_;
  VelociyInputPublisher velocity_input_publisher_;
  AccelInputPublihser accel_input_publisher_;

  struct {
    //    SomeDebugInfoVisualization some_debug_info;
    ObstaclePathVisualizationMsg obstacle_path_vis; // Obstacle Path Array  Visualization Data
    TargetBestPathVisualizationMsg
        target_best_path_vis; // Target Best Path Array Visualization Data
    TargetSafePathVisualizationMsg target_safe_path_vis; // Target Safe Path Array Visualization
    TargetRawPathVisualizationMsg target_raw_path_vis;   // Target Primitive Array Visualization
    KeeperRawPathVisualizationMsg keeper_raw_path_vis;   // Keeper Primitive Array Visualization
    KeeperSafePathVisualizationMsg
        keeper_safe_path_vis;           // Keeper Safe Primitive Array Visualization
    FailVisualizationMsg fail_flag_vis; // Fail Flag Visualization

    ObstaclePathVisPublisher
        obstacle_path_vis_publisher; // Obstacle Array Path Visualization Publisher
    TargetBestPathVisPublisher
        target_best_path_vis_publisher; // Target Best Path Visualization Publisher
    TargetSafePathVisPublisher
        target_safe_path_vis_publisher; // Target Safe Path Visualization Publisher
    TargetRawPathVisPublisher
        target_raw_path_vis_publisher; // Target Raw Path Visualization Publisher
    KeeperRawPathVisPublisher
        keeper_raw_path_vis_publisher; // Keeper Raw Path Visualization Publisher
    KeeperSafePathVisPublisher
        keeper_safe_path_vis_publisher;           // Keeper Safe Path Visualization Publisher
    FailFlagVisPUblisher fail_flag_vis_publisher; // Fail Flag Visualization Publisher
  } visualization_;

  RosTimer planning_timer_;
  RosTimer control_timer_;
  rclcpp::CallbackGroup::SharedPtr visualization_callback_group_;

  RosTimer visualization_timer_;

  ToggleActivateServer toggle_activate_server_;

  void KeeperStateCallback(const KeeperStateMsg::SharedPtr msg);
  void PointsCallback(const PointCloudMsg::SharedPtr msg);
  void ObjectStateArrayCallback(const ObjectStateArrayMsg::SharedPtr msg);
  void TargetStateArrayCallback(const ObjectStateArrayMsg::SharedPtr msg);
  void PlanningTimerCallback();
  void ControlTimerCallback();
  void VisualizationTimerCallback();
  void ToggleActivateCallback(const std::shared_ptr<ToggleActivateService::Request> reqeust,
                              std::shared_ptr<ToggleActivateService::Response> response);
  void log_planning();

public:
  LosServer(const rclcpp::NodeOptions &options_input);
  ~LosServer();
};

} // namespace los_keeper
#endif /* HEADER_LOS_SERVER */
