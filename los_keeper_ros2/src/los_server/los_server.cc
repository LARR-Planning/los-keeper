#include "los_keeper_ros2/los_server/los_server.h"

using namespace los_keeper;

DroneState los_keeper::ConvertToDroneState(const DroneStateMsg &drone_state_msg) {
  return DroneState();
}

pcl::PointCloud<pcl::PointXYZ>
los_keeper::ConvertToPointCloud(const PointCloudMsg &point_cloud_msg) {

  pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(point_cloud_msg, *cloud_ptr);

  // TODO(@): convert pcl
  return pcl::PointCloud<pcl::PointXYZ>();
}

InputMsg los_keeper::ConvertToInputMsg(const int drone_input) {

  // TODO(@): change argument
  return InputMsg();
}

void LosServer::PlanningTimerCallback() { wrapper_.OnPlanningTimerCallback(); }

void LosServer::ControlTimerCallback() {
  auto t = now();
  auto control_input = wrapper_.GenerateControlInputFromPlanning(t.seconds());
}

void LosServer::DroneStateCallback(const DroneStateMsg::SharedPtr msg) {
  auto drone_state = ConvertToDroneState(*msg);
  wrapper_.SetDroneState(drone_state);
};

void LosServer::PointsCallback(const PointCloudMsg::SharedPtr msg) {
  auto points = ConvertToPointCloud(*msg);
  wrapper_.SetPoints(points);
};

LosServer::LosServer(const rclcpp::NodeOptions &options_input)
    : Node("los_server_node", options_input) {

  rclcpp::SubscriptionOptions options;
  options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  state_subscriber_ = create_subscription<DroneStateMsg>(
      "~/state", rclcpp::QoS(10),
      std::bind(&LosServer::DroneStateCallback, this, std::placeholders::_1), options);

  options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  points_subscriber_ = create_subscription<PointCloudMsg>(
      "~/points", rclcpp::QoS(10),
      std::bind(&LosServer::PointsCallback, this, std::placeholders::_1), options);

  planning_timer_ =
      this->create_wall_timer(10ms, std::bind(&LosServer::PlanningTimerCallback, this));

  control_timer_ = this->create_wall_timer(10ms, std::bind(&LosServer::ControlTimerCallback, this));

  // TODO(Jeon): replaced by wrapper's parameter struct

  ObstacleParam obstacle_param;
  PredictionParam prediction_param;
  PlanningParam planning_param;
  ProblemParam problem_param;
  { // Parameter Settings for Problem
    get_parameter<bool>("problem.is_2d", problem_param.is_2d);
  }
  { // Parameter Settings for ObstacleManager
    get_parameter<float>("obstacle_manager.planning_horizon", obstacle_param.planning_horizon);
  }
  { // Parameter Settings for TargetManager
    get_parameter<int>("target_manager.sampling.num_sample", prediction_param.sampling.num_sample);
    get_parameter<int>("target_manager.sampling.num_thread", prediction_param.sampling.num_thread);
    get_parameter<bool>("target_manager.sampling.is_lite", prediction_param.sampling.is_lite);
    get_parameter<float>("target_manager.horizon.prediction", prediction_param.horizon.prediction);
    get_parameter<float>("target_manager.dynamic_limits.vel_max",
                         prediction_param.dynamic_limits.vel_max);
    get_parameter<float>("target_manager.dynamic_limits.acc_max",
                         prediction_param.dynamic_limits.acc_max);
    get_parameter<float>("target_manager.distance.obstacle_max",
                         prediction_param.distance.obstacle_max);
    get_parameter<float>("target_manager.virtual_pcl_bbox.width",
                         prediction_param.virtual_pcl_bbox.width);
    get_parameter<float>("target_manager.virtual_pcl_bbox.height",
                         prediction_param.virtual_pcl_bbox.height);
  }
  { // Parameter Settings for TrajectoryPlanner
    get_parameter<int>("trajectory_planner.sampling.num_sample",
                       planning_param.sampling.num_sample);
    get_parameter<int>("trajectory_planner.sampling.num_thread",
                       planning_param.sampling.num_thread);
    get_parameter<bool>("trajectory_planner.sampling.is_lite", planning_param.sampling.is_lite);
    get_parameter<float>("trajectory_planner.horizon.prediction", planning_param.horizon.planning);
    get_parameter<float>("trajectory_planner.distance.obstacle_max",
                         planning_param.distance.obstacle_max);
    get_parameter<float>("trajectory_planner.distance.target_min",
                         planning_param.distance.target_min);
    get_parameter<float>("trajectory_planner.distance.target_max",
                         planning_param.distance.target_max);
    get_parameter<float>("trajectory_planner.dynamic_limits.vel_max",
                         planning_param.dynamic_limits.vel_max);
    get_parameter<float>("trajectory_planner.dynamic_limits.acc_max",
                         planning_param.dynamic_limits.acc_max);
    get_parameter<float>("trajectory_planner.drone.rx", planning_param.drone.rx);
    get_parameter<float>("trajectory_planner.drone.ry", planning_param.drone.ry);
    get_parameter<float>("trajectory_planner.drone.rz", planning_param.drone.rz);
    get_parameter<float>("trajectory_planner.virtual_pcl_bbox.width",
                         planning_param.virtual_pcl_bbox.width);
    get_parameter<float>("trajectory_planner.virtual_pcl_bbox.height",
                         planning_param.virtual_pcl_bbox.height);
  }
  wrapper_.SetParameters(problem_param, obstacle_param, prediction_param, planning_param);
  //  Wrapper aa(problem_param,obstacle_param,prediction_param,planning_param);
}
