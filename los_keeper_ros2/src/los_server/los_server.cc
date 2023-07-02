#include "los_keeper_ros2/los_server/los_server.h"

using namespace los_keeper;

DroneState los_keeper::ConvertToDroneState(const DroneStateMsg &drone_state_msg) {
  DroneState drone_state;
  drone_state.t_sec =
      drone_state_msg.header.stamp.sec + drone_state_msg.header.stamp.nanosec * 1E-9;

  drone_state.px = drone_state_msg.px;
  drone_state.py = drone_state_msg.py;
  drone_state.pz = drone_state_msg.pz;

  drone_state.vx = drone_state_msg.vx;
  drone_state.vy = drone_state_msg.vy;
  drone_state.vz = drone_state_msg.vz;

  drone_state.ax = drone_state_msg.ax;
  drone_state.ay = drone_state_msg.ay;
  drone_state.az = drone_state_msg.az;

  drone_state.rx = drone_state_msg.rx;
  drone_state.ry = drone_state_msg.ry;
  drone_state.rz = drone_state_msg.rz;

  return drone_state;
}

pcl::PointCloud<pcl::PointXYZ>
los_keeper::ConvertToPointCloud(const PointCloudMsg &point_cloud_msg) {

  pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(point_cloud_msg, *cloud_ptr);

  // TODO(@): convert pcl
  return pcl::PointCloud<pcl::PointXYZ>();
}

std::vector<ObjectState>
los_keeper::ConvertToObjectStateArray(const ObjectStateArrayMsg &object_state_array_msg) {
  std::vector<ObjectState> object_state_array;
  double t_sec =
      object_state_array_msg.header.stamp.sec + object_state_array_msg.header.stamp.nanosec * 1E-9;
  for (const auto &object_state_msg : object_state_array_msg.object_state_array) {
    ObjectState object_state;
    object_state.t_sec = t_sec;
    object_state.id = object_state_msg.id;
    object_state.px = object_state_msg.px;
    object_state.py = object_state_msg.py;
    object_state.pz = object_state_msg.pz;
    object_state.vx = object_state_msg.vx;
    object_state.vy = object_state_msg.vy;
    object_state.vz = object_state_msg.vz;
    object_state.rx = object_state_msg.rx;
    object_state.ry = object_state_msg.ry;
    object_state.rz = object_state_msg.rz;
    object_state_array.push_back(object_state);
  }
  return object_state_array;
}

InputMsg los_keeper::ConvertToInputMsg(const JerkControlInput &jerk_control_input) {
  InputMsg input_msg;
  input_msg.seq = jerk_control_input.seq;
  input_msg.header.stamp.sec = std::floor(jerk_control_input.t_sec);
  input_msg.header.stamp.nanosec =
      std::round((jerk_control_input.t_sec - input_msg.header.stamp.sec) * 1E+9);
  input_msg.jx = jerk_control_input.jx;
  input_msg.jy = jerk_control_input.jy;
  input_msg.jz = jerk_control_input.jz;

  return input_msg;
}

void LosServer::PlanningTimerCallback() { wrapper_ptr_->OnPlanningTimerCallback(); }

void LosServer::ControlTimerCallback() {
  auto t = now();
  auto control_input = wrapper_ptr_->GenerateControlInputFromPlanning(t.seconds());
  if (control_input.has_value()) {
    input_publisher_->publish(ConvertToInputMsg(control_input.value()));
  }
}

void LosServer::VisualizationTimerCallback() {
  auto t = now();
  DebugInfo debug_info = wrapper_ptr_->GetDebugInfo();
  visualizer_.UpdateTime(t); // TODO(@): set time for individual debug info?
  {                          // Obstacle path
    visualization_.obstacle_path_vis = visualizer_.VisualizeObstaclePathArray(
        debug_info.obstacle_manager.structured_obstacle_poly_list);
    visualization_.obstacle_path_vis_publisher->publish(visualization_.obstacle_path_vis);
  }
  { // Target raw path
    visualization_.target_raw_path_vis =
        visualizer_.VisualizeRawTargetPathArray(debug_info.target_manager.primitives_list);
    visualization_.target_raw_path_vis_publisher->publish(visualization_.target_raw_path_vis);
  }
  { // Target safe path
    visualization_.target_safe_path_vis = visualizer_.VisualizeSafeTargetPathArray(
        debug_info.target_manager.primitives_list,
        debug_info.target_manager.primitive_safe_total_index);
    visualization_.target_safe_path_vis_publisher->publish(visualization_.target_safe_path_vis);
  }
  { // Target best path
    visualization_.target_best_path_vis = visualizer_.VisualizeBestTargetPathArray(
        debug_info.target_manager.primitives_list, debug_info.target_manager.primitive_best_index);
    visualization_.target_best_path_vis_publisher->publish(visualization_.target_best_path_vis);
  }
}

void los_keeper::LosServer::ToggleActivateCallback(
    const std::shared_ptr<ToggleActivateService::Request> reqeust,
    std::shared_ptr<ToggleActivateService::Response> response) {
  wrapper_ptr_->OnToggleActivateServiceCallback();
}

void LosServer::DroneStateCallback(const DroneStateMsg::SharedPtr msg) {
  auto drone_state = ConvertToDroneState(*msg);
  wrapper_ptr_->SetDroneState(drone_state);
};

void LosServer::PointsCallback(const PointCloudMsg::SharedPtr msg) {
  auto points = ConvertToPointCloud(*msg);
  wrapper_ptr_->SetPoints(points);
}

void los_keeper::LosServer::ObjectStateArrayCallback(const ObjectStateArrayMsg::SharedPtr msg) {
  const auto object_state_array = ConvertToObjectStateArray(*msg);
  wrapper_ptr_->SetObjectStateArray(object_state_array);
};

void LosServer::TargetStateArrayCallback(const ObjectStateArrayMsg::SharedPtr msg) {
  const auto target_state_array = ConvertToObjectStateArray(*msg);
  wrapper_ptr_->SetTargetStateArray(target_state_array);
}

LosServer::LosServer(const rclcpp::NodeOptions &options_input)
    : Node("los_server_node", options_input) {
  rclcpp::SubscriptionOptions options;
  {
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_subscriber_ = create_subscription<DroneStateMsg>(
        "~/state", rclcpp::QoS(1),
        std::bind(&LosServer::DroneStateCallback, this, std::placeholders::_1), options);

    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    points_subscriber_ = create_subscription<PointCloudMsg>(
        "~/points", rclcpp::QoS(1),
        std::bind(&LosServer::PointsCallback, this, std::placeholders::_1), options);

    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    structured_obstacle_state_array_subscriber_ = create_subscription<ObjectStateArrayMsg>(
        "~/object_state_array", rclcpp::QoS(1),
        std::bind(&LosServer::ObjectStateArrayCallback, this, std::placeholders::_1), options);
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    target_state_array_subscriber_ = create_subscription<ObjectStateArrayMsg>(
        "~/target_state_array", rclcpp::QoS(1),
        std::bind(&LosServer::TargetStateArrayCallback, this, std::placeholders::_1), options);
  }
  {
    //        options.callback_group =
    //        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); state_subscriber_
    //        = create_subscription<DroneStateMsg>(
    //            "/drone_state", rclcpp::QoS(1),
    //            std::bind(&LosServer::DroneStateCallback, this, std::placeholders::_1), options);
    //        options.callback_group =
    //        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //        points_subscriber_ = create_subscription<PointCloudMsg>(
    //            "/point_cloud", rclcpp::QoS(1),
    //            std::bind(&LosServer::PointsCallback, this, std::placeholders::_1), options);
    //        options.callback_group =
    //        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //        structured_obstacle_state_array_subscriber_ =
    //        create_subscription<ObjectStateArrayMsg>(
    //            "/obstacle_state_list", rclcpp::QoS(1),
    //            std::bind(&LosServer::ObjectStateArrayCallback, this, std::placeholders::_1),
    //            options);
    //        options.callback_group =
    //        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //        target_state_array_subscriber_ = create_subscription<ObjectStateArrayMsg>(
    //            "/target_state_list", rclcpp::QoS(1),
    //            std::bind(&LosServer::TargetStateArrayCallback, this, std::placeholders::_1),
    //            options);
  }
  visualization_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  visualization_timer_ = this->create_wall_timer(
      50ms, std::bind(&LosServer::VisualizationTimerCallback, this), visualization_callback_group_);
  visualization_.obstacle_path_vis_publisher = create_publisher<ObstaclePathVisualizationMsg>(
      "~/visualization/obstacle_array_info", rclcpp::QoS(1));
  visualization_.target_best_path_vis_publisher = create_publisher<TargetBestPathVisualizationMsg>(
      "~/visualization/target_best_array_info", rclcpp::QoS(1));
  visualization_.target_safe_path_vis_publisher = create_publisher<TargetSafePathVisualizationMsg>(
      "~/visualization/target_safe_array_info", rclcpp::QoS(1));
  visualization_.target_raw_path_vis_publisher = create_publisher<TargetRawPathVisualizationMsg>(
      "~/visualization/target_raw_array_info", rclcpp::QoS(1));

  input_publisher_ = create_publisher<InputMsg>("~/input", rclcpp::QoS(1));

  control_timer_ = this->create_wall_timer(10ms, std::bind(&LosServer::ControlTimerCallback, this));

  toggle_activate_server_ = this->create_service<ToggleActivateService>(
      "~/toggle_activate", std::bind(&LosServer::ToggleActivateCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));

  // Parameter Settings
  ObstacleParameter obstacle_param;
  PredictionParameter prediction_param;
  PlanningParameter planning_param;
  ProblemParameter problem_param;
  { // Parameter Settings for Problem
    get_parameter<bool>("problem.is_2d", problem_param.is_2d);
    get_parameter<double>("problem.replanning_period", problem_param.replanning_period);
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
    get_parameter<float>("trajectory_planner.horizon.planning", planning_param.horizon.planning);
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
    get_parameter<float>("trajectory_planner.safe_distance.rx", planning_param.safe_distance.rx);
    get_parameter<float>("trajectory_planner.safe_distance.ry", planning_param.safe_distance.ry);
    get_parameter<float>("trajectory_planner.safe_distance.rz", planning_param.safe_distance.rz);
    get_parameter<float>("trajectory_planner.virtual_pcl_bbox.width",
                         planning_param.virtual_pcl_bbox.width);
    get_parameter<float>("trajectory_planner.virtual_pcl_bbox.height",
                         planning_param.virtual_pcl_bbox.height);
  }
  Parameters parameters;
  parameters.obstacle = obstacle_param;
  parameters.prediction = prediction_param;
  parameters.planning = planning_param;
  parameters.problem = problem_param;
  wrapper_ptr_ = new Wrapper(parameters);

  VisualizationParameters parameters_vis;
  {
    get_parameter<string>("frame_id", parameters_vis.frame_id);
    get_parameter<bool>("obstacle.publish", parameters_vis.obstacle.publish);
    get_parameter<int>("obstacle.num_time_sample", parameters_vis.obstacle.num_time_sample);
    get_parameter<float>("obstacle.line_scale", parameters_vis.obstacle.line_scale);
    get_parameter<float>("obstacle.color.a", parameters_vis.obstacle.color.a);
    get_parameter<float>("obstacle.color.r", parameters_vis.obstacle.color.r);
    get_parameter<float>("obstacle.color.g", parameters_vis.obstacle.color.g);
    get_parameter<float>("obstacle.color.b", parameters_vis.obstacle.color.b);

    get_parameter<bool>("target.raw.publish", parameters_vis.target.raw.publish);
    get_parameter<float>("target.raw.proportion", parameters_vis.target.raw.proportion);
    get_parameter<int>("target.raw.num_time_sample", parameters_vis.target.raw.num_time_sample);
    get_parameter<float>("target.raw.color.a", parameters_vis.target.raw.color.a);
    get_parameter<float>("target.raw.color.r", parameters_vis.target.raw.color.r);
    get_parameter<float>("target.raw.color.g", parameters_vis.target.raw.color.g);
    get_parameter<float>("target.raw.color.b", parameters_vis.target.raw.color.b);
    get_parameter<float>("target.raw.line_scale", parameters_vis.target.raw.line_scale);

    get_parameter<bool>("target.safe.publish", parameters_vis.target.safe.publish);
    get_parameter<float>("target.safe.proportion", parameters_vis.target.safe.proportion);
    get_parameter<int>("target.safe.num_time_sample", parameters_vis.target.safe.num_time_sample);
    get_parameter<float>("target.safe.color.a", parameters_vis.target.safe.color.a);
    get_parameter<float>("target.safe.color.r", parameters_vis.target.safe.color.r);
    get_parameter<float>("target.safe.color.g", parameters_vis.target.safe.color.g);
    get_parameter<float>("target.safe.color.b", parameters_vis.target.safe.color.b);
    get_parameter<float>("target.safe.line_scale", parameters_vis.target.safe.line_scale);

    get_parameter<bool>("target.best.publish", parameters_vis.target.best.publish);
    get_parameter<int>("target.best.num_time_sample", parameters_vis.target.best.num_time_sample);
    get_parameter<float>("target.best.color.a", parameters_vis.target.best.color.a);
    get_parameter<float>("target.best.color.r", parameters_vis.target.best.color.r);
    get_parameter<float>("target.best.color.g", parameters_vis.target.best.color.g);
    get_parameter<float>("target.best.color.b", parameters_vis.target.best.color.b);
    get_parameter<float>("target.best.line_scale", parameters_vis.target.best.line_scale);
  }
  visualizer_.UpdateParam(parameters_vis);
  planning_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(problem_param.replanning_period),
                              std::bind(&LosServer::PlanningTimerCallback, this));
}