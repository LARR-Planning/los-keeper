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

AccelInputMsg los_keeper::ConvertToAccelInputMsg(const AccelControlInput &accel_control_input) {
  AccelInputMsg input_msg;
  input_msg.seq = accel_control_input.seq;
  input_msg.header.stamp.sec = std::floor(accel_control_input.t_sec);
  input_msg.header.stamp.nanosec =
      std::round((accel_control_input.t_sec - input_msg.header.stamp.sec) * 1E+9);
  input_msg.ax = accel_control_input.ax;
  input_msg.ay = accel_control_input.ay;
  input_msg.az = accel_control_input.az;
  return input_msg;
}

VelocityInputMsg
los_keeper::ConvertToVelocityInputMsg(const VelocityControlInput &velocity_control_input) {
  VelocityInputMsg input_msg;
  input_msg.seq = velocity_control_input.seq;
  input_msg.header.stamp.sec = std::floor(velocity_control_input.t_sec);
  input_msg.header.stamp.nanosec =
      std::round((velocity_control_input.t_sec - input_msg.header.stamp.sec) * 1E+9);
  input_msg.vx = velocity_control_input.vx;
  input_msg.vy = velocity_control_input.vy;
  input_msg.vz = velocity_control_input.vz;
  return input_msg;
}

void LosServer::PlanningTimerCallback() { wrapper_ptr_->OnPlanningTimerCallback(); }

void LosServer::ControlTimerCallback() {
  auto t = now();
  auto jerk_control_input = wrapper_ptr_->GenerateControlInputFromPlanning(t.seconds());
  if (jerk_control_input.has_value()) {
    input_publisher_->publish(ConvertToInputMsg(jerk_control_input.value()));
  } else {
    JerkControlInput jerk_input_at_fail;
    jerk_input_at_fail.t_sec = t.seconds();
    jerk_input_at_fail.jx = 0.0f;
    jerk_input_at_fail.jy = 0.0f;
    jerk_input_at_fail.jz = 0.0f;
    input_publisher_->publish(ConvertToInputMsg(jerk_input_at_fail));
  }
  auto velocity_control_input = wrapper_ptr_->GenerateVelocityControlInputFromPlanning(t.seconds());
  if (velocity_control_input.has_value()) {
    velocity_input_publisher_->publish(ConvertToVelocityInputMsg(velocity_control_input.value()));
  } else {
    VelocityControlInput velocity_input_at_fail;
    velocity_input_at_fail.t_sec = t.seconds();
    velocity_input_at_fail.vx = 0.0f;
    velocity_input_at_fail.vy = 0.0f;
    velocity_input_at_fail.vz = 0.0f;
    velocity_input_publisher_->publish(ConvertToVelocityInputMsg(velocity_input_at_fail));
  }
  auto accel_control_input = wrapper_ptr_->GenerateAccelControlInputFromPlanning(t.seconds());
  if (velocity_control_input.has_value()) {
    accel_input_publisher_->publish(ConvertToAccelInputMsg(accel_control_input.value()));
  } else {
    AccelControlInput accel_input_at_fail;
    accel_input_at_fail.t_sec = t.seconds();
    accel_input_at_fail.ax = 0.0f;
    accel_input_at_fail.ay = 0.0f;
    accel_input_at_fail.az = 0.0f;
    accel_input_publisher_->publish(ConvertToAccelInputMsg(accel_input_at_fail));
  }
}

void LosServer::VisualizationTimerCallback() {
  auto t = now();
  DebugInfo debug_info = wrapper_ptr_->GetDebugInfo();
  visualizer_.UpdateTime(t); // TODO(@): set time for individual debug info?
  visualization_.obstacle_path_vis = visualizer_.VisualizeObstaclePathArray(
      debug_info.obstacle_manager.structured_obstacle_poly_list); // obstacle path
  visualization_.obstacle_path_vis_publisher->publish(visualization_.obstacle_path_vis);
  visualization_.target_raw_path_vis = visualizer_.VisualizeRawTargetPathArray(
      debug_info.target_manager.primitives_list); // Target prediction primitives (raw)
  visualization_.target_raw_path_vis_publisher->publish(visualization_.target_raw_path_vis);
  visualization_.target_safe_path_vis = visualizer_.VisualizeSafeTargetPathArray(
      debug_info.target_manager.primitives_list,
      debug_info.target_manager
          .primitive_safe_total_index); // Target prediction primitives (no collision)
  visualization_.target_safe_path_vis_publisher->publish(visualization_.target_safe_path_vis);
  visualization_.target_best_path_vis = visualizer_.VisualizeBestTargetPathArray(
      debug_info.target_manager.primitives_list,
      debug_info.target_manager.primitive_best_index); // Target prediction results
  visualization_.target_best_path_vis_publisher->publish(visualization_.target_best_path_vis);
  visualization_.keeper_raw_path_vis = visualizer_.VisualizeRawKeeperPathArray(
      debug_info.planning.primitives_list); // Keeper primitives (raw)
  visualization_.keeper_raw_path_vis_publisher->publish(visualization_.keeper_raw_path_vis);
  visualization_.keeper_safe_path_vis = visualizer_.VisualizeSafeKeeperPathArray(
      debug_info.planning.primitives_list,
      debug_info.planning.safe_visibility_index); // Keeper primitives (safe and target visible)
  visualization_.keeper_safe_path_vis_publisher->publish(visualization_.keeper_safe_path_vis);
  visualization_.fail_flag_vis = visualizer_.VisualizeFailFlagList(
      debug_info.target_manager.success_flag, debug_info.planning.success_flag,
      debug_info.target_manager.seq); // Fail flag
  visualization_.fail_flag_vis_publisher->publish(visualization_.fail_flag_vis);
  //  printf("PREDICTION TIME: %f \n", debug_info.target_manager.prediction_time);
  //  printf("PLANNING TIME: %f \n", debug_info.planning.planning_time);
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
    //            options.callback_group =
    //            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //            state_subscriber_ = create_subscription<DroneStateMsg>(
    //                "/drone_state", rclcpp::QoS(1),
    //                std::bind(&LosServer::DroneStateCallback, this, std::placeholders::_1),
    //                options);
    //            options.callback_group =
    //            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //            points_subscriber_ = create_subscription<PointCloudMsg>(
    //                "/point_cloud", rclcpp::QoS(1),
    //                std::bind(&LosServer::PointsCallback, this, std::placeholders::_1), options);
    //            options.callback_group =
    //            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //            structured_obstacle_state_array_subscriber_ =
    //            create_subscription<ObjectStateArrayMsg>(
    //                "/obstacle_state_list", rclcpp::QoS(1),
    //                std::bind(&LosServer::ObjectStateArrayCallback, this, std::placeholders::_1),
    //                options);
    //            options.callback_group =
    //            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //            target_state_array_subscriber_ = create_subscription<ObjectStateArrayMsg>(
    //                "/target_state_list", rclcpp::QoS(1),
    //                std::bind(&LosServer::TargetStateArrayCallback, this, std::placeholders::_1),
    //                options);
  }
  visualization_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  visualization_timer_ = this->create_wall_timer(
      40ms, std::bind(&LosServer::VisualizationTimerCallback, this), visualization_callback_group_);
  visualization_.obstacle_path_vis_publisher = create_publisher<ObstaclePathVisualizationMsg>(
      "~/visualization/obstacle_array_info", rclcpp::QoS(1));
  visualization_.target_best_path_vis_publisher = create_publisher<TargetBestPathVisualizationMsg>(
      "~/visualization/target_best_array_info", rclcpp::QoS(1));
  visualization_.target_safe_path_vis_publisher = create_publisher<TargetSafePathVisualizationMsg>(
      "~/visualization/target_safe_array_info", rclcpp::QoS(1));
  visualization_.target_raw_path_vis_publisher = create_publisher<TargetRawPathVisualizationMsg>(
      "~/visualization/target_raw_array_info", rclcpp::QoS(1));
  visualization_.keeper_raw_path_vis_publisher = create_publisher<KeeperRawPathVisualizationMsg>(
      "~/visualization/keeper_raw_array_info", rclcpp::QoS(1));
  visualization_.keeper_safe_path_vis_publisher = create_publisher<KeeperSafePathVisualizationMsg>(
      "~/visualization/keeper_safe_array_info", rclcpp::QoS(1));
  visualization_.fail_flag_vis_publisher =
      create_publisher<FailVisualizationMsg>("~/visualization/fail_flag", rclcpp::QoS(1));

  input_publisher_ = create_publisher<InputMsg>("jerk_control_input", rclcpp::QoS(1));
  velocity_input_publisher_ =
      create_publisher<VelocityInputMsg>("velocity_control_input", rclcpp::QoS(1));
  accel_input_publisher_ = create_publisher<AccelInputMsg>("accel_control_input", rclcpp::QoS(1));

  control_timer_ = this->create_wall_timer(20ms, std::bind(&LosServer::ControlTimerCallback, this));

  toggle_activate_server_ = this->create_service<ToggleActivateService>(
      "~/toggle_activate", std::bind(&LosServer::ToggleActivateCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));

  // Parameter Settings
  ObstacleParameter obstacle_param;
  PredictionParameter prediction_param;
  PlanningParameter planning_param;
  ProblemParameter problem_param;
  // Parameter Settings for Problem
  get_parameter<bool>("problem.is_2d", problem_param.is_2d);
  get_parameter<double>("problem.replanning_period", problem_param.replanning_period);
  get_parameter<int>("problem.control_mode", problem_param.control_mode);

  // Parameter Settings for ObstacleManager
  get_parameter<float>("obstacle_manager.planning_horizon", obstacle_param.planning_horizon);

  // Parameter Settings for TargetManager
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

  // Parameter Settings for TrajectoryPlanner
  get_parameter<int>("trajectory_planner.sampling.num_sample", planning_param.sampling.num_sample);
  get_parameter<int>("trajectory_planner.sampling.num_thread", planning_param.sampling.num_thread);
  get_parameter<bool>("trajectory_planner.sampling.is_lite", planning_param.sampling.is_lite);
  get_parameter<float>("trajectory_planner.horizon.planning", planning_param.horizon.planning);
  get_parameter<float>("trajectory_planner.distance.obstacle_max",
                       planning_param.distance.obstacle_max);
  get_parameter<float>("trajectory_planner.distance.end_points_min",
                       planning_param.distance.end_points_min);
  get_parameter<float>("trajectory_planner.distance.end_points_max",
                       planning_param.distance.end_points_max);
  get_parameter<float>("trajectory_planner.distance.target_min",
                       planning_param.distance.target_min);
  get_parameter<float>("trajectory_planner.distance.target_max",
                       planning_param.distance.target_max);
  get_parameter<float>("trajectory_planner.dynamic_limits.vel_max",
                       planning_param.dynamic_limits.vel_max);
  get_parameter<float>("trajectory_planner.dynamic_limits.acc_max",
                       planning_param.dynamic_limits.acc_max);
  get_parameter<float>("trajectory_planner.dynamic_limits.yaw_rate_max",
                       planning_param.dynamic_limits.yaw_rate_max);
  get_parameter<float>("trajectory_planner.safe_distance.rx", planning_param.safe_distance.rx);
  get_parameter<float>("trajectory_planner.safe_distance.ry", planning_param.safe_distance.ry);
  get_parameter<float>("trajectory_planner.safe_distance.rz", planning_param.safe_distance.rz);
  get_parameter<float>("trajectory_planner.virtual_pcl_bbox.width",
                       planning_param.virtual_pcl_bbox.width);
  get_parameter<float>("trajectory_planner.virtual_pcl_bbox.height",
                       planning_param.virtual_pcl_bbox.height);

  Parameters parameters;
  parameters.obstacle = obstacle_param;
  parameters.prediction = prediction_param;
  parameters.planning = planning_param;
  parameters.problem = problem_param;
  wrapper_ptr_ = new Wrapper(parameters);

  VisualizationParameters parameters_vis;
  get_parameter<string>("frame_id", parameters_vis.frame_id);
  // obstacle path
  get_parameter<bool>("obstacle.publish", parameters_vis.obstacle.publish);
  get_parameter<int>("obstacle.num_time_sample", parameters_vis.obstacle.num_time_sample);
  get_parameter<float>("obstacle.line_scale", parameters_vis.obstacle.line_scale);
  get_parameter<float>("obstacle.color.a", parameters_vis.obstacle.color.a);
  get_parameter<float>("obstacle.color.r", parameters_vis.obstacle.color.r);
  get_parameter<float>("obstacle.color.g", parameters_vis.obstacle.color.g);
  get_parameter<float>("obstacle.color.b", parameters_vis.obstacle.color.b);
  // target raw primitives
  get_parameter<bool>("target.raw.publish", parameters_vis.target.raw.publish);
  get_parameter<float>("target.raw.proportion", parameters_vis.target.raw.proportion);
  get_parameter<int>("target.raw.num_time_sample", parameters_vis.target.raw.num_time_sample);
  get_parameter<float>("target.raw.color.a", parameters_vis.target.raw.color.a);
  get_parameter<float>("target.raw.color.r", parameters_vis.target.raw.color.r);
  get_parameter<float>("target.raw.color.g", parameters_vis.target.raw.color.g);
  get_parameter<float>("target.raw.color.b", parameters_vis.target.raw.color.b);
  get_parameter<float>("target.raw.line_scale", parameters_vis.target.raw.line_scale);
  // target safe primitives
  get_parameter<bool>("target.safe.publish", parameters_vis.target.safe.publish);
  get_parameter<float>("target.safe.proportion", parameters_vis.target.safe.proportion);
  get_parameter<int>("target.safe.num_time_sample", parameters_vis.target.safe.num_time_sample);
  get_parameter<float>("target.safe.color.a", parameters_vis.target.safe.color.a);
  get_parameter<float>("target.safe.color.r", parameters_vis.target.safe.color.r);
  get_parameter<float>("target.safe.color.g", parameters_vis.target.safe.color.g);
  get_parameter<float>("target.safe.color.b", parameters_vis.target.safe.color.b);
  get_parameter<float>("target.safe.line_scale", parameters_vis.target.safe.line_scale);
  // target prediction results
  get_parameter<bool>("target.best.publish", parameters_vis.target.best.publish);
  get_parameter<int>("target.best.num_time_sample", parameters_vis.target.best.num_time_sample);
  get_parameter<float>("target.best.color.a", parameters_vis.target.best.color.a);
  get_parameter<float>("target.best.color.r", parameters_vis.target.best.color.r);
  get_parameter<float>("target.best.color.g", parameters_vis.target.best.color.g);
  get_parameter<float>("target.best.color.b", parameters_vis.target.best.color.b);
  get_parameter<float>("target.best.line_scale", parameters_vis.target.best.line_scale);
  // keeper raw primitives
  get_parameter<bool>("keeper.raw.publish", parameters_vis.keeper.raw.publish);
  get_parameter<float>("keeper.raw.proportion", parameters_vis.keeper.raw.proportion);
  get_parameter<int>("keeper.raw.num_time_sample", parameters_vis.keeper.raw.num_time_sample);
  get_parameter<float>("keeper.raw.color.a", parameters_vis.keeper.raw.color.a);
  get_parameter<float>("keeper.raw.color.r", parameters_vis.keeper.raw.color.r);
  get_parameter<float>("keeper.raw.color.g", parameters_vis.keeper.raw.color.g);
  get_parameter<float>("keeper.raw.color.b", parameters_vis.keeper.raw.color.b);
  get_parameter<float>("keeper.raw.line_scale", parameters_vis.keeper.raw.line_scale);
  // keeper safe primitives
  get_parameter<bool>("keeper.safe.publish", parameters_vis.keeper.safe.publish);
  get_parameter<float>("keeper.safe.proportion", parameters_vis.keeper.safe.proportion);
  get_parameter<int>("keeper.safe.num_time_sample", parameters_vis.keeper.safe.num_time_sample);
  get_parameter<float>("keeper.safe.color.a", parameters_vis.keeper.safe.color.a);
  get_parameter<float>("keeper.safe.color.r", parameters_vis.keeper.safe.color.r);
  get_parameter<float>("keeper.safe.color.g", parameters_vis.keeper.safe.color.g);
  get_parameter<float>("keeper.safe.color.b", parameters_vis.keeper.safe.color.b);
  get_parameter<float>("keeper.safe.line_scale", parameters_vis.keeper.safe.line_scale);
  // prediction fail flag
  get_parameter<float>("fail_flag.prediction.color.a", parameters_vis.fail_flag.prediction.color.a);
  get_parameter<float>("fail_flag.prediction.color.r", parameters_vis.fail_flag.prediction.color.r);
  get_parameter<float>("fail_flag.prediction.color.g", parameters_vis.fail_flag.prediction.color.g);
  get_parameter<float>("fail_flag.prediction.color.b", parameters_vis.fail_flag.prediction.color.b);
  // planning fail flag
  get_parameter<float>("fail_flag.planning.color.a", parameters_vis.fail_flag.planning.color.a);
  get_parameter<float>("fail_flag.planning.color.r", parameters_vis.fail_flag.planning.color.r);
  get_parameter<float>("fail_flag.planning.color.g", parameters_vis.fail_flag.planning.color.g);
  get_parameter<float>("fail_flag.planning.color.b", parameters_vis.fail_flag.planning.color.b);

  visualizer_.UpdateParam(parameters_vis);
  planning_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(problem_param.replanning_period),
                              std::bind(&LosServer::PlanningTimerCallback, this));
}