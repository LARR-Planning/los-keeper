#include "los_keeper/wrapper/wrapper.h"
using namespace los_keeper;

std::optional<JerkControlInput> PlanningResult::GetJerkInputAtTime(double t) const {
  JerkControlInput input;
  input.seq = seq;
  input.t_sec = t;
  if (!chasing_trajectory.has_value()) {
    input.jx = 0.0f;
    input.jy = 0.0f;
    input.jz = 0.0f;
    //    return input;
    return nullopt;
  }
  BernsteinCoefficients jx_coeff = {
      float(60.0 /
            pow(chasing_trajectory.value().px.GetTimeInterval()[1] -
                    chasing_trajectory.value().px.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[3] -
                   3.0f * chasing_trajectory.value().px.GetBernsteinCoefficient()[2] +
                   3.0f * chasing_trajectory.value().px.GetBernsteinCoefficient()[1] -
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[0])),
      float(60.0 /
            pow(chasing_trajectory.value().px.GetTimeInterval()[1] -
                    chasing_trajectory.value().px.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[4] -
                   3.0f * chasing_trajectory.value().px.GetBernsteinCoefficient()[3] +
                   3.0f * chasing_trajectory.value().px.GetBernsteinCoefficient()[2] -
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[1])),
      float(60.0 /
            pow(chasing_trajectory.value().px.GetTimeInterval()[1] -
                    chasing_trajectory.value().px.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[5] -
                   3.0f * chasing_trajectory.value().px.GetBernsteinCoefficient()[4] +
                   3.0f * chasing_trajectory.value().px.GetBernsteinCoefficient()[3] -
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[2]))};
  BernsteinCoefficients jy_coeff = {
      float(60.0 /
            pow(chasing_trajectory.value().py.GetTimeInterval()[1] -
                    chasing_trajectory.value().py.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[3] -
                   3.0f * chasing_trajectory.value().py.GetBernsteinCoefficient()[2] +
                   3.0f * chasing_trajectory.value().py.GetBernsteinCoefficient()[1] -
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[0])),
      float(60.0 /
            pow(chasing_trajectory.value().py.GetTimeInterval()[1] -
                    chasing_trajectory.value().py.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[4] -
                   3.0f * chasing_trajectory.value().py.GetBernsteinCoefficient()[3] +
                   3.0f * chasing_trajectory.value().py.GetBernsteinCoefficient()[2] -
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[1])),
      float(60.0 /
            pow(chasing_trajectory.value().py.GetTimeInterval()[1] -
                    chasing_trajectory.value().py.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[5] -
                   3.0f * chasing_trajectory.value().py.GetBernsteinCoefficient()[4] +
                   3.0f * chasing_trajectory.value().py.GetBernsteinCoefficient()[3] -
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[2]))};
  BernsteinCoefficients jz_coeff = {
      float(60.0 /
            pow(chasing_trajectory.value().pz.GetTimeInterval()[1] -
                    chasing_trajectory.value().pz.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[3] -
                   3.0f * chasing_trajectory.value().pz.GetBernsteinCoefficient()[2] +
                   3.0f * chasing_trajectory.value().pz.GetBernsteinCoefficient()[1] -
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[0])),
      float(60.0 /
            pow(chasing_trajectory.value().pz.GetTimeInterval()[1] -
                    chasing_trajectory.value().pz.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[4] -
                   3.0f * chasing_trajectory.value().pz.GetBernsteinCoefficient()[3] +
                   3.0f * chasing_trajectory.value().pz.GetBernsteinCoefficient()[2] -
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[1])),
      float(60.0 /
            pow(chasing_trajectory.value().pz.GetTimeInterval()[1] -
                    chasing_trajectory.value().pz.GetTimeInterval()[0],
                3) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[5] -
                   3.0f * chasing_trajectory.value().pz.GetBernsteinCoefficient()[4] +
                   3.0f * chasing_trajectory.value().pz.GetBernsteinCoefficient()[3] -
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[2]))};
  BernsteinPoly jx_poly{chasing_trajectory.value().px.GetTimeInterval(), jx_coeff, 2},
      jy_poly{chasing_trajectory.value().py.GetTimeInterval(), jy_coeff, 2},
      jz_poly{chasing_trajectory.value().pz.GetTimeInterval(), jz_coeff, 2};
  //  printf("JX POLY T0: %f \n",jx_poly.GetTimeInterval()[0]);
  //  printf("QUERY TIME: %f \n",t);
  input.jx = jx_poly.GetValue(t);
  input.jy = jy_poly.GetValue(t);
  input.jz = jz_poly.GetValue(t);
  return input;
}
std::optional<VelocityControlInput> PlanningResult::GetVelocityInputAtTime(double t) const {
  VelocityControlInput input;
  input.seq = seq;
  input.t_sec = t;
  if (!chasing_trajectory.has_value()) {
    input.vx = 0.0f;
    input.vy = 0.0f;
    input.vz = 0.0f;
    return nullopt;
  }
  BernsteinCoefficients vx_coeff = {
      float(5.0 /
            (chasing_trajectory.value().px.GetTimeInterval()[1] -
             chasing_trajectory.value().px.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[1] -
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[0])),
      float(5.0 /
            (chasing_trajectory.value().px.GetTimeInterval()[1] -
             chasing_trajectory.value().px.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[2] -
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[1])),
      float(5.0 /
            (chasing_trajectory.value().px.GetTimeInterval()[1] -
             chasing_trajectory.value().px.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[3] -
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[2])),
      float(5.0 /
            (chasing_trajectory.value().px.GetTimeInterval()[1] -
             chasing_trajectory.value().px.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[4] -
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[3])),
      float(5.0 /
            (chasing_trajectory.value().px.GetTimeInterval()[1] -
             chasing_trajectory.value().px.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[5] -
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[4]))};
  BernsteinCoefficients vy_coeff = {
      float(5.0 /
            (chasing_trajectory.value().py.GetTimeInterval()[1] -
             chasing_trajectory.value().py.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[1] -
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[0])),
      float(5.0 /
            (chasing_trajectory.value().py.GetTimeInterval()[1] -
             chasing_trajectory.value().py.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[2] -
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[1])),
      float(5.0 /
            (chasing_trajectory.value().py.GetTimeInterval()[1] -
             chasing_trajectory.value().py.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[3] -
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[2])),
      float(5.0 /
            (chasing_trajectory.value().py.GetTimeInterval()[1] -
             chasing_trajectory.value().py.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[4] -
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[3])),
      float(5.0 /
            (chasing_trajectory.value().py.GetTimeInterval()[1] -
             chasing_trajectory.value().py.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[5] -
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[4]))};
  BernsteinCoefficients vz_coeff = {
      float(5.0 /
            (chasing_trajectory.value().pz.GetTimeInterval()[1] -
             chasing_trajectory.value().pz.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[1] -
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[0])),
      float(5.0 /
            (chasing_trajectory.value().pz.GetTimeInterval()[1] -
             chasing_trajectory.value().pz.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[2] -
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[1])),
      float(5.0 /
            (chasing_trajectory.value().pz.GetTimeInterval()[1] -
             chasing_trajectory.value().pz.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[3] -
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[2])),
      float(5.0 /
            (chasing_trajectory.value().pz.GetTimeInterval()[1] -
             chasing_trajectory.value().pz.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[4] -
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[3])),
      float(5.0 /
            (chasing_trajectory.value().pz.GetTimeInterval()[1] -
             chasing_trajectory.value().pz.GetTimeInterval()[0]) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[5] -
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[4]))};
  BernsteinPoly vx_poly{chasing_trajectory.value().px.GetTimeInterval(), vx_coeff, 4},
      vy_poly{chasing_trajectory.value().py.GetTimeInterval(), vy_coeff, 4},
      vz_poly{chasing_trajectory.value().pz.GetTimeInterval(), vz_coeff, 4};
  input.vx = vx_poly.GetValue(t);
  input.vy = vy_poly.GetValue(t);
  input.vz = vz_poly.GetValue(t);
  return input;
}

std::optional<AccelControlInput> PlanningResult::GetAccelInputAtTime(double t) const {
  AccelControlInput input;
  input.seq = seq;
  input.t_sec = t;
  if (!chasing_trajectory.has_value()) {
    input.ax = 0.0f;
    input.ay = 0.0f;
    input.az = 0.0f;
    return nullopt;
  }
  BernsteinCoefficients ax_coeff = {
      float(20.0 /
            pow(chasing_trajectory.value().px.GetTimeInterval()[1] -
                    chasing_trajectory.value().px.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[2] -
                   2 * chasing_trajectory.value().px.GetBernsteinCoefficient()[1] +
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[0])),
      float(20.0 /
            pow(chasing_trajectory.value().px.GetTimeInterval()[1] -
                    chasing_trajectory.value().px.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[3] -
                   2 * chasing_trajectory.value().px.GetBernsteinCoefficient()[2] +
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[1])),
      float(20.0 /
            pow(chasing_trajectory.value().px.GetTimeInterval()[1] -
                    chasing_trajectory.value().px.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[4] -
                   2 * chasing_trajectory.value().px.GetBernsteinCoefficient()[3] +
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[2])),
      float(20.0 /
            pow(chasing_trajectory.value().px.GetTimeInterval()[1] -
                    chasing_trajectory.value().px.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().px.GetBernsteinCoefficient()[5] -
                   2 * chasing_trajectory.value().px.GetBernsteinCoefficient()[4] +
                   chasing_trajectory.value().px.GetBernsteinCoefficient()[3]))};
  BernsteinCoefficients ay_coeff = {
      float(20.0 /
            pow(chasing_trajectory.value().py.GetTimeInterval()[1] -
                    chasing_trajectory.value().py.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[2] -
                   2 * chasing_trajectory.value().py.GetBernsteinCoefficient()[1] +
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[0])),
      float(20.0 /
            pow(chasing_trajectory.value().py.GetTimeInterval()[1] -
                    chasing_trajectory.value().py.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[3] -
                   2 * chasing_trajectory.value().py.GetBernsteinCoefficient()[2] +
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[1])),
      float(20.0 /
            pow(chasing_trajectory.value().py.GetTimeInterval()[1] -
                    chasing_trajectory.value().py.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[4] -
                   2 * chasing_trajectory.value().py.GetBernsteinCoefficient()[3] +
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[2])),
      float(20.0 /
            pow(chasing_trajectory.value().py.GetTimeInterval()[1] -
                    chasing_trajectory.value().py.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().py.GetBernsteinCoefficient()[5] -
                   2 * chasing_trajectory.value().py.GetBernsteinCoefficient()[4] +
                   chasing_trajectory.value().py.GetBernsteinCoefficient()[3]))};
  BernsteinCoefficients az_coeff = {
      float(20.0 /
            pow(chasing_trajectory.value().pz.GetTimeInterval()[1] -
                    chasing_trajectory.value().pz.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[2] -
                   2 * chasing_trajectory.value().pz.GetBernsteinCoefficient()[1] +
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[0])),
      float(20.0 /
            pow(chasing_trajectory.value().pz.GetTimeInterval()[1] -
                    chasing_trajectory.value().pz.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[3] -
                   2 * chasing_trajectory.value().pz.GetBernsteinCoefficient()[2] +
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[1])),
      float(20.0 /
            pow(chasing_trajectory.value().pz.GetTimeInterval()[1] -
                    chasing_trajectory.value().pz.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[4] -
                   2 * chasing_trajectory.value().pz.GetBernsteinCoefficient()[3] +
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[2])),
      float(20.0 /
            pow(chasing_trajectory.value().pz.GetTimeInterval()[1] -
                    chasing_trajectory.value().pz.GetTimeInterval()[0],
                2) *
            double(chasing_trajectory.value().pz.GetBernsteinCoefficient()[5] -
                   2 * chasing_trajectory.value().pz.GetBernsteinCoefficient()[4] +
                   chasing_trajectory.value().pz.GetBernsteinCoefficient()[3]))};
  BernsteinPoly ax_poly{chasing_trajectory.value().px.GetTimeInterval(), ax_coeff, 3},
      ay_poly{chasing_trajectory.value().py.GetTimeInterval(), ay_coeff, 3},
      az_poly{chasing_trajectory.value().pz.GetTimeInterval(), az_coeff, 3};
  input.ax = ax_poly.GetValue(t);
  input.ay = ay_poly.GetValue(t);
  input.az = az_poly.GetValue(t);
  return input;
}

Wrapper::Wrapper() {
  // TODO(@): remove if unnecessary
  //  obstacle_manager_ = std::make_shared<ObstacleManager>();
  //  target_manager_ = std::make_shared<TargetManager>();
  //  trajectory_planner_ = std::make_shared<TrajectoryPlanner>();
}

Wrapper::Wrapper(const Parameters &parameters) {
  obstacle_manager_.reset(new ObstacleManager(parameters.obstacle));
  if (parameters.problem.is_2d) {
    target_manager_.reset(new TargetManager2D(parameters.prediction));
    trajectory_planner_.reset(new TrajectoryPlanner2D(parameters.planning));
  } else {
    target_manager_.reset(new TargetManager3D(parameters.prediction));
    trajectory_planner_.reset(new TrajectoryPlanner3D(parameters.planning));
  }
}

std::optional<Point> PlanningResult::GetPointAtTime(double t) const {
  if (!chasing_trajectory)
    return std::nullopt;
  return Point();
}

void Wrapper::UpdateState(store::State &state) {
  state.is_data_received = drone_state_.t_sec > 0.0 && target_state_list_.size() > 0;
  double t_current =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

  state.is_planning_expired = (t_current - planning_result_.last_plan_success_t_sec) >
                              parameters_.problem.replanning_period;

  // TODO(Lee): implement these
  // state.is_currently_safe =
  //     obstacle_manager_->CheckCollisionOnPosition(drone_state);
  // state.is_planning_safe =
  //     obstacle_manager_->CheckCollisionAlongTrajectory(drone_state);
}

void Wrapper::HandleStopAction() { state_.is_activated = false; }
void Wrapper::HandleActivateAction() { state_.is_activated = true; }
void Wrapper::HandleReplanAction() {
  //      printf("Handle ReplanAction\n");
  PlanningProblem planning_problem;
  { // Update Drone State
    std::scoped_lock lock(mutex_list_.drone_state);
    planning_problem.drone_state = drone_state_;
    //    printf("drone state time: %f \n",drone_state_.t_sec);
  }

  { // Update Obstacle State
    std::scoped_lock lock(mutex_list_.point_cloud, mutex_list_.object_state_list);
    planning_problem.point_cloud = obstacle_manager_->GetPointCloud();
    planning_problem.structured_obstacle_poly_list =
        obstacle_manager_->GetStructuredObstaclePolyList();
  }
  { // Update Target State
    std::scoped_lock lock(mutex_list_.target_state_list);
    planning_problem.target_state_list = target_state_list_;
  }
  UpdateObstacleDeubgInfo();
  PlanningResult new_planning_result;
  new_planning_result.seq = planning_result_.seq + 1;
  auto target_prediction_list = target_manager_->PredictTargetList(
      planning_problem.target_state_list, planning_problem.point_cloud,
      planning_problem.structured_obstacle_poly_list);
  UpdateTargetDebugInfo();
  if (target_prediction_list) {
    new_planning_result.chasing_trajectory = trajectory_planner_->ComputeChasingTrajectory(
        drone_state_, target_prediction_list.value(), planning_problem.point_cloud,
        planning_problem.structured_obstacle_poly_list);
    UpdatePlanningDebugInfo();
  } else {
    //    printf("NO TARGET PREDICTION \n");
  }

  if (new_planning_result.chasing_trajectory) {
    //        printf("PLANNING SUCCESS\n");
    new_planning_result.last_plan_success_t_sec =
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::unique_lock<std::mutex> lock(mutex_list_.control);
    planning_result_ = new_planning_result;
  } else {
    //    printf("PLANNING FAILURE\n");
    std::unique_lock<std::mutex> lock(mutex_list_.control);
    planning_result_.chasing_trajectory = std::nullopt;
  }
}

void Wrapper::HandleIdleAction() { /*printf("Handle IdleAction\n");*/
}

void Wrapper::OnPlanningTimerCallback() {

  UpdateState(state_);
  auto action = DecideAction(state_);
  switch (action) {
  case store::Action::kStop:
    HandleStopAction();
    break;
  case store::Action::kReplan:
    HandleReplanAction();
    break;
  default:
    HandleIdleAction();
    break;
  }
}

void Wrapper::OnToggleActivateServiceCallback() {
  printf("[Wrapper] received activation\n");
  using namespace store;
  if (!state_.is_activated)
    HandleActivateAction();
  else
    HandleStopAction();
}

void Wrapper::SetPoints(const pcl::PointCloud<pcl::PointXYZ> &points) {
  std::unique_lock<std::mutex> lock(mutex_list_.point_cloud, std::defer_lock);
  if (lock.try_lock()) {
    obstacle_manager_->SetObstacleCloud(points);
  }
}

void Wrapper::SetDroneState(const DroneState &drone_state) {
  std::unique_lock<std::mutex> lock(mutex_list_.drone_state, std::defer_lock);
  if (lock.try_lock()) {
    drone_state_ = drone_state;
  }
}

void Wrapper::SetObjectStateArray(const std::vector<ObjectState> &object_state_list) {
  std::unique_lock<std::mutex> lock(mutex_list_.object_state_list, std::defer_lock);
  if (lock.try_lock()) {
    object_state_list_.clear();
    object_state_list_ = object_state_list;
    obstacle_manager_->SetStructuredObstacleState(object_state_list_);
  }
}
void Wrapper::SetTargetStateArray(const vector<ObjectState> &target_state_list) {
  std::unique_lock<std::mutex> lock(mutex_list_.target_state_list, std::defer_lock);
  if (lock.try_lock()) {
    target_state_list_ = target_state_list;
  }
}

std::optional<JerkControlInput> Wrapper::GenerateControlInputFromPlanning(double time) {
  std::optional<JerkControlInput> control_input;
  {
    std::unique_lock<std::mutex> lock(mutex_list_.control);
    control_input = planning_result_.GetJerkInputAtTime(time);
  }
  return control_input;
}
std::optional<AccelControlInput> Wrapper::GenerateAccelControlInputFromPlanning(double time) {
  std::optional<AccelControlInput> control_input;
  {
    std::unique_lock<std::mutex> lock(mutex_list_.control);
    control_input = planning_result_.GetAccelInputAtTime(time);
  }
  return control_input;
}
std::optional<VelocityControlInput> Wrapper::GenerateVelocityControlInputFromPlanning(double time) {
  std::optional<VelocityControlInput> control_input;
  {
    std::unique_lock<std::mutex> lock(mutex_list_.control);
    control_input = planning_result_.GetVelocityInputAtTime(time);
  }
  return control_input;
}

DebugInfo Wrapper::GetDebugInfo() {
  DebugInfo debug_info{};
  { // obstacle
    std::unique_lock<std::mutex> lock(mutex_list_.debug_obstacle_info, std::defer_lock);
    if (lock.try_lock()) {
      debug_info.obstacle_manager = debug_info_.obstacle_manager;
    }
  }
  { // target
    std::unique_lock<std::mutex> lock(mutex_list_.debug_target_info, std::defer_lock);
    if (lock.try_lock()) {
      debug_info.target_manager = debug_info_.target_manager;
    }
  }
  { // keeper
    std::unique_lock<std::mutex> lock(mutex_list_.debug_planning_info, std::defer_lock);
    if (lock.try_lock()) {
      debug_info.planning = debug_info_.planning;
    }
  }
  return debug_info;
}

void Wrapper::UpdateTargetDebugInfo() {
  std::scoped_lock lock(mutex_list_.debug_target_info);
  debug_info_.target_manager = target_manager_->GetDebugInfo();
}
void Wrapper::UpdateObstacleDeubgInfo() {
  std::scoped_lock lock(mutex_list_.debug_obstacle_info);
  debug_info_.obstacle_manager = obstacle_manager_->GetDebugInfo();
}
void Wrapper::UpdatePlanningDebugInfo() {
  std::scoped_lock lock(mutex_list_.debug_planning_info);
  debug_info_.planning = trajectory_planner_->GetDebugInfo();
}
