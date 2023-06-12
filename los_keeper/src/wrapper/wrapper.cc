#include "los_keeper/wrapper/wrapper.h"
using namespace los_keeper;

std::optional<JerkControlInput> PlanningResult::GetJerkInputAtTime(double t) const {
  if (!chasing_trajectory.has_value())
    return std::nullopt;
  JerkControlInput input;
  input.seq = seq;
  input.t_sec = t;
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
                              parameters_.planning.replan_period_sec;

  // TODO(Lee): implement these
  // state.is_currently_safe =
  //     obstacle_manager_->CheckCollisionOnPosition(drone_state);
  // state.is_planning_safe =
  //     obstacle_manager_->CheckCollisionAlongTrajectory(drone_state);
}

void Wrapper::HandleStopAction() { state_.is_activated = false; }
void Wrapper::HandleActivateAction() { state_.is_activated = true; }
void Wrapper::HandleReplanAction() {
  //  printf("Handle ReplanAction\n");
  PlanningProblem planning_problem;
  {
    std::scoped_lock lock(mutex_list_.drone_state, mutex_list_.point_cloud);
    planning_problem.drone_state = drone_state_;
    planning_problem.point_cloud = obstacle_manager_->GetPointCloud();
    planning_problem.structured_obstacle_poly_list =
        obstacle_manager_->GetStructuredObstaclePolyList();
    planning_problem.target_state_list = target_state_list_;
  }
  const auto &point_cloud = planning_problem.point_cloud;
  const auto &structured_obstacle_poly_list = planning_problem.structured_obstacle_poly_list;

  PlanningResult new_planning_result;
  new_planning_result.seq = planning_result_.seq + 1;
  auto target_prediction_list = target_manager_->PredictTargetList(
      planning_problem.target_state_list, point_cloud, structured_obstacle_poly_list);
  if (!target_prediction_list)
    goto update;

  new_planning_result.last_plan_success_t_sec =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

  new_planning_result.chasing_trajectory = StatePoly();

update : {
  std::unique_lock<std::mutex> lock(mutex_list_.control);
  planning_result_ = new_planning_result;
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
    // TODO(@): set whoever need this
    drone_state_ = drone_state;
  }
}

void Wrapper::SetObjectStateArray(const std::vector<ObjectState> &object_state_list) {
  std::unique_lock<std::mutex> lock(mutex_list_.object_state_list, std::defer_lock);
  if (lock.try_lock()) {
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
    // TODO(Lee): compute jerk from planning result
    control_input = planning_result_.GetJerkInputAtTime(time);
  }
  return control_input;
}
