#include "los_keeper/wrapper/wrapper.h"
using namespace los_keeper;

void Wrapper::UpdateState(store::State &state) {
  // state.is_initialized =
  //     obstacle_manager_->IsInitialized() && target_manager_->IsInitialized();
  // state.is_currently_safe =
  //     obstacle_manager_->CheckCollisionOnPosition(drone_state);
  // state.is_planning_safe =
  //     obstacle_manager_->CheckCollisionAlongTrajectory(drone_state);
}

void Wrapper::HandleStopAction() const {}
void Wrapper::HandleInitializeAction() const {}
void Wrapper::HandleReplanAction() const {}

Wrapper::Wrapper() { target_manager_.reset(new TargetManager2D); }

void Wrapper::OnPlanningTimerCallback() {

  UpdateState(state_);
  auto action = DecideAction(state_);

  switch (action) {
  case store::Action::kInitialize:
    HandleInitializeAction();
  case store::Action::kStop:
    HandleStopAction();
  default:
    HandleReplanAction();
  }
}

void Wrapper::OnStartServiceCallback() {
  using namespace store;
  // state_ = HandleStartAction(state_);
}

void Wrapper::SetPoints(const pcl::PointCloud<pcl::PointXYZ> &points) {
  std::unique_lock<std::mutex> lock(mutex_list_.pointcloud, std::defer_lock);
  if (lock.try_lock()) {
    // TODO(@): set whoever need this
    // obstacle_manager_->SetObstaclePoints(points)
  }
}

void Wrapper::SetDroneState(const DroneState &drone_state) {
  std::unique_lock<std::mutex> lock(mutex_list_.drone_state, std::defer_lock);
  if (lock.try_lock()) {
    // TODO(@): set whoever need this
  }
}

int Wrapper::GenerateControlInputFromPlanning(
    const PlanningOutput &planning_output, double time) const {
  return 0;
}
