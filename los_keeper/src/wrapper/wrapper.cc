#include "los_keeper/wrapper/wrapper.h"
using namespace los_keeper;

store::State los_keeper::Wrapper::HandleUpdateMonitorAction(
    store::State &previous_state) const {
  return store::State();
}
store::State los_keeper::Wrapper::HandleInitializeAction(
    store::State &previous_state) const {
  return store::State();
}

store::State
los_keeper::Wrapper::HandleReplanAction(store::State &previous_state) const {
  return store::State();
}

Wrapper::Wrapper() { target_manager_.reset(new TargetManager2D); }

void Wrapper::OnPlanningTimerCallback() {
  using namespace store;
  auto action = DecideAction(state_);
  store::State new_state;
  switch (action) {
  case Action::kUpdateMonitor:
    new_state = HandleUpdateMonitorAction(state_);
  case Action::kInitialize:
    new_state = HandleInitializeAction(state_);
  case Action::kReplan:
    new_state = HandleReplanAction(state_);
  }
  state_ = new_state;
}

void los_keeper::Wrapper::SetPoints(
    const pcl::PointCloud<pcl::PointXYZ> &points) {
  std::unique_lock<std::mutex> lock(mutex_list_.pointcloud, std::defer_lock);
  if (lock.try_lock()) {
    // TODO(@): set whoever need this
    // obstacle_manager_->SetObstaclePoints(points)
  }
}

void los_keeper::Wrapper::SetDroneState(const DroneState &drone_state) {
  std::unique_lock<std::mutex> lock(mutex_list_.drone_state, std::defer_lock);
  if (lock.try_lock()) {
    // TODO(@): set whoever need this
  }
}

int los_keeper::Wrapper::GetControlInput(double time) const { return 0; }
