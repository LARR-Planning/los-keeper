#include "los_keeper/wrapper/wrapper.h"
using namespace los_keeper;

bool Wrapper::Plan() const {
  return target_manager_->GetName() == "TargetManager";
}

void Wrapper::SetProblem(
    const pcl::PointCloud<pcl::PointXYZ> &cloud,
    const std::vector<ObjectState> &structured_obstacle_state_list,
    const std::vector<ObjectState> &target_state_list) {
  obstacle_manager_->SetObstacleInformation(cloud,
                                            structured_obstacle_state_list);
  target_manager_->SetObstacleState(
      obstacle_manager_->GetPcl(),
      obstacle_manager_->GetStructuredObstaclePolyList());
  target_manager_->SetTargetState(target_state_list);
}

Wrapper::Wrapper() { target_manager_.reset(new TargetManager2D); }

void Wrapper::ApplyStartAction(State &state) {}
void Wrapper::ApplyPauseAction(State &state) {}
void Wrapper::ApplyResetAction(State &state) {}
void Wrapper::ApplyUpdateAction(State &state) {}

void Wrapper::ApplyAction(const los_keeper::Action &action) {
  switch (action) {
  case Action::kStart:
    ApplyStartAction(state_);
    break;
  case Action::kPause:
    ApplyPauseAction(state_);
    break;
  case Action::kReset:
    ApplyResetAction(state_);
    break;
  case Action::kUpdate:
    ApplyUpdateAction(state_);
    break;
  }
}
