#include "los_keeper/wrapper/wrapper.h"
using namespace los_keeper;

bool Wrapper::Plan() const {
  return target_manager_->GetName() == "TargetManager";
}

void Wrapper::SetLongString(const std::string &long_string) {
  long_string_ = long_string;
}
void Wrapper::SetShortString(const std::string &short_string) {
  short_string_ = short_string;
};

std::string Wrapper::GetConcatString() const {
  return long_string_ + " and " + short_string_;
}
void Wrapper::SetProblem(
    const pcl::PointCloud<pcl::PointXYZ> &cloud,
    const std::vector<ObjectState> &structured_obstacle_state_list,
    const std::vector<ObjectState> &target_state_list) {
  obstacle_manager_.SetObstacleInformation(cloud,
                                           structured_obstacle_state_list);
  target_manager_->SetObstacleState(
      obstacle_manager_.GetPcl(),
      obstacle_manager_.GetStructuredObstaclePolyList());
  target_manager_->SetTargetState(target_state_list);
}
Wrapper::Wrapper() { target_manager_ = new los_keeper::TargetManager2D; }
