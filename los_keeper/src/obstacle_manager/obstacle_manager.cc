#include "los_keeper/obstacle_manager/obstacle_manager.h"

std::string los_keeper::ObstacleManager::GetName() const { return name_;}

void los_keeper::ObstacleManager::SetObstacleInformation(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<ObjectState> &structured_obstacle_state_list) {
  cloud_.points = cloud.points;
  structured_obstacle_state_list_ = structured_obstacle_state_list;
  TranslateStateToPoly();
}
void los_keeper::ObstacleManager::TranslateStateToPoly() {
  structured_obstacle_poly_list_.clear();
  StatePoly temp_state_poly;
  temp_state_poly.SetDegree(3);
  for(const auto & i : structured_obstacle_state_list_){
    float temp_coefficient_x [4]{i.px,i.px+0.33333333f*i.vx*planning_horizon_,i.px+0.66666667f*i.vx*planning_horizon_,i.px+i.vx*planning_horizon_};
    float temp_coefficient_y [4]{i.py,i.py+0.33333333f*i.vy*planning_horizon_,i.py+0.66666667f*i.vy*planning_horizon_,i.py+i.vy*planning_horizon_};
    float temp_coefficient_z [4]{i.pz,i.pz+0.33333333f*i.vz*planning_horizon_,i.pz+0.66666667f*i.vz*planning_horizon_,i.pz+i.vz*planning_horizon_};
    temp_state_poly.px.SetBernsteinCoeff(temp_coefficient_x);
    temp_state_poly.py.SetBernsteinCoeff(temp_coefficient_y);
    temp_state_poly.pz.SetBernsteinCoeff(temp_coefficient_z);
    temp_state_poly.rx = i.rx;
    temp_state_poly.ry = i.ry;
    temp_state_poly.rz = i.rz;
  }
}
