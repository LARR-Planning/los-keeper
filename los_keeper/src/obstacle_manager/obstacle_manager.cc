#include "los_keeper/obstacle_manager/obstacle_manager.h"

using namespace los_keeper;

void ObstacleManager::SetObstacleCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  cloud_.points = cloud.points;
}

void ObstacleManager::SetStructuredObstacleState(
    const std::vector<ObjectState> &structured_obstacle_state_list) {
  structured_obstacle_state_list_ = structured_obstacle_state_list;
  TranslateStateToPoly();
}

void ObstacleManager::TranslateStateToPoly() {
  structured_obstacle_poly_list_.clear();
  StatePoly temp_state_poly;
  temp_state_poly.SetDegree(3);
  for (const auto &i : structured_obstacle_state_list_) {
    float temp_coefficient_x[4]{i.px, i.px + 0.33333333f * i.vx * param_.planning_horizon,
                                i.px + 0.66666667f * i.vx * param_.planning_horizon,
                                i.px + i.vx * param_.planning_horizon};
    float temp_coefficient_y[4]{i.py, i.py + 0.33333333f * i.vy * param_.planning_horizon,
                                i.py + 0.66666667f * i.vy * param_.planning_horizon,
                                i.py + i.vy * param_.planning_horizon};
    float temp_coefficient_z[4]{i.pz, i.pz + 0.33333333f * i.vz * param_.planning_horizon,
                                i.pz + 0.66666667f * i.vz * param_.planning_horizon,
                                i.pz + i.vz * param_.planning_horizon};
    temp_state_poly.px.SetBernsteinCoeff(temp_coefficient_x);
    temp_state_poly.py.SetBernsteinCoeff(temp_coefficient_y);
    temp_state_poly.pz.SetBernsteinCoeff(temp_coefficient_z);
    temp_state_poly.rx = i.rx;
    temp_state_poly.ry = i.ry;
    temp_state_poly.rz = i.rz;
  }
}

PclPointCloud ObstacleManager::GetPointCloud() { return cloud_; }
ObstacleManager::ObstacleManager(const ObstacleParam &param) : param_(param) {}
