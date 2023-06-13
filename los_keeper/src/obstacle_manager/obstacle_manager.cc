#include "los_keeper/obstacle_manager/obstacle_manager.h"

using namespace los_keeper;

void ObstacleManager::SetObstacleCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  cloud_.points = cloud.points;
}

void ObstacleManager::SetStructuredObstacleState(
    const std::vector<ObjectState> &structured_obstacle_state_list) {
  structured_obstacle_state_list_.clear();
  structured_obstacle_state_list_ = structured_obstacle_state_list;
  TranslateStateToPoly();
}

void ObstacleManager::TranslateStateToPoly() {
  structured_obstacle_poly_list_.clear();
  StatePoly temp_state_poly;
  temp_state_poly.SetDegree(3);
  float time_interval[2]{0.0f, param_.planning_horizon};
  temp_state_poly.SetTimeInterval(time_interval);
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
    structured_obstacle_poly_list_.push_back(temp_state_poly);
  }
}

PclPointCloud ObstacleManager::GetPointCloud() { return cloud_; }
ObstacleManager::ObstacleManager(const ObstacleParameter &param) : param_(param) {}

bool ObstacleManager::CheckCollisionAlongTrajectory(const StatePoly &trajectory) {
  int num_sample = 20;
  float time_sample[num_sample];
  for (int i = 0; i < num_sample; i++) {
    time_sample[i] = param_.planning_horizon / float(num_sample - 1) * (float)i;
  }
  bool safe_pcl = true;
  bool safe_structured = true;
  if (not cloud_.points.empty()) {
    printf("HIHIHI\n");
    for (int i = 0; i < num_sample; i++) {
      for (int j = 0; j < cloud_.points.size(); j++) {
        if (powf(trajectory.GetPointAtTime(time_sample[i]).x - cloud_.points[j].x, 2) /
                    powf(trajectory.rx, 2) +
                powf(trajectory.GetPointAtTime(time_sample[i]).y - cloud_.points[j].y, 2) /
                    powf(trajectory.ry, 2) +
                powf(trajectory.GetPointAtTime(time_sample[i]).z - cloud_.points[j].z, 2) /
                    powf(trajectory.rz, 2) <
            1.0f) {
          safe_pcl = false;
          break;
        }
      }
      if (not safe_pcl)
        break;
    }
  }
  if (not structured_obstacle_state_list_.empty()) {
    for (int i = 0; i < num_sample; i++) {
      for (int j = 0; j < structured_obstacle_state_list_.size(); j++) {
        if (powf(trajectory.GetPointAtTime(time_sample[i]).x -
                     (structured_obstacle_state_list_[j].px +
                      structured_obstacle_state_list_[j].vx * time_sample[i]),
                 2) /
                    powf(trajectory.rx + structured_obstacle_state_list_[j].rx, 2) +
                powf(trajectory.GetPointAtTime(time_sample[i]).y -
                         (structured_obstacle_state_list_[j].py +
                          structured_obstacle_state_list_[j].vy * time_sample[i]),
                     2) /
                    powf(trajectory.ry + structured_obstacle_state_list_[j].ry, 2) +
                powf(trajectory.GetPointAtTime(time_sample[i]).z -
                         (structured_obstacle_state_list_[j].pz +
                          structured_obstacle_state_list_[j].vz * time_sample[i]),
                     2) /
                    powf(trajectory.rz + structured_obstacle_state_list_[j].rz, 2) <
            1.0f) {
          safe_structured = false;
          break;
        }
      }
      if (not safe_structured)
        break;
    }
  }
  if (safe_pcl and safe_structured)
    return false;
  else
    return true;
}
bool ObstacleManager::CheckCollisionWithPoint(const DroneState &drone_state) {
  bool safe_pcl = true;
  bool safe_structured = true;
  if (not cloud_.points.empty()) {
    for (int i = 0; i < cloud_.points.size(); i++) {
      if (powf(drone_state.px - cloud_.points[i].x, 2) / powf(drone_state.rx, 2) +
              powf(drone_state.py - cloud_.points[i].y, 2) / powf(drone_state.ry, 2) +
              powf(drone_state.pz - cloud_.points[i].z, 2) / powf(drone_state.rz, 2) <
          1.0f) {
        safe_pcl = false;
        break;
      }
    }
  }
  if (not structured_obstacle_state_list_.empty()) {
    for (int i = 0; i < structured_obstacle_state_list_.size(); i++) {
      if (powf(drone_state.px - structured_obstacle_state_list_[i].px, 2) /
                  powf(drone_state.rx, 2) +
              powf(drone_state.py - structured_obstacle_state_list_[i].py, 2) /
                  powf(drone_state.ry, 2) +
              powf(drone_state.pz - structured_obstacle_state_list_[i].pz, 2) /
                  powf(drone_state.rz, 2) <
          1.0f) {
        safe_structured = false;
        break;
      }
    }
  }
  if (safe_pcl and safe_structured)
    return false;
  else
    return true;
}
