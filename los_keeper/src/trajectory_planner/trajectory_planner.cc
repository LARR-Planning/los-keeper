#include "los_keeper/trajectory_planner/trajectory_planner.h"

using namespace los_keeper;

std::optional<StatePoly> TrajectoryPlanner::ComputeChasingTrajectory(
    const std::vector<StatePoly> &target_prediction_list,
    const los_keeper::PclPointCloud &obstacle_points,
    const std::vector<StatePoly> &structured_obstacle_poly_list) {
  return std::optional<StatePoly>();
}

void TrajectoryPlanner::SetTargetState(const PrimitiveList &target_trajectory_list) {
  target_trajectory_list_.clear();
  target_trajectory_list_ = target_trajectory_list; // TODO: change the order 3 to order 5
  num_target_ = (int)target_trajectory_list.size();
}

void TrajectoryPlanner::SetObstacleState(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                         const PrimitiveList &structured_obstacle_poly_list) {
  structured_obstacle_poly_list_.clear();
  structured_obstacle_poly_list_ = structured_obstacle_poly_list;
  cloud_.points.clear();
  cloud_.points = cloud.points;
}

void TrajectoryPlanner::SampleShootingPoints() {}

void TrajectoryPlanner::SampleShootingPointsSubProcess(const int &target_id, const int &chunk_size,
                                                       PointList &shooting_points_sub) {}

void TrajectoryPlanner::ComputePrimitives() {}

void TrajectoryPlanner::ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                    PrimitiveList &primitive_list_sub) {}
TrajectoryPlanner::TrajectoryPlanner(const PlanningParameter &param) { param_ = param; }
StatePoly TrajectoryPlanner::GetBestKeeperTrajectory() {}
void TrajectoryPlanner::CheckDistanceFromTargets() {}
void TrajectoryPlanner::CheckDistanceFromTargetsSubProcess(const int &start_idx, const int &end_idx,
                                                           IndexList &dist_idx_sub){

};

bool TrajectoryPlanner2D::PlanKeeperTrajectory() {
  SampleShootingPoints();
  ComputePrimitives();

  return false;
}

void TrajectoryPlanner2D::SampleShootingPoints() {
  shooting_points_.clear();
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread / num_target_;
    vector<thread> worker_thread;
    vector<vector<Point>> shooting_point_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TrajectoryPlanner2D::SampleShootingPointsSubProcess, this, i,
                                 num_chunk, std::ref(shooting_point_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < shooting_point_temp[j].size(); k++) {
        shooting_points_.push_back(shooting_point_temp[j][k]);
      }
    }
  }
}

void TrajectoryPlanner2D::SampleShootingPointsSubProcess(const int &target_id,
                                                         const int &chunk_size,
                                                         PointList &shooting_points_sub) {
  Point end_point_center{target_trajectory_list_[target_id].px.GetTerminalValue(),
                         target_trajectory_list_[target_id].py.GetTerminalValue(),
                         target_trajectory_list_[target_id].pz.GetTerminalValue()};
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> r_dis(param_.distance.target_min, param_.distance.target_max);
  std::uniform_real_distribution<> theta_dis(-M_PI, M_PI);
  double r, theta;
  Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
  for (int i = 0; i < chunk_size; i++) {
    r = r_dis(gen);
    theta = theta_dis(gen);
    tempPoint.x = float(end_point_center.x + r * cos(theta));
    tempPoint.y = float(end_point_center.y + r * sin(theta));
    tempPoint.z = float(end_point_center.z);
    shooting_points_sub.push_back(tempPoint);
  }
}

void TrajectoryPlanner2D::ComputePrimitives() {
  primitives_list_.clear();
  int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
  vector<thread> worker_thread;
  PrimitiveListSet primitive_list_temp;
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    worker_thread.emplace_back(&TrajectoryPlanner2D::ComputePrimitivesSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(primitive_list_temp[i]));
  }
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    worker_thread[i].join();
  }
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    for (int j = 0; j < primitive_list_temp[i].size(); j++) {
      primitives_list_.push_back(primitive_list_temp[i][j]);
    }
  }
}

void TrajectoryPlanner2D::ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                      PrimitiveList &primitive_list_sub) {
  StatePoly primitive_temp;
  primitive_temp.SetDegree(5);
  float time_interval_temp[2]{0.0, param_.horizon.planning};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[6];
  float param_horizon_planning_square = param_.horizon.planning * param_.horizon.planning;

  for (int i = start_idx; i < end_idx; i++) {
    { // x-component
      bernstein_coeff_temp[0] = drone_state_.px;
      bernstein_coeff_temp[1] = drone_state_.px + 0.2f * param_.horizon.planning * drone_state_.vx;
      bernstein_coeff_temp[2] = drone_state_.px + 0.4f * param_.horizon.planning * drone_state_.vx +
                                0.05f * param_horizon_planning_square * drone_state_.ax;
      bernstein_coeff_temp[3] = 0.16666667f * shooting_points_[i].x +
                                0.83333333f * drone_state_.px +
                                0.43333333f * param_.horizon.planning * drone_state_.vx +
                                0.06666667f * param_horizon_planning_square * drone_state_.ax;
      bernstein_coeff_temp[4] = 0.5f * shooting_points_[i].x + 0.5f * drone_state_.px +
                                0.3f * param_.horizon.planning * drone_state_.vx +
                                0.05f * param_horizon_planning_square * drone_state_.ax;
      bernstein_coeff_temp[5] = shooting_points_[i].x;
      primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // y-component
      bernstein_coeff_temp[0] = drone_state_.py;
      bernstein_coeff_temp[1] = drone_state_.py + 0.2f * param_.horizon.planning * drone_state_.vy;
      bernstein_coeff_temp[2] = drone_state_.py + 0.4f * param_.horizon.planning * drone_state_.vy +
                                0.05f * param_horizon_planning_square * drone_state_.ay;
      bernstein_coeff_temp[3] = 0.16666667f * shooting_points_[i].y +
                                0.83333333f * drone_state_.py +
                                0.43333333f * param_.horizon.planning * drone_state_.vy +
                                0.06666667f * param_horizon_planning_square * drone_state_.ay;
      bernstein_coeff_temp[4] = 0.5f * shooting_points_[i].y + 0.5f * drone_state_.py +
                                0.3f * param_.horizon.planning * drone_state_.vy +
                                0.05f * param_horizon_planning_square * drone_state_.ay;
      bernstein_coeff_temp[5] = shooting_points_[i].y;
      primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // z-component
      bernstein_coeff_temp[0] = drone_state_.pz;
      bernstein_coeff_temp[1] = drone_state_.pz + 0.2f * param_.horizon.planning * drone_state_.vz;
      bernstein_coeff_temp[2] = drone_state_.pz + 0.4f * param_.horizon.planning * drone_state_.vz;
      bernstein_coeff_temp[3] = drone_state_.pz + 0.6f * param_.horizon.planning * drone_state_.vz;
      bernstein_coeff_temp[4] = drone_state_.pz + 0.8f * param_.horizon.planning * drone_state_.vz;
      bernstein_coeff_temp[5] = drone_state_.pz + 1.0f * param_.horizon.planning * drone_state_.vz;
      primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    primitive_list_sub.push_back(primitive_temp);
  }
}

TrajectoryPlanner2D::TrajectoryPlanner2D(const PlanningParameter &param)
    : TrajectoryPlanner(param) {}

optional<StatePoly> TrajectoryPlanner2D::ComputeChasingTrajectory(
    const vector<StatePoly> &target_prediction_list, const PclPointCloud &obstacle_points,
    const vector<StatePoly> &structured_obstacle_poly_list) {
  this->SetTargetState(target_prediction_list);
  this->SetObstacleState(obstacle_points, structured_obstacle_poly_list);
  bool plan_success = this->PlanKeeperTrajectory();
  if (plan_success)
    return GetBestKeeperTrajectory();
  else
    return std::nullopt;
}
void TrajectoryPlanner2D::CheckDistanceFromTargets() {
  good_target_distance_index_list_.clear();
  int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
  vector<thread> worker_thread;
  IndexListSet good_target_distance_index_list_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    worker_thread.emplace_back(&TrajectoryPlanner2D::CheckDistanceFromTargetsSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(good_target_distance_index_list_temp[i]));
  }
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    worker_thread[i].join();
  }
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    for (int j = 0; j < good_target_distance_index_list_temp[i].size(); j++) {
      good_target_distance_index_list_.push_back(good_target_distance_index_list_temp[i][j]);
    }
  }
}
void TrajectoryPlanner2D::CheckDistanceFromTargetsSubProcess(const int &start_idx,
                                                             const int &end_idx,
                                                             IndexList &dist_idx_sub) {
  bool flag_store_in;
  bool flag_store_out;
  float value;
  for (int idx = start_idx; idx < end_idx; idx++) {
    flag_store_out = true;
    for (int k = 0; k < num_target_; k++) {
      flag_store_in = true;
      for (int i = 0; i <= 2 * 5; i++) {
        value = 0.0f;
        for (int j = std::max(0, i - 5); j <= std::min(5, i); j++) {
          value += (float)nchoosek(5, j) * (float)nchoosek(5, i - j) / (float)nchoosek(2 * 5, i) *
                   (primitives_list_[idx].px.GetBernsteinCoefficient()[j] *
                        primitives_list_[idx].px.GetBernsteinCoefficient()[i - j] -
                    2 * primitives_list_[idx].px.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].px.GetBernsteinCoefficient()[i - j] +
                    target_trajectory_list_[k].px.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].px.GetBernsteinCoefficient()[i - j] +
                    primitives_list_[idx].py.GetBernsteinCoefficient()[j] *
                        primitives_list_[idx].py.GetBernsteinCoefficient()[i - j] -
                    2 * primitives_list_[idx].py.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].py.GetBernsteinCoefficient()[i - j] +
                    target_trajectory_list_[k].py.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].py.GetBernsteinCoefficient()[i - j]);
        }
        if (value - param_.distance.target_min * param_.distance.target_min < 0.0f or
            value - param_.distance.target_max * param_.distance.target_max > 0.0f) {
          flag_store_in = false;
          break;
        }
      }
      if (not flag_store_in) {
        flag_store_out = false;
        break;
      }
    }
    if (flag_store_out)
      dist_idx_sub.push_back(idx);
  }
}

bool TrajectoryPlanner3D::PlanKeeperTrajectory() {
  SampleShootingPoints();
  ComputePrimitives();

  return false;
}

void TrajectoryPlanner3D::SampleShootingPoints() {
  shooting_points_.clear();
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread / num_target_;
    vector<thread> worker_thread;
    vector<vector<Point>> shooting_point_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TrajectoryPlanner3D::SampleShootingPointsSubProcess, this, i,
                                 num_chunk, std::ref(shooting_point_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < shooting_point_temp[j].size(); k++) {
        shooting_points_.push_back(shooting_point_temp[j][k]);
      }
    }
  }
}

void TrajectoryPlanner3D::SampleShootingPointsSubProcess(const int &target_id,
                                                         const int &chunk_size,
                                                         PointList &shooting_points_sub) {
  Point end_point_center{target_trajectory_list_[target_id].px.GetTerminalValue(),
                         target_trajectory_list_[target_id].py.GetTerminalValue(),
                         target_trajectory_list_[target_id].pz.GetTerminalValue()};
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> r_dis(param_.distance.target_min, param_.distance.target_max);
  std::uniform_real_distribution<> azimuth_dis(-M_PI, M_PI);
  std::uniform_real_distribution<> elevation_dis(-M_PI, M_PI);
  double r, theta, phi;
  Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
  for (int i = 0; i < chunk_size; i++) {
    r = r_dis(gen);
    theta = azimuth_dis(gen);
    phi = elevation_dis(gen);
    tempPoint.x = float(end_point_center.x + r * cos(phi) * cos(theta));
    tempPoint.y = float(end_point_center.y + r * cos(phi) * sin(theta));
    tempPoint.z = float(end_point_center.z + r * sin(phi));
    shooting_points_sub.push_back(tempPoint);
  }
}

void TrajectoryPlanner3D::ComputePrimitives() {
  primitives_list_.clear();
  int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
  vector<thread> worker_thread;
  PrimitiveListSet primitive_list_temp;
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    worker_thread.emplace_back(&TrajectoryPlanner3D::ComputePrimitivesSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(primitive_list_temp[i]));
  }
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    worker_thread[i].join();
  }
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    for (int j = 0; j < primitive_list_temp[i].size(); j++) {
      primitives_list_.push_back(primitive_list_temp[i][j]);
    }
  }
}

void TrajectoryPlanner3D::ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                      PrimitiveList &primitive_list_sub) {
  StatePoly primitive_temp;
  primitive_temp.SetDegree(5);
  float time_interval_temp[2]{0.0, param_.horizon.planning};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[6];
  float param_horizon_planning_square = param_.horizon.planning * param_.horizon.planning;
  primitive_temp.rx = param_.drone.rx;
  primitive_temp.ry = param_.drone.ry;
  primitive_temp.rz = param_.drone.rz;
  for (int i = start_idx; i < end_idx; i++) {
    { // x-component
      bernstein_coeff_temp[0] = drone_state_.px;
      bernstein_coeff_temp[1] = drone_state_.px + 0.2f * param_.horizon.planning * drone_state_.vx;
      bernstein_coeff_temp[2] = drone_state_.px + 0.4f * param_.horizon.planning * drone_state_.vx +
                                0.05f * param_horizon_planning_square * drone_state_.ax;
      bernstein_coeff_temp[3] = 0.16666667f * shooting_points_[i].x +
                                0.83333333f * drone_state_.px +
                                0.43333333f * param_.horizon.planning * drone_state_.vx +
                                0.06666667f * param_horizon_planning_square * drone_state_.ax;
      bernstein_coeff_temp[4] = 0.5f * shooting_points_[i].x + 0.5f * drone_state_.px +
                                0.3f * param_.horizon.planning * drone_state_.vx +
                                0.05f * param_horizon_planning_square * drone_state_.ax;
      bernstein_coeff_temp[5] = shooting_points_[i].x;
      primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // y-component
      bernstein_coeff_temp[0] = drone_state_.py;
      bernstein_coeff_temp[1] = drone_state_.py + 0.2f * param_.horizon.planning * drone_state_.vy;
      bernstein_coeff_temp[2] = drone_state_.py + 0.4f * param_.horizon.planning * drone_state_.vy +
                                0.05f * param_horizon_planning_square * drone_state_.ay;
      bernstein_coeff_temp[3] = 0.16666667f * shooting_points_[i].y +
                                0.83333333f * drone_state_.py +
                                0.43333333f * param_.horizon.planning * drone_state_.vy +
                                0.06666667f * param_horizon_planning_square * drone_state_.ay;
      bernstein_coeff_temp[4] = 0.5f * shooting_points_[i].y + 0.5f * drone_state_.py +
                                0.3f * param_.horizon.planning * drone_state_.vy +
                                0.05f * param_horizon_planning_square * drone_state_.ay;
      bernstein_coeff_temp[5] = shooting_points_[i].y;
      primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // z-component
      bernstein_coeff_temp[0] = drone_state_.pz;
      bernstein_coeff_temp[1] = drone_state_.pz + 0.2f * param_.horizon.planning * drone_state_.vz;
      bernstein_coeff_temp[2] = drone_state_.pz + 0.4f * param_.horizon.planning * drone_state_.vz +
                                0.05f * param_horizon_planning_square * drone_state_.az;
      bernstein_coeff_temp[3] = 0.16666667f * shooting_points_[i].z +
                                0.83333333f * drone_state_.pz +
                                0.43333333f * param_.horizon.planning * drone_state_.vz +
                                0.06666667f * param_horizon_planning_square * drone_state_.az;
      bernstein_coeff_temp[4] = 0.5f * shooting_points_[i].z + 0.5f * drone_state_.pz +
                                0.3f * param_.horizon.planning * drone_state_.vz +
                                0.05f * param_horizon_planning_square * drone_state_.az;
      bernstein_coeff_temp[5] = shooting_points_[i].z;
      primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    primitive_list_sub.push_back(primitive_temp);
  }
}
TrajectoryPlanner3D::TrajectoryPlanner3D(const PlanningParameter &param)
    : TrajectoryPlanner(param) {}
optional<StatePoly> TrajectoryPlanner3D::ComputeChasingTrajectory(
    const vector<StatePoly> &target_prediction_list, const PclPointCloud &obstacle_points,
    const vector<StatePoly> &structured_obstacle_poly_list) {
  this->SetTargetState(target_prediction_list);
  this->SetObstacleState(obstacle_points, structured_obstacle_poly_list);
  bool plan_success = this->PlanKeeperTrajectory();
  if (plan_success)
    return GetBestKeeperTrajectory();
  else
    return std::nullopt;
}
void TrajectoryPlanner3D::CheckDistanceFromTargets() {
  good_target_distance_index_list_.clear();
  int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
  vector<thread> worker_thread;
  IndexListSet good_target_distance_index_list_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    worker_thread.emplace_back(&TrajectoryPlanner3D::CheckDistanceFromTargetsSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(good_target_distance_index_list_temp[i]));
  }
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    worker_thread[i].join();
  }
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    for (int j = 0; j < good_target_distance_index_list_temp[i].size(); j++) {
      good_target_distance_index_list_.push_back(good_target_distance_index_list_temp[i][j]);
    }
  }
}
void TrajectoryPlanner3D::CheckDistanceFromTargetsSubProcess(const int &start_idx,
                                                             const int &end_idx,
                                                             IndexList &dist_idx_sub) {
  bool flag_store_in;
  bool flag_store_out;
  float value;
  for (int idx = start_idx; idx < end_idx; idx++) {
    flag_store_out = true;
    for (int k = 0; k < num_target_; k++) {
      flag_store_in = true;
      for (int i = 0; i <= 2 * 5; i++) {
        value = 0.0f;
        for (int j = std::max(0, i - 5); j <= std::min(5, i); j++) {
          value += (float)nchoosek(5, j) * (float)nchoosek(5, i - j) / (float)nchoosek(2 * 5, i) *
                   (primitives_list_[idx].px.GetBernsteinCoefficient()[j] *
                        primitives_list_[idx].px.GetBernsteinCoefficient()[i - j] -
                    2 * primitives_list_[idx].px.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].px.GetBernsteinCoefficient()[i - j] +
                    target_trajectory_list_[k].px.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].px.GetBernsteinCoefficient()[i - j] +
                    primitives_list_[idx].py.GetBernsteinCoefficient()[j] *
                        primitives_list_[idx].py.GetBernsteinCoefficient()[i - j] -
                    2 * primitives_list_[idx].py.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].py.GetBernsteinCoefficient()[i - j] +
                    target_trajectory_list_[k].py.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].py.GetBernsteinCoefficient()[i - j] +
                    primitives_list_[idx].pz.GetBernsteinCoefficient()[j] *
                        primitives_list_[idx].pz.GetBernsteinCoefficient()[i - j] -
                    2 * primitives_list_[idx].pz.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].pz.GetBernsteinCoefficient()[i - j] +
                    target_trajectory_list_[k].pz.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].pz.GetBernsteinCoefficient()[i - j]);
        }
        if (value - param_.distance.target_min * param_.distance.target_min < 0.0f or
            value - param_.distance.target_max * param_.distance.target_max > 0.0f) {
          flag_store_in = false;
          break;
        }
      }
      if (not flag_store_in) {
        flag_store_out = false;
        break;
      }
    }
    if (flag_store_out)
      dist_idx_sub.push_back(idx);
  }
}
