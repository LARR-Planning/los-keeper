#include "los_keeper/trajectory_planner/trajectory_planner.h"

using namespace los_keeper;

std::optional<StatePoly> TrajectoryPlanner::ComputeChasingTrajectory(
    const DroneState &drone_state, const std::vector<StatePoly> &target_prediction_list,
    const los_keeper::PclPointCloud &obstacle_points,
    const std::vector<StatePoly> &structured_obstacle_poly_list) {
  return std::nullopt;
}

void TrajectoryPlanner::SetTargetState(const PrimitiveList &target_trajectory_list) {
  target_trajectory_list_.clear();
  for (int i = 0; i < target_trajectory_list.size(); i++) {
    BernsteinPoly px = target_trajectory_list[i].px.ElevateDegree(5);
    BernsteinPoly py = target_trajectory_list[i].py.ElevateDegree(5);
    BernsteinPoly pz = target_trajectory_list[i].pz.ElevateDegree(5);
    StatePoly poly;
    poly.px = px;
    poly.py = py;
    poly.pz = pz;
    poly.rx = target_trajectory_list[i].rx;
    poly.ry = target_trajectory_list[i].ry;
    poly.rz = target_trajectory_list[i].rz;
    target_trajectory_list_.push_back(poly);
  }
  num_target_ = (int)target_trajectory_list.size();
}

void TrajectoryPlanner::SetObstacleState(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                         const PrimitiveList &structured_obstacle_poly_list) {
  structured_obstacle_poly_list_.clear();
  for (int i = 0; i < structured_obstacle_poly_list.size(); i++) {
    BernsteinPoly px = structured_obstacle_poly_list[i].px.ElevateDegree(5);
    BernsteinPoly py = structured_obstacle_poly_list[i].py.ElevateDegree(5);
    BernsteinPoly pz = structured_obstacle_poly_list[i].pz.ElevateDegree(5);
    StatePoly poly;
    poly.px = px;
    poly.py = py;
    poly.pz = pz;
    poly.rx = structured_obstacle_poly_list[i].rx;
    poly.ry = structured_obstacle_poly_list[i].ry;
    poly.rz = structured_obstacle_poly_list[i].rz;
    structured_obstacle_poly_list_.push_back(poly);
  }
  cloud_.points.clear();
  if (not cloud.points.empty())
    cloud_.points = cloud.points;
}

void TrajectoryPlanner::SampleShootingPoints() {}

void TrajectoryPlanner::SampleShootingPointsSubProcess(const int &target_id, const int &chunk_size,
                                                       PointList &shooting_points_sub) {}

void TrajectoryPlanner::ComputePrimitives() {}

void TrajectoryPlanner::ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                    PrimitiveList &primitive_list_sub) {}
TrajectoryPlanner::TrajectoryPlanner(const PlanningParameter &param) { param_ = param; }
StatePoly TrajectoryPlanner::GetBestKeeperTrajectory() { return primitives_list_[best_index_]; }
void TrajectoryPlanner::CheckDistanceFromTargets() {}
void TrajectoryPlanner::CheckDistanceFromTargetsSubProcess(const int &start_idx, const int &end_idx,
                                                           IndexList &dist_idx_sub) {}
PlanningDebugInfo TrajectoryPlanner::GetDebugInfo() const {
  PlanningDebugInfo debug_info;
  debug_info.success_flag = not primitives_list_.empty() and not visible_total_index_.empty();
  debug_info.planning_time = planning_time_;
  if (debug_info.success_flag) {
    debug_info.primitives_list = primitives_list_;
    debug_info.safe_visibility_index = visible_total_index_;
  } else
    debug_info.primitives_list = primitives_list_;
  return debug_info;
}
void TrajectoryPlanner::SetKeeperState(const DroneState &drone_state) {
  drone_state_ = drone_state;
}
bool TrajectoryPlanner::CheckVisibility() {}
bool CheckVisibilityAgainstStructuredObstacle() {}
void TrajectoryPlanner::CheckVisibilityAgainstPcl() {}
void TrajectoryPlanner::CheckVisibilityAgainstStructuredObstacleSubProcess(const int &start_idx,
                                                                           const int &end_idx,
                                                                           IndexList &visible_idx) {
}
void TrajectoryPlanner::CalculateBestIndex() {}
void TrajectoryPlanner::CalculateCloseObstacleIndex() {}
void TrajectoryPlanner::CalculateBestIndexSubProcess(const int &start_idx, const int &end_idx,
                                                     pair<int, float> &min_jerk_pair) {}

bool TrajectoryPlanner2D::PlanKeeperTrajectory() {
  bool plan_success;
  auto check_planning_start = std::chrono::system_clock::now();
  auto check_planning_end = check_planning_start = check_planning_start;
  std::chrono::duration<double> elapsed_check_planning{};
  SampleShootingPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  CheckDistanceFromTargets();
  if (good_target_distance_index_list_.empty()) {
    plan_success = false;
    goto end_process;
  } else {
    CheckVisibility();
    if (visible_total_index_.empty()) {
      plan_success = false;
      goto end_process;
    } else {
      CalculateBestIndex();
      plan_success = true;
      goto end_process;
    }
  }
end_process : {
  check_planning_end = std::chrono::system_clock::now();
  elapsed_check_planning = check_planning_end - check_planning_start;
  planning_time_ = elapsed_check_planning.count();
  return plan_success;
};
}

void TrajectoryPlanner2D::SampleShootingPoints() {
  shooting_points_.clear();
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread / num_target_;
    vector<thread> worker_thread;
    vector<vector<Point>> shooting_point_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++)
      worker_thread.emplace_back(&TrajectoryPlanner2D::SampleShootingPointsSubProcess, this, i,
                                 num_chunk, std::ref(shooting_point_temp[j]));
    for (int j = 0; j < param_.sampling.num_thread; j++)
      worker_thread[j].join();
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
  std::uniform_real_distribution<> r_dis(param_.distance.end_points_min,
                                         param_.distance.end_points_max);
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
  PrimitiveListSet primitive_list_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread.emplace_back(&TrajectoryPlanner2D::ComputePrimitivesSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(primitive_list_temp[i]));
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread[i].join();
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
  double time_interval_temp[2]{drone_state_.t_sec, drone_state_.t_sec + param_.horizon.planning};
  primitive_temp.SetTimeInterval(time_interval_temp);
  BernsteinCoefficients bernstein_coeff_temp(6);
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
    primitive_temp.rx = drone_state_.rx;
    primitive_temp.ry = drone_state_.ry;
    primitive_temp.rz = drone_state_.rz;
    primitive_list_sub.push_back(primitive_temp);
  }
}

TrajectoryPlanner2D::TrajectoryPlanner2D(const PlanningParameter &param)
    : TrajectoryPlanner(param) {}

optional<StatePoly> TrajectoryPlanner2D::ComputeChasingTrajectory(
    const DroneState &drone_state, const vector<StatePoly> &target_prediction_list,
    const PclPointCloud &obstacle_points, const vector<StatePoly> &structured_obstacle_poly_list) {
  this->SetKeeperState(drone_state);
  this->SetTargetState(target_prediction_list);
  this->SetObstacleState(obstacle_points, structured_obstacle_poly_list);
  bool plan_success = this->PlanKeeperTrajectory();
  if (plan_success) {
    //    printf("PLAN SUCCESS\n");
    return GetBestKeeperTrajectory();
  }    // target trajectories exist
  else // no target trajectory exists
    return std::nullopt;
}
void TrajectoryPlanner2D::CalculateCloseObstacleIndex() {
  close_obstacle_index_.clear();
  bool is_close;
  for (int j = 0; j < structured_obstacle_poly_list_.size(); j++) {
    is_close =
        powf(drone_state_.px - structured_obstacle_poly_list_[j].px.GetInitialValue(), 2) +
            powf(drone_state_.py - structured_obstacle_poly_list_[j].py.GetInitialValue(), 2) <
        param_.distance.obstacle_max * param_.distance.obstacle_max;
    if (is_close)
      close_obstacle_index_.push_back(j);
  }
}
void TrajectoryPlanner2D::CheckDistanceFromTargets() {
  good_target_distance_index_list_.clear();
  int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
  vector<thread> worker_thread;
  IndexListSet good_target_distance_index_list_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread.emplace_back(&TrajectoryPlanner2D::CheckDistanceFromTargetsSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(good_target_distance_index_list_temp[i]));
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread[i].join();
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
  float target_distance_squared_min = param_.distance.target_min * param_.distance.target_min;
  float target_distance_squared_max = param_.distance.target_max * param_.distance.target_max;
  for (int idx = start_idx; idx < end_idx; idx++) {
    flag_store_out = true;
    for (int k = 0; k < num_target_; k++) {
      flag_store_in = true;
      for (int i = 0; i <= 10; i++) {
        value = 0.0f;
        for (int j = std::max(0, i - 5); j <= std::min(5, i); j++) {
          value += (float)nchoosek(5, j) * (float)nchoosek(5, i - j) / (float)nchoosek(10, i) *
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
        if (value - target_distance_squared_min < 0.0f or
            value - target_distance_squared_max > 0.0f) {
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
bool TrajectoryPlanner2D::CheckVisibility() {
  visible_total_index_.clear();
  bool is_available_keeper_path;
  if (not cloud_.points.empty())
    CheckVisibilityAgainstPcl();
  if (not structured_obstacle_poly_list_.empty())
    is_available_keeper_path = CheckVisibilityAgainstStructuredObstacle();
  if (cloud_.points.empty() and structured_obstacle_poly_list_.empty()) // Case I: No Obstacle
    visible_total_index_ = good_target_distance_index_list_;
  else if (cloud_.points.empty() and
           (not structured_obstacle_poly_list_.empty())) // Case II: Only Ellipsoidal Obstacle
    visible_total_index_ = visible_structured_index_;
  else if ((not cloud_.points.empty()) and
           structured_obstacle_poly_list_.empty()) // Case III: Only Pcl
    visible_total_index_ = visible_pcl_index_;
  else if (not cloud_.points.empty() and not structured_obstacle_poly_list_.empty()) {
    std::vector<bool> is_visible_pcl_temp;
    std::vector<bool> is_visible_structured_obstacle_temp;
    for (int j = 0; j < param_.sampling.num_sample; j++) {
      is_visible_pcl_temp.push_back(false);
      is_visible_structured_obstacle_temp.push_back(false);
    }
    for (int j : visible_structured_index_)
      is_visible_structured_obstacle_temp[j] = true;
    for (int j : visible_pcl_index_)
      is_visible_pcl_temp[j] = true;
    for (int j = 0; j < param_.sampling.num_sample; j++) {
      if (is_visible_structured_obstacle_temp[j] and is_visible_structured_obstacle_temp[j])
        visible_total_index_.push_back(j);
    }
  }
  return is_available_keeper_path;
}
bool TrajectoryPlanner2D::CheckVisibilityAgainstStructuredObstacle() {
  visible_structured_index_.clear();
  int num_chunk = (int)good_target_distance_index_list_.size() / param_.sampling.num_thread;
  vector<thread> worker_thread;
  IndexListSet visible_structured_index_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread.emplace_back(
        &TrajectoryPlanner2D::CheckVisibilityAgainstStructuredObstacleSubProcess, this,
        num_chunk * (i), num_chunk * (i + 1), std::ref(visible_structured_index_temp[i]));
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread[i].join();
  for (int i = 0; i < param_.sampling.num_thread; i++)
    for (int j = 0; j < visible_structured_index_temp[i].size(); j++)
      visible_structured_index_.push_back(visible_structured_index_temp[i][j]);

  if (visible_structured_index_.empty())
    return false;
  return true;
}
void TrajectoryPlanner2D::CheckVisibilityAgainstStructuredObstacleSubProcess(
    const int &start_idx, const int &end_idx, IndexList &visible_idx) {
  bool flag_store_in1 = true;     // collision between obstacle and keeper
  bool flag_store_in2 = true;     // LOS from obstacles (target)
  bool flag_store_in2_sub = true; // LOS from obstacles
  bool flag_store_out = true;     //
  float rx_enlarged_squared_inverse;
  float ry_enlarged_squared_inverse;
  float rx_squared_inverse;
  float ry_squared_inverse;
  float value;
  for (int idx = start_idx; idx < end_idx; idx++) {
    flag_store_out = true;
    for (int i = 0; i < close_obstacle_index_.size(); i++) {
      rx_enlarged_squared_inverse =
          1 / powf(primitives_list_[good_target_distance_index_list_[idx]].rx +
                       structured_obstacle_poly_list_[close_obstacle_index_[i]].rx +
                       param_.safe_distance.rx,
                   2);
      ry_enlarged_squared_inverse =
          1 / powf(primitives_list_[good_target_distance_index_list_[idx]].ry +
                       structured_obstacle_poly_list_[close_obstacle_index_[i]].ry +
                       param_.safe_distance.ry,
                   2);
      flag_store_in1 = true;
      for (int j = 0; j <= 10; j++) {
        value = 0.0f;
        for (int k = std::max(0, j - 5); k <= std::min(5, j); k++) {
          value += (float)nchoosek(5, k) * (float)nchoosek(5, j - k) / (float)nchoosek(10, j) *
                   ((primitives_list_[good_target_distance_index_list_[idx]]
                             .px.GetBernsteinCoefficient()[k] *
                         primitives_list_[good_target_distance_index_list_[idx]]
                             .px.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .px.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .px.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .px.GetBernsteinCoefficient()[j - k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .px.GetBernsteinCoefficient()[k] +
                     structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .px.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .px.GetBernsteinCoefficient()[j - k]) *
                        rx_enlarged_squared_inverse + // x-components
                    (primitives_list_[good_target_distance_index_list_[idx]]
                             .py.GetBernsteinCoefficient()[k] *
                         primitives_list_[good_target_distance_index_list_[idx]]
                             .py.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .py.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .py.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .py.GetBernsteinCoefficient()[j - k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .py.GetBernsteinCoefficient()[k] +
                     structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .py.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .py.GetBernsteinCoefficient()[j - k]) *
                        ry_enlarged_squared_inverse);
        }
        if (value < 1.0f) {
          flag_store_in1 = false;
          break;
        }
      }
      if (not flag_store_in1) {
        flag_store_out = false;
        break;
      }
    }
    if (not flag_store_out)
      continue;
    for (int i = 0; i < num_target_; i++) {
      flag_store_in2 = true;
      for (int j = 0; j < close_obstacle_index_.size(); j++) {
        rx_squared_inverse =
            1 / powf(structured_obstacle_poly_list_[close_obstacle_index_[j]].rx, 2);
        ry_squared_inverse =
            1 / powf(structured_obstacle_poly_list_[close_obstacle_index_[j]].ry, 2);
        flag_store_in2_sub = true;
        for (int k = 0; k <= 10; k++) {
          value = 0.0f;
          for (int l = std::max(0, k - 5); l <= std::min(5, k); l++) {
            value += (float)nchoosek(5, l) * (float)nchoosek(5, k - l) / (float)nchoosek(10, k) *
                     ((primitives_list_[good_target_distance_index_list_[idx]]
                               .px.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].px.GetBernsteinCoefficient()[k - l] -
                       primitives_list_[good_target_distance_index_list_[idx]]
                               .px.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .px.GetBernsteinCoefficient()[k - l] -
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .px.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].px.GetBernsteinCoefficient()[k - l] +
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .px.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .px.GetBernsteinCoefficient()[k - l]) *
                          rx_squared_inverse +
                      (primitives_list_[good_target_distance_index_list_[idx]]
                               .py.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].py.GetBernsteinCoefficient()[k - l] -
                       primitives_list_[good_target_distance_index_list_[idx]]
                               .py.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .py.GetBernsteinCoefficient()[k - l] -
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .py.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].py.GetBernsteinCoefficient()[k - l] +
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .py.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .py.GetBernsteinCoefficient()[k - l]) *
                          ry_squared_inverse);
          }
          if (value < 1.0f) {
            flag_store_in2_sub = false;
            break;
          }
        }
        if (not flag_store_in2_sub) {
          flag_store_in2 = false;
          break;
        }
      }
      if (not flag_store_in2) {
        flag_store_out = false;
        break;
      }
    }
    if (flag_store_out)
      visible_idx.push_back(good_target_distance_index_list_[idx]);
  }
}
void TrajectoryPlanner2D::CheckVisibilityAgainstPcl() {
  TrajectoryPlanner::CheckVisibilityAgainstPcl();
}
void TrajectoryPlanner2D::CalculateBestIndex() {
  int num_chunk = visible_total_index_.size() / param_.sampling.num_thread;
  vector<thread> worker_thread;
  vector<pair<int, float>> min_jerk_pair_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread.emplace_back(&TrajectoryPlanner2D::CalculateBestIndexSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(min_jerk_pair_temp[i]));
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread[i].join();
  float min_jerk_temp = 9999.0f;
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    if (min_jerk_temp > min_jerk_pair_temp[i].second) {
      min_jerk_temp = min_jerk_pair_temp[i].second;
      best_index_ = min_jerk_pair_temp[i].first;
    }
  }
}
void TrajectoryPlanner2D::CalculateBestIndexSubProcess(const int &start_idx, const int &end_idx,
                                                       pair<int, float> &min_jerk_pair) {
  int chunk_size = end_idx - start_idx;
  float jerk_integral_list[chunk_size];
  float jerk_coeff_x[3];
  float jerk_coeff_y[3];
  for (int i = 0; i < chunk_size; i++) {
    jerk_coeff_x[0] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[3] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[2] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[1] -
         primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[0]));
    jerk_coeff_x[1] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[4] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[3] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[2] -
         primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[1]));
    jerk_coeff_x[2] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[5] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[4] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[3] -
         primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[2]));
    jerk_coeff_y[0] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[3] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[2] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[1] -
         primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[0]));
    jerk_coeff_y[1] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[4] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[3] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[2] -
         primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[1]));
    jerk_coeff_y[2] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[5] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[4] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[3] -
         primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[2]));
    jerk_integral_list[i - start_idx] =
        jerk_coeff_x[0] * jerk_coeff_x[0] + jerk_coeff_x[0] * jerk_coeff_x[1] +
        0.33333333f * jerk_coeff_x[0] * jerk_coeff_x[2] +
        0.6666667f * jerk_coeff_x[1] * jerk_coeff_x[1] + jerk_coeff_x[1] * jerk_coeff_x[2] +
        jerk_coeff_x[2] * jerk_coeff_x[2] + jerk_coeff_y[0] * jerk_coeff_y[0] +
        jerk_coeff_y[0] * jerk_coeff_y[1] + 0.33333333f * jerk_coeff_y[0] * jerk_coeff_y[2] +
        0.6666667f * jerk_coeff_y[1] * jerk_coeff_y[1] + jerk_coeff_y[1] * jerk_coeff_y[2] +
        jerk_coeff_y[2] * jerk_coeff_y[2];
  }
  min_jerk_pair.second = 9999.0f;
  for (int i = 0; i < chunk_size; i++) {
    if (min_jerk_pair.second > jerk_integral_list[i]) {
      min_jerk_pair.second = jerk_integral_list[i];
      min_jerk_pair.first = visible_total_index_[i + start_idx];
    }
  }
}

bool TrajectoryPlanner3D::PlanKeeperTrajectory() {
  bool plan_success;
  auto check_planning_start = std::chrono::system_clock::now();
  auto check_planning_end = check_planning_start = check_planning_start;
  std::chrono::duration<double> elapsed_check_planning{};
  SampleShootingPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  CheckDistanceFromTargets();
  if (good_target_distance_index_list_.empty()) {
    plan_success = false;
    goto end_process;
  } else {
    CheckVisibility();
    if (visible_total_index_.empty()) {
      plan_success = false;
      goto end_process;
    } else {
      CalculateBestIndex();
      plan_success = true;
      goto end_process;
    }
  }
end_process : {
  check_planning_end = std::chrono::system_clock::now();
  elapsed_check_planning = check_planning_end - check_planning_start;
  planning_time_ = elapsed_check_planning.count();
  return plan_success;
};
}

void TrajectoryPlanner3D::SampleShootingPoints() {
  shooting_points_.clear();
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread / num_target_;
    vector<thread> worker_thread;
    PointListSet shooting_point_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++)
      worker_thread.emplace_back(&TrajectoryPlanner3D::SampleShootingPointsSubProcess, this, i,
                                 num_chunk, std::ref(shooting_point_temp[j]));
    for (int j = 0; j < param_.sampling.num_thread; j++)
      worker_thread[j].join();
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
  std::uniform_real_distribution<> r_dis(param_.distance.end_points_min,
                                         param_.distance.end_points_max);
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
  PrimitiveListSet primitive_list_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread.emplace_back(&TrajectoryPlanner3D::ComputePrimitivesSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(primitive_list_temp[i]));
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread[i].join();
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
  double time_interval_temp[2]{drone_state_.t_sec, drone_state_.t_sec + param_.horizon.planning};
  primitive_temp.SetTimeInterval(time_interval_temp);
  BernsteinCoefficients bernstein_coeff_temp(6);
  float param_horizon_planning_square = param_.horizon.planning * param_.horizon.planning;
  primitive_temp.rx = drone_state_.rx;
  primitive_temp.ry = drone_state_.ry;
  primitive_temp.rz = drone_state_.rz;
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
    const DroneState &drone_state, const vector<StatePoly> &target_prediction_list,
    const PclPointCloud &obstacle_points, const vector<StatePoly> &structured_obstacle_poly_list) {
  this->SetKeeperState(drone_state);
  this->SetTargetState(target_prediction_list);
  this->SetObstacleState(obstacle_points, structured_obstacle_poly_list);
  bool plan_success = this->PlanKeeperTrajectory();
  if (plan_success) {
    return GetBestKeeperTrajectory();
  } else
    return std::nullopt;
}
void TrajectoryPlanner3D::CheckDistanceFromTargets() {
  good_target_distance_index_list_.clear();
  int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
  vector<thread> worker_thread;
  IndexListSet good_target_distance_index_list_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread.emplace_back(&TrajectoryPlanner3D::CheckDistanceFromTargetsSubProcess, this,
                               num_chunk * (i), num_chunk * (i + 1),
                               std::ref(good_target_distance_index_list_temp[i]));
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread[i].join();
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    for (int j = 0; j < good_target_distance_index_list_temp[i].size(); j++) {
      good_target_distance_index_list_.push_back(good_target_distance_index_list_temp[i][j]);
    }
  }
}
void TrajectoryPlanner3D::CheckDistanceFromTargetsSubProcess(const int &start_idx,
                                                             const int &end_idx,
                                                             IndexList &dist_idx_sub) {
  bool flag_store_in, flag_store_out;
  float value;
  float target_distance_squared_min = param_.distance.target_min * param_.distance.target_min;
  float target_distance_squared_max = param_.distance.target_max * param_.distance.target_max;
  for (int idx = start_idx; idx < end_idx; idx++) {
    flag_store_out = true;
    for (int k = 0; k < num_target_; k++) {
      flag_store_in = true;
      for (int i = 0; i <= 10; i++) {
        value = 0.0f;
        for (int j = std::max(0, i - 5); j <= std::min(5, i); j++) {
          value += (float)nchoosek(5, j) * (float)nchoosek(5, i - j) / (float)nchoosek(10, i) *
                   (primitives_list_[idx].px.GetBernsteinCoefficient()[j] *
                        primitives_list_[idx].px.GetBernsteinCoefficient()[i - j] -
                    primitives_list_[idx].px.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].px.GetBernsteinCoefficient()[i - j] -
                    primitives_list_[idx].px.GetBernsteinCoefficient()[i - j] *
                        target_trajectory_list_[k].px.GetBernsteinCoefficient()[j] +
                    target_trajectory_list_[k].px.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].px.GetBernsteinCoefficient()[i - j] + // x-comp
                    primitives_list_[idx].py.GetBernsteinCoefficient()[j] *
                        primitives_list_[idx].py.GetBernsteinCoefficient()[i - j] -
                    primitives_list_[idx].py.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].py.GetBernsteinCoefficient()[i - j] -
                    primitives_list_[idx].py.GetBernsteinCoefficient()[i - j] *
                        target_trajectory_list_[k].py.GetBernsteinCoefficient()[j] +
                    target_trajectory_list_[k].py.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].py.GetBernsteinCoefficient()[i - j] + // y-comp
                    primitives_list_[idx].pz.GetBernsteinCoefficient()[j] *
                        primitives_list_[idx].pz.GetBernsteinCoefficient()[i - j] -
                    primitives_list_[idx].pz.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].pz.GetBernsteinCoefficient()[i - j] -
                    primitives_list_[idx].pz.GetBernsteinCoefficient()[i - j] *
                        target_trajectory_list_[k].pz.GetBernsteinCoefficient()[j] +
                    target_trajectory_list_[k].pz.GetBernsteinCoefficient()[j] *
                        target_trajectory_list_[k].pz.GetBernsteinCoefficient()[i - j]); // z-comp
        }
        if (value - target_distance_squared_min < 0.0f or
            value - target_distance_squared_max > 0.0f) {
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
bool TrajectoryPlanner3D::CheckVisibility() {
  visible_total_index_.clear();
  bool is_available_keeper_path;
  if (not cloud_.points.empty())
    CheckVisibilityAgainstPcl();
  if (not structured_obstacle_poly_list_.empty())
    is_available_keeper_path = CheckVisibilityAgainstStructuredObstacle();
  if (cloud_.points.empty() and structured_obstacle_poly_list_.empty()) // Case I: No Obstacle
    visible_total_index_ = good_target_distance_index_list_;
  else if (cloud_.points.empty() and
           not structured_obstacle_poly_list_.empty()) // Case II: Only Ellipsoidal Obstacle
    visible_total_index_ = visible_structured_index_;
  else if (not cloud_.points.empty() and
           structured_obstacle_poly_list_.empty()) // Case III: Only Pcl
    visible_total_index_ = visible_pcl_index_;
  else if (not cloud_.points.empty() and not structured_obstacle_poly_list_.empty()) {
    vector<bool> is_visible_pcl_temp;
    vector<bool> is_visible_structured_obstacle_temp;
    for (int j = 0; j < param_.sampling.num_sample; j++) {
      is_visible_pcl_temp.push_back(false);
      is_visible_structured_obstacle_temp.push_back(false);
    }
    for (int j : visible_structured_index_)
      is_visible_structured_obstacle_temp[j] = true;
    for (int j : visible_pcl_index_)
      is_visible_pcl_temp[j] = true;
    for (int j = 0; j < param_.sampling.num_sample; j++) {
      if (is_visible_structured_obstacle_temp[j] and is_visible_structured_obstacle_temp[j])
        visible_total_index_.push_back(j);
    }
  }
  return is_available_keeper_path;
}
bool TrajectoryPlanner3D::CheckVisibilityAgainstStructuredObstacle() {
  visible_structured_index_.clear();
  int num_chunk = good_target_distance_index_list_.size() / param_.sampling.num_thread;
  vector<thread> worker_thread;
  IndexListSet visible_structured_index_temp(param_.sampling.num_thread);
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread.emplace_back(
        &TrajectoryPlanner3D::CheckVisibilityAgainstStructuredObstacleSubProcess, this,
        num_chunk * (i), num_chunk * (i + 1), std::ref(visible_structured_index_temp[i]));
  for (int i = 0; i < param_.sampling.num_thread; i++)
    worker_thread[i].join();
  for (int i = 0; i < param_.sampling.num_thread; i++) {
    for (int j = 0; j < visible_structured_index_temp[i].size(); j++)
      visible_structured_index_.push_back(visible_structured_index_temp[i][j]);
  }
  if (visible_structured_index_.empty())
    return false;
  return true;
}
void TrajectoryPlanner3D::CheckVisibilityAgainstStructuredObstacleSubProcess(
    const int &start_idx, const int &end_idx, IndexList &visible_idx) {
  bool flag_store_in1 = true; // collision between obstacle and keeper
  bool flag_store_in2 = true; // LOS from obstacles (target)
  bool flag_store_in2_sub = true;
  bool flag_store_out = true;
  float rx_enlarged_squared_inverse, ry_enlarged_squared_inverse, rz_enlarged_squared_inverse;
  float rx_squared_inverse, ry_squared_inverse, rz_squared_inverse;
  float value;
  for (int idx = start_idx; idx < end_idx; idx++) {
    flag_store_out = true;
    for (int i = 0; i < close_obstacle_index_.size(); i++) {
      rx_enlarged_squared_inverse =
          1 / powf(primitives_list_[good_target_distance_index_list_[idx]].rx +
                       structured_obstacle_poly_list_[close_obstacle_index_[i]].rx +
                       param_.safe_distance.rx,
                   2);
      ry_enlarged_squared_inverse =
          1 / powf(primitives_list_[good_target_distance_index_list_[idx]].ry +
                       structured_obstacle_poly_list_[close_obstacle_index_[i]].ry +
                       param_.safe_distance.ry,
                   2);
      rz_enlarged_squared_inverse =
          1 / powf(primitives_list_[good_target_distance_index_list_[idx]].rz +
                       structured_obstacle_poly_list_[close_obstacle_index_[i]].rz +
                       param_.safe_distance.rz,
                   2);
      flag_store_in1 = true;
      for (int j = 0; j <= 10; j++) {
        value = 0.0f;
        for (int k = std::max(0, j - 5); k <= std::min(5, j); k++) {
          value += (float)nchoosek(5, k) * (float)nchoosek(5, j - k) / (float)nchoosek(10, j) *
                   ((primitives_list_[good_target_distance_index_list_[idx]]
                             .px.GetBernsteinCoefficient()[k] *
                         primitives_list_[good_target_distance_index_list_[idx]]
                             .px.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .px.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .px.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .px.GetBernsteinCoefficient()[j - k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .px.GetBernsteinCoefficient()[k] +
                     structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .px.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .px.GetBernsteinCoefficient()[j - k]) *
                        rx_enlarged_squared_inverse + // x-components
                    (primitives_list_[good_target_distance_index_list_[idx]]
                             .py.GetBernsteinCoefficient()[k] *
                         primitives_list_[good_target_distance_index_list_[idx]]
                             .py.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .py.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .py.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .py.GetBernsteinCoefficient()[j - k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .py.GetBernsteinCoefficient()[k] +
                     structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .py.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .py.GetBernsteinCoefficient()[j - k]) *
                        ry_enlarged_squared_inverse + // y-components
                    (primitives_list_[good_target_distance_index_list_[idx]]
                             .pz.GetBernsteinCoefficient()[k] *
                         primitives_list_[good_target_distance_index_list_[idx]]
                             .pz.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .pz.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .pz.GetBernsteinCoefficient()[j - k] -
                     primitives_list_[good_target_distance_index_list_[idx]]
                             .pz.GetBernsteinCoefficient()[j - k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .pz.GetBernsteinCoefficient()[k] +
                     structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .pz.GetBernsteinCoefficient()[k] *
                         structured_obstacle_poly_list_[close_obstacle_index_[i]]
                             .pz.GetBernsteinCoefficient()[j - k]) * // z-components
                        rz_enlarged_squared_inverse);
        }
        if (value < 1.0f) {
          flag_store_in1 = false;
          break;
        }
      }
      if (not flag_store_in1) {
        flag_store_out = false;
        break;
      }
    }
    if (not flag_store_out)
      continue;
    for (int i = 0; i < num_target_; i++) {
      flag_store_in2 = true;
      for (int j = 0; j < close_obstacle_index_.size(); j++) {
        rx_squared_inverse =
            1 / powf(structured_obstacle_poly_list_[close_obstacle_index_[j]].rx, 2);
        ry_squared_inverse =
            1 / powf(structured_obstacle_poly_list_[close_obstacle_index_[j]].ry, 2);
        rz_squared_inverse =
            1 / powf(structured_obstacle_poly_list_[close_obstacle_index_[j]].rz, 2);
        flag_store_in2_sub = true;
        for (int k = 0; k <= 10; k++) {
          value = 0.0f;
          for (int l = std::max(0, k - 5); l <= std::min(5, k); l++) {
            value += (float)nchoosek(5, l) * (float)nchoosek(5, k - l) / (float)nchoosek(10, k) *
                     ((primitives_list_[good_target_distance_index_list_[idx]]
                               .px.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].px.GetBernsteinCoefficient()[k - l] -
                       primitives_list_[good_target_distance_index_list_[idx]]
                               .px.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .px.GetBernsteinCoefficient()[k - l] -
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .px.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].px.GetBernsteinCoefficient()[k - l] +
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .px.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .px.GetBernsteinCoefficient()[k - l]) *
                          rx_squared_inverse +
                      (primitives_list_[good_target_distance_index_list_[idx]]
                               .py.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].py.GetBernsteinCoefficient()[k - l] -
                       primitives_list_[good_target_distance_index_list_[idx]]
                               .py.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .py.GetBernsteinCoefficient()[k - l] -
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .py.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].py.GetBernsteinCoefficient()[k - l] +
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .py.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .py.GetBernsteinCoefficient()[k - l]) *
                          ry_squared_inverse +
                      (primitives_list_[good_target_distance_index_list_[idx]]
                               .pz.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].pz.GetBernsteinCoefficient()[k - l] -
                       primitives_list_[good_target_distance_index_list_[idx]]
                               .pz.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .pz.GetBernsteinCoefficient()[k - l] -
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .pz.GetBernsteinCoefficient()[l] *
                           target_trajectory_list_[i].pz.GetBernsteinCoefficient()[k - l] +
                       structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .pz.GetBernsteinCoefficient()[l] *
                           structured_obstacle_poly_list_[close_obstacle_index_[j]]
                               .pz.GetBernsteinCoefficient()[k - l]) *
                          rz_squared_inverse);
          }
          if (value < 1.0f) {
            flag_store_in2_sub = false;
            break;
          }
        }
        if (not flag_store_in2_sub) {
          flag_store_in2 = false;
          break;
        }
      }
      if (not flag_store_in2) {
        flag_store_out = false;
        break;
      }
    }
    if (flag_store_out)
      visible_idx.push_back(good_target_distance_index_list_[idx]);
  }
}
void TrajectoryPlanner3D::CheckVisibilityAgainstPcl() {
  TrajectoryPlanner::CheckVisibilityAgainstPcl();
}
void TrajectoryPlanner3D::CalculateBestIndex() {
  //  best_index_ = -1;
  //  int num_chunk = visible_total_index_.size() / param_.sampling.num_thread;
  //  vector<thread> worker_thread;
  //  vector<pair<int, float>> min_jerk_pair_temp(param_.sampling.num_thread);
  //  for (int i = 0; i < param_.sampling.num_thread; i++) {
  //    worker_thread.emplace_back(&TrajectoryPlanner3D::CalculateBestIndexSubProcess, this,
  //                               num_chunk * (i), num_chunk * (i + 1),
  //                               std::ref(min_jerk_pair_temp[i]));
  //  }
  //  for (int i = 0; i < param_.sampling.num_thread; i++) {
  //    worker_thread[i].join();
  //  }
  //  float min_jerk_temp = 999999.0f;
  //  for (int i = 0; i < param_.sampling.num_thread; i++) {
  //    if (min_jerk_temp > min_jerk_pair_temp[i].second) {
  //      min_jerk_temp = min_jerk_pair_temp[i].second;
  //      best_index_ = min_jerk_pair_temp[i].first;
  //    }
  //  }
  best_index_ = visible_total_index_[0];
}
void TrajectoryPlanner3D::CalculateCloseObstacleIndex() {
  close_obstacle_index_.clear();
  bool is_close;
  for (int j = 0; j < structured_obstacle_poly_list_.size(); j++) {
    is_close =
        powf(drone_state_.px - structured_obstacle_poly_list_[j].px.GetInitialValue(), 2) +
            powf(drone_state_.py - structured_obstacle_poly_list_[j].py.GetInitialValue(), 2) +
            powf(drone_state_.pz - structured_obstacle_poly_list_[j].pz.GetInitialValue(), 2) <
        param_.distance.obstacle_max * param_.distance.obstacle_max;
    if (is_close)
      close_obstacle_index_.push_back(j);
  }
}
void TrajectoryPlanner3D::CalculateBestIndexSubProcess(const int &start_idx, const int &end_idx,
                                                       pair<int, float> &min_jerk_pair) {
  int chunk_size = end_idx - start_idx;
  float jerk_integral_list[chunk_size];
  float jerk_coeff_x[3], jerk_coeff_y[3], jerk_coeff_z[3];
  for (int i = 0; i < chunk_size; i++) {
    jerk_coeff_x[0] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[3] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[2] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[1] -
         primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[0]));
    jerk_coeff_x[1] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[4] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[3] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[2] -
         primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[1]));
    jerk_coeff_x[2] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].px.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[5] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[4] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[3] -
         primitives_list_[visible_total_index_[i + start_idx]].px.GetBernsteinCoefficient()[2]));
    jerk_coeff_y[0] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[3] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[2] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[1] -
         primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[0]));
    jerk_coeff_y[1] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[4] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[3] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[2] -
         primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[1]));
    jerk_coeff_y[2] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].py.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[5] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[4] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[3] -
         primitives_list_[visible_total_index_[i + start_idx]].py.GetBernsteinCoefficient()[2]));
    jerk_coeff_z[0] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].pz.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].pz.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[3] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[2] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[1] -
         primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[0]));
    jerk_coeff_z[1] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].pz.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].pz.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[4] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[3] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[2] -
         primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[1]));
    jerk_coeff_z[2] = float(
        60.0 /
        pow(primitives_list_[visible_total_index_[i + start_idx]].pz.GetTimeInterval()[1] -
                primitives_list_[visible_total_index_[i + start_idx]].pz.GetTimeInterval()[0],
            3) *
        (primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[5] -
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[4] +
         3.0 *
             primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[3] -
         primitives_list_[visible_total_index_[i + start_idx]].pz.GetBernsteinCoefficient()[2]));
    jerk_integral_list[i - start_idx] =
        jerk_coeff_x[0] * jerk_coeff_x[0] + jerk_coeff_x[0] * jerk_coeff_x[1] +
        0.33333333f * jerk_coeff_x[0] * jerk_coeff_x[2] +
        0.6666667f * jerk_coeff_x[1] * jerk_coeff_x[1] + jerk_coeff_x[1] * jerk_coeff_x[2] +
        jerk_coeff_x[2] * jerk_coeff_x[2] + jerk_coeff_y[0] * jerk_coeff_y[0] +
        jerk_coeff_y[0] * jerk_coeff_y[1] + 0.33333333f * jerk_coeff_y[0] * jerk_coeff_y[2] +
        0.6666667f * jerk_coeff_y[1] * jerk_coeff_y[1] + jerk_coeff_y[1] * jerk_coeff_y[2] +
        jerk_coeff_y[2] * jerk_coeff_y[2] + jerk_coeff_z[0] * jerk_coeff_z[0] +
        jerk_coeff_z[0] * jerk_coeff_z[1] + 0.33333333f * jerk_coeff_z[0] * jerk_coeff_z[2] +
        0.6666667f * jerk_coeff_z[1] * jerk_coeff_z[1] + jerk_coeff_z[1] * jerk_coeff_z[2] +
        jerk_coeff_z[2] * jerk_coeff_z[2];
  }
  min_jerk_pair.second = 9999.0f;
  for (int i = 0; i < chunk_size; i++) {
    if (min_jerk_pair.second > jerk_integral_list[i]) {
      min_jerk_pair.second = jerk_integral_list[i];
      min_jerk_pair.first = visible_total_index_[i + start_idx];
    }
  }
}
