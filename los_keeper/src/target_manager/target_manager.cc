#include "los_keeper/target_manager/target_manager.h"

using namespace std;

// bool los_keeper::TargetManager::PredictTargetTrajectory() {}

void los_keeper::TargetManager::SampleEndPoints(const vector<ObjectState> &target_state_list) {}

los_keeper::TargetManager::TargetManager() { // Abstract Target Manager
}

void los_keeper::TargetManager::ComputePrimitives(const vector<ObjectState> &target_state_list) {}

void los_keeper::TargetManager::CalculateCloseObstacleIndex(
    const vector<ObjectState> &target_state_list,
    const vector<StatePoly> &structured_obstacle_poly_list) {}

void los_keeper::TargetManager::CalculateCentroid() {}

void los_keeper::TargetManager::CheckPclCollision(const los_keeper::PclPointCloud &point_cloud) {}

bool los_keeper::TargetManager::CheckStructuredObstacleCollision(
    const vector<StatePoly> &structured_obstacle_poly_list) {}

void los_keeper::TargetManager::SampleEndPointsSubProcess(
    const int &target_id, const int &chunk_size, const vector<ObjectState> &target_state_list,
    PointList &endpoint_sub) {}

void los_keeper::TargetManager::CheckStructuredObstacleCollisionSubProcess(
    const int &target_id, const int &start_idx, const int &end_idx,
    const vector<StatePoly> &structured_obstacle_poly_list, IndexList &safe_structured_index_sub) {}

void los_keeper::TargetManager::CalculateCentroidSubProcess(const int &target_id,
                                                            const int &start_idx,
                                                            const int &end_idx,
                                                            pair<int, float> &min_dist) {}
PrimitiveList los_keeper::TargetManager::GetTargetPredictionResult() {
  PrimitiveList prediction_result;
  for (int i = 0; i < num_target_; i++)
    prediction_result.push_back(primitives_list_[i][primitive_best_index_[i]]);

  return prediction_result;
}
los_keeper::TargetManagerDebugInfo los_keeper::TargetManager::GetDebugInfo() const {
  TargetManagerDebugInfo debug_info;
  static uint seq = 0;
  debug_info.num_target = num_target_;
  debug_info.seq = ++seq;
  debug_info.success_flag = not primitives_list_.empty() and
                            not primitive_safe_total_index_.empty() and
                            not primitive_best_index_.empty();
  debug_info.prediction_time = prediction_time_;
  if (debug_info.success_flag) {
    debug_info.primitives_list = primitives_list_;
    debug_info.primitive_safe_total_index = primitive_safe_total_index_;
    debug_info.primitive_best_index = primitive_best_index_;
  } else {
    debug_info.primitives_list = primitives_list_;
  }
  return debug_info;
}
void los_keeper::TargetManager::ComputePrimitivesSubProcess(
    const int &target_id, const int &start_idx, const int &end_idx,
    const vector<ObjectState> &target_state_list, PrimitiveList &primitive_list_sub) {}

void los_keeper::TargetManager2D::SampleEndPoints(const vector<ObjectState> &target_state_list) {
  end_points_.clear();
  end_points_.resize(num_target_); //
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    vector<vector<Point>> end_point_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager2D::SampleEndPointsSubProcess, this, i, num_chunk,
                                 target_state_list, std::ref(end_point_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < end_point_temp[j].size(); k++) {
        end_points_[i].push_back(end_point_temp[j][k]);
      }
    }
  }
}

void los_keeper::TargetManager2D::ComputePrimitives(const vector<ObjectState> &target_state_list) {
  primitives_list_.clear();
  primitives_list_.resize(num_target_);

  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    PrimitiveListSet primitive_list_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager2D::ComputePrimitivesSubProcess, this, i,
                                 num_chunk * (j), num_chunk * (j + 1), target_state_list,
                                 std::ref(primitive_list_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < primitive_list_temp[j].size(); k++) {
        primitives_list_[i].push_back(primitive_list_temp[j][k]);
      }
    }
  }
}

void los_keeper::TargetManager2D::CalculateCloseObstacleIndex(
    const vector<ObjectState> &target_state_list,
    const vector<StatePoly> &structured_obstacle_poly_list) {
  close_obstacle_index_.clear();
  bool is_close;
  for (int i = 0; i < num_target_; i++) {
    IndexList close_obstacle_index_temp;
    for (int j = 0; j < structured_obstacle_poly_list.size(); j++) {
      is_close =
          powf(target_state_list[i].px - structured_obstacle_poly_list[j].px.GetInitialValue(), 2) +
              powf(target_state_list[i].py - structured_obstacle_poly_list[j].py.GetInitialValue(),
                   2) <
          param_.distance.obstacle_max * param_.distance.obstacle_max;
      if (is_close)
        close_obstacle_index_temp.push_back(j);
    }
    close_obstacle_index_.push_back(close_obstacle_index_temp);
  }
}

bool los_keeper::TargetManager2D::CheckCollision(
    const los_keeper::PclPointCloud &point_cloud,
    const vector<StatePoly> &structured_obstacle_poly_list) {
  primitive_safe_total_index_.clear();
  bool is_cloud_empty = point_cloud.points.empty();
  bool is_structured_obstacle_empty = structured_obstacle_poly_list.empty();
  bool is_available_target_path;
  if (not is_cloud_empty)
    CheckPclCollision(point_cloud);
  if (not is_structured_obstacle_empty)
    is_available_target_path = CheckStructuredObstacleCollision(structured_obstacle_poly_list);
  if (is_cloud_empty and is_structured_obstacle_empty) { // Case I: No Obstacle
    for (int i = 0; i < num_target_; i++) {
      IndexList primitive_safe_total_index_temp_;
      for (int j = 0; j < param_.sampling.num_sample; j++) {
        primitive_safe_total_index_temp_.push_back(j);
      }
      primitive_safe_total_index_.push_back(primitive_safe_total_index_temp_);
    }
  } else if (is_cloud_empty and
             (not is_structured_obstacle_empty)) { // Case II: Only Ellipsoidal Obstacle
    primitive_safe_total_index_ = primitive_safe_structured_obstacle_index_;
  } else if ((not is_cloud_empty) and
             is_structured_obstacle_empty) { // Case III: Only Unstructured Obstacle
    primitive_safe_total_index_ = primitive_safe_pcl_index_;
  } else if (not is_cloud_empty and not is_structured_obstacle_empty) { // Case IV: Ellipsoidal and
    // Unstructured Obstacle
    for (int i = 0; i < num_target_; i++) {
      std::vector<bool> is_primitive_safe_pcl_temp;
      std::vector<bool> is_primitive_safe_structured_obstacle_temp;
      IndexList primitive_safe_total_index_temp;
      for (int j = 0; j < param_.sampling.num_sample; j++) {
        is_primitive_safe_pcl_temp.push_back(false);
        is_primitive_safe_structured_obstacle_temp.push_back(false);
      }
      for (int j : primitive_safe_structured_obstacle_index_[i]) {
        is_primitive_safe_structured_obstacle_temp[j] = true;
      }
      for (int j : primitive_safe_pcl_index_[i]) {
        is_primitive_safe_pcl_temp[j] = true;
      }
      for (int j = 0; j < param_.sampling.num_sample; j++) {
        if (is_primitive_safe_pcl_temp[j] and is_primitive_safe_structured_obstacle_temp[j])
          primitive_safe_total_index_temp.push_back(j);
      }
      primitive_safe_total_index_.push_back(primitive_safe_total_index_temp);
    }
  }
  return is_available_target_path;
}

void los_keeper::TargetManager2D::CalculateCentroid() {
  primitive_best_index_.clear();
  primitive_best_index_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = (int)primitive_safe_total_index_[i].size() / param_.sampling.num_thread;
    vector<thread> worker_thread;
    vector<std::pair<int, float>> min_dist_pair_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager2D::CalculateCentroidSubProcess, this, i,
                                 num_chunk * (j), num_chunk * (j + 1),
                                 std::ref(min_dist_pair_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    float min_dist_temp = 99999.0f;
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      if (min_dist_temp > min_dist_pair_temp[j].second) {
        min_dist_temp = min_dist_pair_temp[j].second;
        primitive_best_index_[i] = min_dist_pair_temp[j].first;
      }
    }
  }
}

void los_keeper::TargetManager2D::CheckPclCollision(const los_keeper::PclPointCloud &point_cloud) {
  std::vector<LinearConstraint2D> safe_corridor = GenLinearConstraint(point_cloud);
  CalculateSafePclIndex(safe_corridor);
}

bool los_keeper::TargetManager2D::CheckStructuredObstacleCollision(
    const vector<StatePoly> &structured_obstacle_poly_list) {
  primitive_safe_structured_obstacle_index_.clear();
  primitive_safe_structured_obstacle_index_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = primitives_list_[i].size() / param_.sampling.num_thread;
    vector<thread> worker_thread;
    IndexListSet primitive_safe_structured_obstacle_index_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager2D::CheckStructuredObstacleCollisionSubProcess, this,
                                 i, num_chunk * (j), num_chunk * (j + 1),
                                 structured_obstacle_poly_list,
                                 std::ref(primitive_safe_structured_obstacle_index_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < primitive_safe_structured_obstacle_index_temp[j].size(); k++) {
        primitive_safe_structured_obstacle_index_[i].push_back(
            primitive_safe_structured_obstacle_index_temp[j][k]);
      }
    }
  }
  for (int i = 0; i < num_target_; i++) {
    if (primitive_safe_structured_obstacle_index_[i].empty()) {
      return false;
    }
  }
  return true;
}

std::vector<LinearConstraint2D>
los_keeper::TargetManager2D::GenLinearConstraint(const los_keeper::PclPointCloud &point_cloud) {
  Vec2f pcl_points_temp;
  vec_Vec2f obstacle_pcl;
  for (const auto &point : point_cloud.points) {
    pcl_points_temp.coeffRef(0, 0) = point.x;
    pcl_points_temp.coeffRef(1, 0) = point.y;
    obstacle_pcl.push_back(pcl_points_temp);
  }
  polys.clear();
  std::vector<LinearConstraint2D> linear_constraint_temp;
  for (int i = 0; i < num_target_; i++) {
    Vec2f seed = Eigen::Matrix<double, 2, 1>{primitives_list_[i][0].px.GetInitialValue(),
                                             primitives_list_[i][0].py.GetInitialValue()};
    SeedDecomp2D decomp_util(seed);
    decomp_util.set_obs(obstacle_pcl);
    decomp_util.set_local_bbox(
        Vec2f(0.5f * param_.virtual_pcl_bbox.width, 0.5f * param_.virtual_pcl_bbox.width));
    decomp_util.dilate(0.1);
    polys.push_back(decomp_util.get_polyhedron());
    auto poly_hedrons = decomp_util.get_polyhedron();
    LinearConstraint2D safe_corridor_constraint(seed, poly_hedrons.hyperplanes());
    linear_constraint_temp.push_back(safe_corridor_constraint);
  }
  return linear_constraint_temp;
}

void los_keeper::TargetManager2D::CalculateSafePclIndex(
    const std::vector<LinearConstraint2D> &safe_corridor_list) {
  primitive_safe_pcl_index_.clear();
  primitive_safe_pcl_index_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    IndexListSet safe_pcl_index_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager2D::CheckPclCollisionSubProcess, this, i,
                                 safe_corridor_list[i], num_chunk * (j), num_chunk * (j + 1),
                                 std::ref(safe_pcl_index_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < safe_pcl_index_temp[j].size(); k++) {
        primitive_safe_pcl_index_[i].push_back(safe_pcl_index_temp[j][k]);
      }
    }
  }
}

void los_keeper::TargetManager2D::SampleEndPointsSubProcess(
    const int &target_id, const int &chunk_size, const vector<ObjectState> &target_state_list,
    PointList &endpoint_sub) {
  Point end_point_center{float(target_state_list[target_id].px +
                               target_state_list[target_id].vx * param_.horizon.prediction),
                         float(target_state_list[target_id].py +
                               target_state_list[target_id].vy * param_.horizon.prediction),
                         float(target_state_list[target_id].pz +
                               target_state_list[target_id].vz * param_.horizon.prediction)};
  uint n_cols = 2;
  uint n_rows = chunk_size;
  using namespace Eigen;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> gaussian_data_eigen;
  gaussian_data_eigen.setZero(n_rows, n_cols);
  Eigen::Vector2f mean;
  Eigen::Matrix2f covar;
  mean << end_point_center.x, end_point_center.y;
  covar << (0.5f * 0.33333333f * param_.dynamic_limits.acc_max * param_.horizon.prediction *
            param_.horizon.prediction),
      0, 0,
      (0.5f * 0.33333333f * param_.dynamic_limits.acc_max * param_.horizon.prediction *
       param_.horizon.prediction);
  Eigen::EigenMultivariateNormal<float> normX_solver1(mean, covar);
  gaussian_data_eigen << normX_solver1.samples(n_rows).transpose();
  Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
  PointList end_points_temp;
  for (int j = 0; j < n_rows; j++) {
    tempPoint.x = gaussian_data_eigen.coeffRef(j, 0);
    tempPoint.y = gaussian_data_eigen.coeffRef(j, 1);
    tempPoint.z = end_point_center.z;
    endpoint_sub.push_back(tempPoint);
  }
}

void los_keeper::TargetManager2D::ComputePrimitivesSubProcess(
    const int &target_id, const int &start_idx, const int &end_idx,
    const vector<ObjectState> &target_state_list, PrimitiveList &primitive_list_sub) {
  StatePoly primitive_temp;
  primitive_temp.SetDegree(3);
  double time_interval_temp[2]{0.0, param_.horizon.prediction};
  primitive_temp.SetTimeInterval(time_interval_temp);
  BernsteinCoefficients bernstein_coeff_temp(4);

  for (int j = start_idx; j < end_idx; j++) {
    { // x-coefficient
      bernstein_coeff_temp[0] = target_state_list[target_id].px;
      bernstein_coeff_temp[1] =
          target_state_list[target_id].px +
          0.33333333f * target_state_list[target_id].vx * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list[target_id].px + 0.5f * end_points_[target_id][j].x +
          0.16666667f * target_state_list[target_id].vx * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].x;
      primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // y-coefficient
      bernstein_coeff_temp[0] = target_state_list[target_id].py;
      bernstein_coeff_temp[1] =
          target_state_list[target_id].py +
          0.33333333f * target_state_list[target_id].vy * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list[target_id].py + 0.5f * end_points_[target_id][j].y +
          0.16666667f * target_state_list[target_id].vy * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].y;
      primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // z-coefficient
      bernstein_coeff_temp[0] = target_state_list[target_id].pz;
      bernstein_coeff_temp[1] =
          target_state_list[target_id].pz +
          0.33333333f * target_state_list[target_id].vz * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          target_state_list[target_id].pz +
          0.66666667f * target_state_list[target_id].vz * param_.horizon.prediction;
      bernstein_coeff_temp[3] = target_state_list[target_id].pz +
                                target_state_list[target_id].vz * param_.horizon.prediction;
      primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    primitive_temp.rx = target_state_list[target_id].rx;
    primitive_temp.ry = target_state_list[target_id].ry;
    primitive_temp.rz = target_state_list[target_id].rz;
    primitive_list_sub.push_back(primitive_temp);
  }
}

void los_keeper::TargetManager2D::CheckPclCollisionSubProcess(const int &target_id,
                                                              const LinearConstraint2D &constraints,
                                                              const int &start_idx,
                                                              const int &end_idx,
                                                              IndexList &safe_pcl_index_sub) {

  int num_constraints = (int)constraints.A().rows();
  int num_vars = (int)constraints.A().cols(); // 2D
  Eigen::Vector2f A_comp_temp{0.0, 0.0};
  float b_comp_temp{0.0};
  std::vector<Eigen::Vector2f> LinearConstraintA;
  std::vector<float> LinearConstraintB;
  for (int j = 0; j < num_constraints; j++) {
    A_comp_temp[0] = (float)constraints.A().coeffRef(j, 0);
    A_comp_temp[1] = (float)constraints.A().coeffRef(j, 1);
    b_comp_temp = (float)constraints.b().coeffRef(j, 0);
    LinearConstraintA.push_back(A_comp_temp);
    LinearConstraintB.push_back(b_comp_temp);
  }
  float temp_value = 0.0f;
  bool is_safe = true;
  for (int j = start_idx; j < end_idx; j++) {
    is_safe = true;
    for (int k = 0; k < (int)LinearConstraintA.size(); k++) {
      for (int l = 0; l < 4; l++) {
        temp_value = LinearConstraintA[k].coeffRef(0, 0) *
                         primitives_list_[target_id][j].px.GetBernsteinCoefficient()[l] +
                     LinearConstraintA[k].coeffRef(1, 0) *
                         primitives_list_[target_id][j].py.GetBernsteinCoefficient()[l] -
                     LinearConstraintB[k];
        if (temp_value > 0.0f) {
          is_safe = false;
          break;
        }
      }
      if (not is_safe)
        break;
    }
    if (is_safe)
      safe_pcl_index_sub.push_back(j);
  }
}

void los_keeper::TargetManager2D::CheckStructuredObstacleCollisionSubProcess(
    const int &target_id, const int &start_idx, const int &end_idx,
    const vector<StatePoly> &structured_obstacle_poly_list, IndexList &safe_structured_index_sub) {
  bool flag_store_in = true;
  bool flag_store_out = true;
  float value;
  float rx_squared_inverse;
  float ry_squared_inverse;
  float qox[4], qoy[4];
  for (int j = start_idx; j < end_idx; j++) {
    for (int k = 0; k < close_obstacle_index_[target_id].size(); k++) {
      flag_store_out = true;
      rx_squared_inverse =
          1 / powf(structured_obstacle_poly_list[close_obstacle_index_[target_id][k]].rx +
                       primitives_list_[target_id][j].rx,
                   2);
      ry_squared_inverse =
          1 / powf(structured_obstacle_poly_list[close_obstacle_index_[target_id][k]].ry +
                       primitives_list_[target_id][j].ry,
                   2);
      for (int l = 0; l < 4; l++) {
        qox[l] = primitives_list_[target_id][j].px.GetBernsteinCoefficient()[l] -
                 structured_obstacle_poly_list[close_obstacle_index_[target_id][k]]
                     .px.GetBernsteinCoefficient()[l];
        qoy[l] = primitives_list_[target_id][j].py.GetBernsteinCoefficient()[l] -
                 structured_obstacle_poly_list[close_obstacle_index_[target_id][k]]
                     .py.GetBernsteinCoefficient()[l];
      }
      for (int l = 0; l <= 6; l++) {
        flag_store_in = true;
        value = 0.0f;
        for (int m = std::max(0, l - 3); m <= std::min(3, l); m++) {
          value += (float)nchoosek(3, m) * (float)nchoosek(3, l - m) / (float)nchoosek(6, l) *
                   (qox[m] * qox[l - m] * rx_squared_inverse + // x-component
                    qoy[m] * qoy[l - m] * ry_squared_inverse   // y-component
                   );
        }
        if (value < 1.0f) {
          flag_store_in = false;
          break;
        }
      }
      if (not flag_store_in) {
        flag_store_out = false;
        break;
      }
    }
    if (flag_store_in and flag_store_out)
      safe_structured_index_sub.push_back(j);
  }
}

void los_keeper::TargetManager2D::CalculateCentroidSubProcess(const int &target_id,
                                                              const int &start_idx,
                                                              const int &end_idx,
                                                              pair<int, float> &min_dist) {
  int chunk_size = end_idx - start_idx;
  float distance_sum_list[chunk_size];
  for (int i = 0; i < chunk_size; i++) {
    distance_sum_list[i] = 0.0f;
    for (int j = 0; j < primitive_safe_total_index_[target_id].size(); j++) {
      distance_sum_list[i] +=
          powf(primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                       .px.GetTerminalValue() -
                   primitives_list_[target_id][primitive_safe_total_index_[target_id][j]]
                       .px.GetTerminalValue(),
               2) +
          powf(primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                       .py.GetTerminalValue() -
                   primitives_list_[target_id][primitive_safe_total_index_[target_id][j]]
                       .py.GetTerminalValue(),
               2);
    }
  }
  min_dist.second = 999999.0f;
  for (int i = 0; i < chunk_size; i++) {
    if (min_dist.second > distance_sum_list[i]) {
      min_dist.second = distance_sum_list[i];
      min_dist.first = primitive_safe_total_index_[target_id][i + start_idx];
    }
  }
}

std::optional<std::vector<StatePoly>> los_keeper::TargetManager2D::PredictTargetList(
    const vector<ObjectState> &target_state_list, const los_keeper::PclPointCloud &point_cloud,
    const vector<StatePoly> &structured_obstacle_poly_list) {
  num_target_ = target_state_list.size();
  bool prediction_success;
  auto check_prediction_start = std::chrono::system_clock::now();
  auto check_prediction_end = check_prediction_start;
  std::chrono::duration<double> elapsed_check_prediction{};
  SampleEndPoints(target_state_list);
  ComputePrimitives(target_state_list);
  CalculateCloseObstacleIndex(target_state_list, structured_obstacle_poly_list);
  bool is_safe_traj_exist = CheckCollision(point_cloud, structured_obstacle_poly_list);
  if (not is_safe_traj_exist) {
    prediction_success = false;
    goto end_process;
  }
  CalculateCentroid();
  prediction_success = true;
  goto end_process;

end_process : {
  check_prediction_end = std::chrono::system_clock::now();
  elapsed_check_prediction = check_prediction_end - check_prediction_start;
  prediction_time_ = elapsed_check_prediction.count();
};
  if (prediction_success) // target trajectories exist
    return GetTargetPredictionResult();
  else // no target trajectory exists
    return std::nullopt;
}
los_keeper::TargetManager2D::TargetManager2D(const los_keeper::PredictionParameter &param)
    : TargetManager(param) {}

void los_keeper::TargetManager3D::SampleEndPoints(const vector<ObjectState> &target_state_list) {
  end_points_.clear();
  end_points_.resize(target_state_list.size());
  for (int i = 0; i < target_state_list.size(); i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    vector<vector<Point>> end_point_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager3D::SampleEndPointsSubProcess, this, i, num_chunk,
                                 target_state_list, std::ref(end_point_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < end_point_temp[j].size(); k++) {
        end_points_[i].push_back(end_point_temp[j][k]);
      }
    }
  }
}

void los_keeper::TargetManager3D::CalculateCloseObstacleIndex(
    const vector<ObjectState> &target_state_list,
    const vector<StatePoly> &structured_obstacle_poly_list) {
  close_obstacle_index_.clear();
  bool is_close;
  for (int i = 0; i < num_target_; i++) {
    IndexList close_obstacle_index_temp;
    for (int j = 0; j < structured_obstacle_poly_list.size(); j++) {
      is_close =
          powf(target_state_list[i].px - structured_obstacle_poly_list[j].px.GetInitialValue(), 2) +
              powf(target_state_list[i].py - structured_obstacle_poly_list[j].py.GetInitialValue(),
                   2) +
              powf(target_state_list[i].pz - structured_obstacle_poly_list[j].pz.GetInitialValue(),
                   2) <
          param_.distance.obstacle_max * param_.distance.obstacle_max;
      if (is_close)
        close_obstacle_index_temp.push_back(j);
    }
    close_obstacle_index_.push_back(close_obstacle_index_temp);
  }
}

bool los_keeper::TargetManager3D::CheckCollision(
    const los_keeper::PclPointCloud &point_cloud,
    const vector<StatePoly> &structured_obstacle_poly_list) {
  primitive_safe_total_index_.clear();
  bool is_cloud_empty = point_cloud.points.empty();
  bool is_structured_obstacle_empty = structured_obstacle_poly_list.empty();
  if (not is_cloud_empty)
    CheckPclCollision(point_cloud);
  if (not is_structured_obstacle_empty)
    CheckStructuredObstacleCollision(structured_obstacle_poly_list);
  if (is_cloud_empty and is_structured_obstacle_empty) { // Case I: No Obstacle
    for (int i = 0; i < num_target_; i++) {
      IndexList primitive_safe_total_index_temp_;
      for (int j = 0; j < param_.sampling.num_sample; j++) {
        primitive_safe_total_index_temp_.push_back(j);
      }
      primitive_safe_total_index_.push_back(primitive_safe_total_index_temp_);
    }
  }
  if (is_cloud_empty and (not is_structured_obstacle_empty)) {
    primitive_safe_total_index_ = primitive_safe_structured_obstacle_index_;
  }
  if ((not is_cloud_empty) and is_structured_obstacle_empty) {
    primitive_safe_total_index_ = primitive_safe_pcl_index_;
  }
  if (not is_cloud_empty and not is_structured_obstacle_empty) {
    for (int i = 0; i < num_target_; i++) {
      std::vector<bool> is_primitive_safe_pcl_temp;
      std::vector<bool> is_primitive_safe_structured_obstacle_temp;
      IndexList primitive_safe_total_index_temp;
      for (int j = 0; j < param_.sampling.num_sample; j++) {
        is_primitive_safe_pcl_temp.push_back(false);
        is_primitive_safe_structured_obstacle_temp.push_back(false);
      }
      for (int j : primitive_safe_structured_obstacle_index_[i]) {
        is_primitive_safe_structured_obstacle_temp[j] = true;
      }
      for (int j : primitive_safe_pcl_index_[i]) {
        is_primitive_safe_pcl_temp[j] = true;
      }
      for (int j = 0; j < param_.sampling.num_sample; j++) {
        if (is_primitive_safe_pcl_temp[j] and is_primitive_safe_structured_obstacle_temp[j])
          primitive_safe_total_index_temp.push_back(j);
      }
      primitive_safe_total_index_.push_back(primitive_safe_total_index_temp);
    }
  }
  if (primitive_safe_total_index_.empty())
    return false;
  for (int i = 0; i < primitive_safe_total_index_.size(); i++) {
    if (primitive_safe_total_index_[i].empty())
      return false;
  }
  return true;
}

void los_keeper::TargetManager3D::ComputePrimitives(const vector<ObjectState> &target_state_list) {
  primitives_list_.clear();
  primitives_list_.resize(num_target_);

  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    PrimitiveListSet primitive_list_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager3D::ComputePrimitivesSubProcess, this, i,
                                 num_chunk * (j), num_chunk * (j + 1), target_state_list,
                                 std::ref(primitive_list_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < primitive_list_temp[j].size(); k++) {
        primitives_list_[i].push_back(primitive_list_temp[j][k]);
      }
    }
  }
}

void los_keeper::TargetManager3D::CalculateCentroid() {
  primitive_best_index_.clear();
  primitive_best_index_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = primitive_safe_total_index_[i].size() / param_.sampling.num_thread;
    vector<thread> worker_thread;
    vector<std::pair<int, float>> min_dist_pair_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager3D::CalculateCentroidSubProcess, this, i,
                                 num_chunk * (j), num_chunk * (j + 1),
                                 std::ref(min_dist_pair_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    float min_dist_temp = 999999.0f;
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      if (min_dist_temp > min_dist_pair_temp[j].second) {
        min_dist_temp = min_dist_pair_temp[j].second;
        primitive_best_index_[i] = min_dist_pair_temp[j].first;
      }
    }
  }
}

void los_keeper::TargetManager3D::CheckPclCollision(const los_keeper::PclPointCloud &point_cloud) {
  std::vector<LinearConstraint3D> safe_corridor = GenLinearConstraint(point_cloud);
  CalculateSafePclIndex(safe_corridor);
}

bool los_keeper::TargetManager3D::CheckStructuredObstacleCollision(
    const vector<StatePoly> &structured_obstacle_poly_list) {
  primitive_safe_structured_obstacle_index_.clear();
  primitive_safe_structured_obstacle_index_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = primitives_list_[i].size() / param_.sampling.num_thread;
    vector<thread> worker_thread;
    IndexListSet primitive_safe_structured_obstacle_index_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager3D::CheckStructuredObstacleCollisionSubProcess, this,
                                 i, num_chunk * (j), num_chunk * (j + 1),
                                 structured_obstacle_poly_list,
                                 std::ref(primitive_safe_structured_obstacle_index_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < primitive_safe_structured_obstacle_index_temp[j].size(); k++) {
        primitive_safe_structured_obstacle_index_[i].push_back(
            primitive_safe_structured_obstacle_index_temp[j][k]);
      }
    }
  }
  for (int i = 0; i < num_target_; i++) {
    if (primitive_safe_structured_obstacle_index_[i].empty()) {
      return false;
    }
  }
  return true;
}

std::vector<LinearConstraint3D>
los_keeper::TargetManager3D::GenLinearConstraint(const los_keeper::PclPointCloud &point_cloud) {
  Vec3f pcl_points_temp;
  vec_Vec3f obstacle_pcl;
  for (const auto &point : point_cloud.points) {
    pcl_points_temp.coeffRef(0, 0) = point.x;
    pcl_points_temp.coeffRef(1, 0) = point.y;
    pcl_points_temp.coeffRef(2, 0) = point.z;
    obstacle_pcl.push_back(pcl_points_temp);
  }
  polys.clear();
  std::vector<LinearConstraint3D> linear_constraint_temp;
  for (int i = 0; i < num_target_; i++) {
    Vec3f seed = Eigen::Matrix<double, 3, 1>{primitives_list_[i][0].px.GetInitialValue(),
                                             primitives_list_[i][0].py.GetInitialValue(),
                                             primitives_list_[i][0].pz.GetInitialValue()};

    SeedDecomp3D decomp_util(seed);
    decomp_util.set_obs(obstacle_pcl);
    decomp_util.set_local_bbox(Vec3f(0.5f * param_.virtual_pcl_bbox.width,
                                     0.5f * param_.virtual_pcl_bbox.width,
                                     0.5f * param_.virtual_pcl_bbox.height));
    decomp_util.dilate(0.1);
    polys.push_back(decomp_util.get_polyhedron());
    auto poly_hedrons = decomp_util.get_polyhedron();
    LinearConstraint3D safe_corridor_constraint(seed, poly_hedrons.hyperplanes());
    linear_constraint_temp.push_back(safe_corridor_constraint);
  }
  return linear_constraint_temp;
}

void los_keeper::TargetManager3D::CalculateSafePclIndex(
    const std::vector<LinearConstraint3D> &safe_corridor_list) {
  primitive_safe_pcl_index_.clear();
  primitive_safe_pcl_index_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    IndexListSet safe_pcl_index_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager3D::CheckPclCollisionSubProcess, this, i,
                                 safe_corridor_list[i], num_chunk * (j), num_chunk * (j + 1),
                                 std::ref(safe_pcl_index_temp[j]));
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread[j].join();
    }
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      for (int k = 0; k < safe_pcl_index_temp[j].size(); k++) {
        primitive_safe_pcl_index_[i].push_back(safe_pcl_index_temp[j][k]);
      }
    }
  }
}

void los_keeper::TargetManager3D::SampleEndPointsSubProcess(
    const int &target_id, const int &chunk_size, const vector<ObjectState> &target_state_list,
    PointList &endpoint_sub) {
  Point end_point_center{float(target_state_list[target_id].px +
                               target_state_list[target_id].vx * param_.horizon.prediction),
                         float(target_state_list[target_id].py +
                               target_state_list[target_id].vy * param_.horizon.prediction),
                         float(target_state_list[target_id].pz +
                               target_state_list[target_id].vz * param_.horizon.prediction)};
  uint n_cols = 3;
  uint n_rows = chunk_size;
  using namespace Eigen;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> gaussian_data_eigen;
  gaussian_data_eigen.setZero(n_rows, n_cols);
  Eigen::Vector3f mean;
  Eigen::Matrix3f covar;
  mean << end_point_center.x, end_point_center.y, end_point_center.z;
  covar << (0.5f * 0.33333333f * param_.dynamic_limits.acc_max * param_.horizon.prediction *
            param_.horizon.prediction),
      0, 0, 0,
      (0.5f * 0.33333333f * param_.dynamic_limits.acc_max * param_.horizon.prediction *
       param_.horizon.prediction),
      0, 0, 0,
      (0.5f * 0.33333333f * param_.dynamic_limits.acc_max * param_.horizon.prediction *
       param_.horizon.prediction);
  Eigen::EigenMultivariateNormal<float> normX_solver1(mean, covar);
  gaussian_data_eigen << normX_solver1.samples(n_rows).transpose();

  Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
  for (int j = 0; j < n_rows; j++) {
    tempPoint.x = gaussian_data_eigen.coeffRef(j, 0);
    tempPoint.y = gaussian_data_eigen.coeffRef(j, 1);
    tempPoint.z = gaussian_data_eigen.coeffRef(j, 2);
    endpoint_sub.push_back(tempPoint);
  }
}

void los_keeper::TargetManager3D::ComputePrimitivesSubProcess(
    const int &target_id, const int &start_idx, const int &end_idx,
    const vector<ObjectState> &target_state_list, PrimitiveList &primitive_list_sub) {
  StatePoly primitive_temp;
  primitive_temp.SetDegree(3);
  double time_interval_temp[2]{0.0, param_.horizon.prediction};
  primitive_temp.SetTimeInterval(time_interval_temp);
  BernsteinCoefficients bernstein_coeff_temp(4);
  for (int j = start_idx; j < end_idx; j++) {
    { // x-coefficient
      bernstein_coeff_temp[0] = target_state_list[target_id].px;
      bernstein_coeff_temp[1] =
          target_state_list[target_id].px +
          0.33333333f * target_state_list[target_id].vx * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list[target_id].px + 0.5f * end_points_[target_id][j].x +
          0.16666667f * target_state_list[target_id].vx * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].x;
      primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // y-coefficient
      bernstein_coeff_temp[0] = target_state_list[target_id].py;
      bernstein_coeff_temp[1] =
          target_state_list[target_id].py +
          0.33333333f * target_state_list[target_id].vy * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list[target_id].py + 0.5f * end_points_[target_id][j].y +
          0.16666667f * target_state_list[target_id].vy * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].y;
      primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // z-coefficient
      bernstein_coeff_temp[0] = target_state_list[target_id].pz;
      bernstein_coeff_temp[1] =
          target_state_list[target_id].pz +
          0.33333333f * target_state_list[target_id].vz * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list[target_id].pz + 0.5f * end_points_[target_id][j].z +
          0.16666667f * target_state_list[target_id].vz * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].z;
      primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    primitive_list_sub.push_back(primitive_temp);
  }
}

void los_keeper::TargetManager3D::CheckPclCollisionSubProcess(const int &target_id,
                                                              const LinearConstraint3D &constraints,
                                                              const int &start_idx,
                                                              const int &end_idx,
                                                              IndexList &safe_pcl_index_sub) {
  int num_constraints = (int)constraints.A().rows();
  int num_vars = (int)constraints.A().cols(); // 3D
  IndexList safe_pcl_index_temp_;
  Eigen::Vector3f A_comp_temp{0.0, 0.0, 0.0};
  float b_comp_temp{0.0};
  std::vector<Eigen::Vector3f> LinearConstraintA;
  std::vector<float> LinearConstraintB;
  for (int j = 0; j < num_constraints; j++) {
    A_comp_temp[0] = (float)constraints.A().coeffRef(j, 0);
    A_comp_temp[1] = (float)constraints.A().coeffRef(j, 1);
    A_comp_temp[2] = (float)constraints.A().coeffRef(j, 2);
    b_comp_temp = (float)constraints.b().coeffRef(j, 0);
    LinearConstraintA.push_back(A_comp_temp);
    LinearConstraintB.push_back(b_comp_temp);
  }
  float temp_value = 0.0f;
  bool is_safe = true;
  for (int j = start_idx; j < end_idx; j++) {
    is_safe = true;
    for (int k = 0; k < (int)LinearConstraintA.size(); k++) {
      for (int l = 0; l < 4; l++) {
        temp_value = LinearConstraintA[k].coeffRef(0, 0) *
                         primitives_list_[target_id][j].px.GetBernsteinCoefficient()[l] +
                     LinearConstraintA[k].coeffRef(1, 0) *
                         primitives_list_[target_id][j].py.GetBernsteinCoefficient()[l] +
                     LinearConstraintA[k].coeffRef(1, 0) *
                         primitives_list_[target_id][j].pz.GetBernsteinCoefficient()[l] -
                     LinearConstraintB[k];
        if (temp_value > 0.0f) {
          is_safe = false;
          break;
        }
      }
      if (not is_safe)
        break;
    }
    if (is_safe)
      safe_pcl_index_sub.push_back(j);
  }
}

void los_keeper::TargetManager3D::CheckStructuredObstacleCollisionSubProcess(
    const int &target_id, const int &start_idx, const int &end_idx,
    const vector<StatePoly> &structured_obstacle_poly_list, IndexList &safe_structured_index_sub) {

  bool flag_store_in = true;
  bool flag_store_out = true;
  float value;
  float qox[4], qoy[4], qoz[4];
  float rx_squared_inverse;
  float ry_squared_inverse;
  float rz_squared_inverse;
  for (int j = start_idx; j < end_idx; j++) {
    for (int k = 0; k < close_obstacle_index_[target_id].size(); k++) {
      flag_store_out = true;
      rx_squared_inverse =
          1 / powf(structured_obstacle_poly_list[close_obstacle_index_[target_id][k]].rx +
                       primitives_list_[target_id][j].rx,
                   2);
      ry_squared_inverse =
          1 / powf(structured_obstacle_poly_list[close_obstacle_index_[target_id][k]].ry +
                       primitives_list_[target_id][j].ry,
                   2);
      rz_squared_inverse =
          1 / powf(structured_obstacle_poly_list[close_obstacle_index_[target_id][k]].rz +
                       primitives_list_[target_id][j].rz,
                   2);
      for (int l = 0; l < 4; l++) {
        qox[l] = primitives_list_[target_id][j].px.GetBernsteinCoefficient()[l] -
                 structured_obstacle_poly_list[close_obstacle_index_[target_id][k]]
                     .px.GetBernsteinCoefficient()[l];
        qoy[l] = primitives_list_[target_id][j].py.GetBernsteinCoefficient()[l] -
                 structured_obstacle_poly_list[close_obstacle_index_[target_id][k]]
                     .py.GetBernsteinCoefficient()[l];
        qoz[l] = primitives_list_[target_id][j].pz.GetBernsteinCoefficient()[l] -
                 structured_obstacle_poly_list[close_obstacle_index_[target_id][k]]
                     .pz.GetBernsteinCoefficient()[l];
      }
      for (int l = 0; l <= 2 * 3; l++) {
        flag_store_in = true;
        value = 0.0f;
        for (int m = std::max(0, l - 3); m <= std::min(3, l); m++) {
          value += (float)nchoosek(3, m) * (float)nchoosek(3, l - m) / (float)nchoosek(2 * 3, l) *
                   (qox[m] * qoy[l - m] * rx_squared_inverse + // x-component
                    qoy[m] * qoy[l - m] * ry_squared_inverse + // y-component
                    qoz[m] * qoz[l - m] * rz_squared_inverse   // z-component
                   );
        }
        if (value < 1.0f) {
          flag_store_in = false;
          break;
        }
      }
      if (not flag_store_in) {
        flag_store_out = false;
        break;
      }
    }
    if (flag_store_in and flag_store_out)
      safe_structured_index_sub.push_back(j);
  }
}

void los_keeper::TargetManager3D::CalculateCentroidSubProcess(const int &target_id,
                                                              const int &start_idx,
                                                              const int &end_idx,
                                                              pair<int, float> &min_dist) {
  int chunk_size = end_idx - start_idx;
  float distance_sum_list[chunk_size];
  for (int i = 0; i < chunk_size; i++) {
    distance_sum_list[i] = 0.0f;
    for (int j = 0; j < primitive_safe_total_index_[target_id].size(); j++) {
      distance_sum_list[i] +=
          powf(primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                       .px.GetTerminalValue() -
                   primitives_list_[target_id][primitive_safe_total_index_[target_id][j]]
                       .px.GetTerminalValue(),
               2) +
          powf(primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                       .py.GetTerminalValue() -
                   primitives_list_[target_id][primitive_safe_total_index_[target_id][j]]
                       .py.GetTerminalValue(),
               2) +
          powf(primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                       .pz.GetTerminalValue() -
                   primitives_list_[target_id][primitive_safe_total_index_[target_id][j]]
                       .pz.GetTerminalValue(),
               2);
    }
  }
  min_dist.second = 999999.0f;
  for (int i = 0; i < chunk_size; i++) {
    if (min_dist.second > distance_sum_list[i]) {
      min_dist.second = distance_sum_list[i];
      min_dist.first = primitive_safe_total_index_[target_id][i + start_idx];
    }
  }
}

std::optional<std::vector<StatePoly>> los_keeper::TargetManager3D::PredictTargetList(
    const vector<ObjectState> &target_state_list, const los_keeper::PclPointCloud &point_cloud,
    const vector<StatePoly> &structured_obstacle_poly_list) {
  num_target_ = target_state_list.size();
  bool prediction_success;
  auto check_prediction_start = std::chrono::system_clock::now();
  auto check_prediction_end = check_prediction_start;
  std::chrono::duration<double> elapsed_check_prediction{};
  SampleEndPoints(target_state_list);
  ComputePrimitives(target_state_list);
  CalculateCloseObstacleIndex(target_state_list, structured_obstacle_poly_list);
  bool is_safe_traj_exist = CheckCollision(point_cloud, structured_obstacle_poly_list);
  if (not is_safe_traj_exist) {
    prediction_success = false;
    goto end_process;
  } else {
    CalculateCentroid();
    prediction_success = true;
    goto end_process;
  }
end_process : {
  check_prediction_end = std::chrono::system_clock::now();
  elapsed_check_prediction = check_prediction_end - check_prediction_start;
  prediction_time_ = elapsed_check_prediction.count();
};
  if (prediction_success) // target trajectories exist
    return GetTargetPredictionResult();
  else // no target trajectory exists
    return std::nullopt;
}
los_keeper::TargetManager3D::TargetManager3D(const los_keeper::PredictionParameter &param)
    : TargetManager(param) {}
