#include "los_keeper/target_manager/target_manager.h"

using namespace std;

void los_keeper::TargetManager::SetTargetState(const std::vector<ObjectState> &target_state_list) {
  target_state_list_.clear();
  target_state_list_ = target_state_list;
  num_target_ = (int)target_state_list_.size();
}

void los_keeper::TargetManager::SetObstacleState(
    pcl::PointCloud<pcl::PointXYZ> cloud, const PrimitiveList &structured_obstacle_poly_list) {

  structured_obstacle_poly_list_.clear();
  structured_obstacle_poly_list_ = structured_obstacle_poly_list;
  cloud_.points.clear();
  cloud_.points = cloud.points;
}

bool los_keeper::TargetManager::PredictTargetTrajectory() {
  cout << "Default Target Manager" << endl;
  SampleEndPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  CheckCollision();
  CalculateCentroid();
  return true;
}

void los_keeper::TargetManager::SampleEndPoints() {}

los_keeper::TargetManager::TargetManager() { // Abstract Target Manager
}

void los_keeper::TargetManager::ComputePrimitives() {}

void los_keeper::TargetManager::CalculateCloseObstacleIndex() {}

void los_keeper::TargetManager::CalculateCentroid() {}

void los_keeper::TargetManager::CheckPclCollision() {}

void los_keeper::TargetManager::CheckStructuredObstacleCollision() {}

void los_keeper::TargetManager::SampleEndPointsSubProcess(const int &target_id,
                                                          const int &chunk_size,
                                                          PointList &endpoint_sub) {}

void los_keeper::TargetManager::ComputePrimitivesSubProcess(const int &target_id,
                                                            const int &start_idx,
                                                            const int &end_idx,
                                                            PrimitiveList &primitive_list_sub) {}

void los_keeper::TargetManager::CheckStructuredObstacleCollisionSubProcess(
    const int &target_id, const int &start_idx, const int &end_idx,
    IndexList &safe_structured_index_sub) {}

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

bool los_keeper::TargetManager2D::PredictTargetTrajectory() {
  SampleEndPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  bool is_safe_traj_exist = CheckCollision();
  if (is_safe_traj_exist)
    CalculateCentroid();
  return is_safe_traj_exist;
}

void los_keeper::TargetManager2D::SampleEndPoints() {
  end_points_.clear();
  end_points_.resize(target_state_list_.size()); //
  cout << "target_state_list_size: " << end_points_.size() << endl;
  for (int i = 0; i < target_state_list_.size(); i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    vector<vector<Point>> end_point_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager2D::SampleEndPointsSubProcess, this, i, num_chunk,
                                 std::ref(end_point_temp[j]));
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

void los_keeper::TargetManager2D::ComputePrimitives() {
  primitives_list_.clear();
  primitives_list_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    PrimitiveListSet primitive_list_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager2D::ComputePrimitivesSubProcess, this, i,
                                 num_chunk * (j), num_chunk * (j + 1),
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

void los_keeper::TargetManager2D::CalculateCloseObstacleIndex() {
  close_obstacle_index_.clear();
  bool is_close;
  for (int i = 0; i < num_target_; i++) {
    IndexList close_obstacle_index_temp;
    for (int j = 0; j < structured_obstacle_poly_list_.size(); j++) {
      is_close =
          powf(target_state_list_[i].px - structured_obstacle_poly_list_[j].px.GetInitialValue(),
               2) +
              powf(target_state_list_[i].py -
                       structured_obstacle_poly_list_[j].py.GetInitialValue(),
                   2) <
          param_.distance.obstacle_max * param_.distance.obstacle_max;
      if (is_close)
        close_obstacle_index_temp.push_back(j);
    }
    close_obstacle_index_.push_back(close_obstacle_index_temp);
  }
}

bool los_keeper::TargetManager2D::CheckCollision() {
  primitive_safe_total_index_.clear();
  bool is_cloud_empty = cloud_.points.empty();
  bool is_structured_obstacle_empty = structured_obstacle_poly_list_.empty();
  if (not is_cloud_empty) {
    CheckPclCollision();
  }
  if (not is_structured_obstacle_empty) {
    CheckStructuredObstacleCollision();
  }
  if (is_cloud_empty and is_structured_obstacle_empty) { // Case I: No Obstacle
    for (int i = 0; i < num_target_; i++) {
      IndexList primitive_safe_total_index_temp_;
      for (int j = 0; j < param_.sampling.num_sample; j++) {
        primitive_safe_total_index_temp_.push_back(j);
      }
      primitive_safe_total_index_.push_back(primitive_safe_total_index_temp_);
    }
  }
  if (is_cloud_empty and (not is_structured_obstacle_empty)) { // Case II: Only Ellipsoidal Obstacle
    primitive_safe_total_index_ = primitive_safe_structured_obstacle_index_;
  }
  if ((not is_cloud_empty) and
      is_structured_obstacle_empty) { // Case III: Only Unstructured Obstacle
    primitive_safe_total_index_ = primitive_safe_pcl_index_;
  }
  if (not is_cloud_empty and not is_structured_obstacle_empty) { // Case IV: Ellipsoidal and
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
  return not primitive_safe_total_index_.empty();
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
    float min_dist_temp = 99999999999999.0f;
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      if (min_dist_temp > min_dist_pair_temp[j].second) {
        min_dist_temp = min_dist_pair_temp[j].second;
        primitive_best_index_[i] = min_dist_pair_temp[j].first;
      }
    }
  }
}

void los_keeper::TargetManager2D::CheckPclCollision() {
  std::vector<LinearConstraint2D> safe_corridor = GenLinearConstraint();
  CalculateSafePclIndex(safe_corridor);
}

void los_keeper::TargetManager2D::CheckStructuredObstacleCollision() {
  primitive_safe_structured_obstacle_index_.clear();
  primitive_safe_structured_obstacle_index_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = (int)primitives_list_[i].size() / param_.sampling.num_thread;
    vector<thread> worker_thread;
    IndexListSet primitive_safe_structured_obstacle_index_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager2D::CheckStructuredObstacleCollisionSubProcess, this,
                                 i, num_chunk * (j), num_chunk * (j + 1),
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
}

std::vector<LinearConstraint2D> los_keeper::TargetManager2D::GenLinearConstraint() {
  Vec2f pcl_points_temp;
  vec_Vec2f obstacle_pcl;
  for (const auto &point : cloud_.points) {
    pcl_points_temp.coeffRef(0, 0) = point.x;
    pcl_points_temp.coeffRef(1, 0) = point.y;
    obstacle_pcl.push_back(pcl_points_temp);
  }
  polys.clear();
  std::vector<LinearConstraint2D> linear_constraint_temp;
  for (int i = 0; i < num_target_; i++) {
    Vec2f seed = Eigen::Matrix<double, 2, 1>{target_state_list_[i].px, target_state_list_[i].py};
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

void los_keeper::TargetManager2D::SampleEndPointsSubProcess(const int &target_id,
                                                            const int &chunk_size,
                                                            PointList &endpoint_sub) {
  Point end_point_center{float(target_state_list_[target_id].px +
                               target_state_list_[target_id].vx * param_.horizon.prediction),
                         float(target_state_list_[target_id].py +
                               target_state_list_[target_id].vy * param_.horizon.prediction),
                         float(target_state_list_[target_id].pz +
                               target_state_list_[target_id].vz * param_.horizon.prediction)};
  uint n_cols = 2;
  uint n_rows = param_.sampling.num_sample;
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
  Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
  PointList end_points_temp;
  for (int j = 0; j < n_rows; j++) {
    tempPoint.x = gaussian_data_eigen.coeffRef(j, 0);
    tempPoint.y = gaussian_data_eigen.coeffRef(j, 1);
    tempPoint.z = end_point_center.z;
    endpoint_sub.push_back(tempPoint);
  }
}

void los_keeper::TargetManager2D::ComputePrimitivesSubProcess(const int &target_id,
                                                              const int &start_idx,
                                                              const int &end_idx,
                                                              PrimitiveList &primitive_list_sub) {

  StatePoly primitive_temp;
  primitive_temp.SetDegree(3);
  float time_interval_temp[2]{0.0, param_.horizon.prediction};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[4];

  for (int j = start_idx; j < end_idx; j++) {
    { // x-coefficient
      bernstein_coeff_temp[0] = target_state_list_[target_id].px;
      bernstein_coeff_temp[1] =
          target_state_list_[target_id].px +
          0.33333333f * target_state_list_[target_id].vx * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list_[target_id].px + 0.5f * end_points_[target_id][j].x +
          0.16666667f * target_state_list_[target_id].vx * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].x;
      primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // y-coefficient
      bernstein_coeff_temp[0] = target_state_list_[target_id].py;
      bernstein_coeff_temp[1] =
          target_state_list_[target_id].py +
          0.33333333f * target_state_list_[target_id].vy * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list_[target_id].py + 0.5f * end_points_[target_id][j].y +
          0.16666667f * target_state_list_[target_id].vy * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].y;
      primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // z-coefficient
      bernstein_coeff_temp[0] = target_state_list_[target_id].pz;
      bernstein_coeff_temp[1] =
          target_state_list_[target_id].pz +
          0.33333333f * target_state_list_[target_id].vz * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          target_state_list_[target_id].pz +
          0.66666667f * target_state_list_[target_id].vz * param_.horizon.prediction;
      bernstein_coeff_temp[3] = target_state_list_[target_id].pz +
                                target_state_list_[target_id].vz * param_.horizon.prediction;
      primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
    }
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
    IndexList &safe_structured_index_sub) {
  bool flag_store_in = true;
  bool flag_store_out = true;
  float value;
  for (int j = start_idx; j < end_idx; j++) {
    for (int k = 0; k < close_obstacle_index_[target_id].size(); k++) {
      flag_store_out = true;
      for (int l = 0; l <= 2 * 3; l++) {
        flag_store_in = true;
        value = 0.0f;
        for (int m = std::max(0, l - 3); m <= std::min(3, l); m++) {
          value += (float)nchoosek(3, m) * (float)nchoosek(3, l - m) / (float)nchoosek(2 * 3, l) *
                   (primitives_list_[target_id][j].px.GetBernsteinCoefficient()[m] *
                        primitives_list_[target_id][j].px.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].px.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .px.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].px.GetBernsteinCoefficient()[l - m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .px.GetBernsteinCoefficient()[m] +
                    structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .px.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .px.GetBernsteinCoefficient()[l - m] + // x-component
                    primitives_list_[target_id][j].py.GetBernsteinCoefficient()[m] *
                        primitives_list_[target_id][j].py.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].py.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .py.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].py.GetBernsteinCoefficient()[l - m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .py.GetBernsteinCoefficient()[m] +
                    structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .py.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .py.GetBernsteinCoefficient()[l - m] // y-component
                   );
        }
        if (value <
            powf(structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]].rx +
                     primitives_list_[target_id][j].rx,
                 2) +
                powf(structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]].ry +
                         primitives_list_[target_id][j].ry,
                     2)) {
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
          (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
               .px.GetTerminalValue() -
           primitives_list_[target_id]
                           [primitive_safe_total_index_
                                [target_id]
                                [primitive_safe_total_index_
                                     [target_id][primitive_safe_total_index_[target_id][j]]]]
                               .px.GetTerminalValue()) *
              (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                   .px.GetTerminalValue() -
               primitives_list_[target_id]
                               [primitive_safe_total_index_
                                    [target_id]
                                    [primitive_safe_total_index_
                                         [target_id][primitive_safe_total_index_[target_id][j]]]]
                                   .px.GetTerminalValue()) +
          (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
               .py.GetTerminalValue() -
           primitives_list_[target_id]
                           [primitive_safe_total_index_
                                [target_id]
                                [primitive_safe_total_index_
                                     [target_id][primitive_safe_total_index_[target_id][j]]]]
                               .py.GetTerminalValue()) *
              (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                   .py.GetTerminalValue() -
               primitives_list_[target_id]
                               [primitive_safe_total_index_
                                    [target_id]
                                    [primitive_safe_total_index_
                                         [target_id][primitive_safe_total_index_[target_id][j]]]]
                                   .py.GetTerminalValue());
    }
  }
  min_dist.second = 99999999999.0f;
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
  this->SetTargetState(target_state_list);
  this->SetObstacleState(point_cloud, structured_obstacle_poly_list);
  bool is_target_trajectory_exist = PredictTargetTrajectory();
  if (is_target_trajectory_exist) // target trajectories exist
    return GetTargetPredictionResult();
  else // no target trajectory exists
    return std::nullopt;
}
los_keeper::TargetManager2D::TargetManager2D(const los_keeper::PredictionParameter &param)
    : TargetManager(param) {}

bool los_keeper::TargetManager3D::PredictTargetTrajectory() {
  cout << "3D Prediction" << endl;
  /** TODO(Lee): fix bug
  SampleEndPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  bool is_safe_traj_exist = CheckCollision();
  if (is_safe_traj_exist)
    CalculateCentroid();
  return is_safe_traj_exist;
  **/
  return true;
}

void los_keeper::TargetManager3D::SampleEndPoints() {
  end_points_.clear();
  end_points_.resize(target_state_list_.size());
  for (int i = 0; i < target_state_list_.size(); i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    vector<vector<Point>> end_point_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager3D::SampleEndPointsSubProcess, this, i, num_chunk,
                                 std::ref(end_point_temp[j]));
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

void los_keeper::TargetManager3D::CalculateCloseObstacleIndex() {
  close_obstacle_index_.clear();
  bool is_close;
  for (int i = 0; i < num_target_; i++) {
    IndexList close_obstacle_index_temp;
    for (int j = 0; j < structured_obstacle_poly_list_.size(); j++) {
      is_close =
          powf(target_state_list_[i].px - structured_obstacle_poly_list_[j].px.GetInitialValue(),
               2) +
              powf(target_state_list_[i].py -
                       structured_obstacle_poly_list_[j].py.GetInitialValue(),
                   2) +
              powf(target_state_list_[i].pz -
                       structured_obstacle_poly_list_[j].pz.GetInitialValue(),
                   2) <
          param_.distance.obstacle_max * param_.distance.obstacle_max;
      if (is_close)
        close_obstacle_index_temp.push_back(j);
    }
    close_obstacle_index_.push_back(close_obstacle_index_temp);
  }
}

bool los_keeper::TargetManager3D::CheckCollision() {
  primitive_safe_total_index_.clear();
  bool is_cloud_empty = cloud_.points.empty();
  bool is_structured_obstacle_empty = structured_obstacle_poly_list_.empty();

  if (not is_cloud_empty) {
    CheckPclCollision();
  }
  if (not is_structured_obstacle_empty) {
    CheckStructuredObstacleCollision();
  }
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
  return not primitive_safe_total_index_.empty();
}

void los_keeper::TargetManager3D::ComputePrimitives() {
  primitives_list_.clear();
  primitives_list_.resize(num_target_);

  for (int i = 0; i < num_target_; i++) {
    int num_chunk = param_.sampling.num_sample / param_.sampling.num_thread;
    vector<thread> worker_thread;
    PrimitiveListSet primitive_list_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager3D::ComputePrimitivesSubProcess, this, i,
                                 num_chunk * (j), num_chunk * (j + 1),
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
    int num_chunk = (int)primitive_safe_total_index_[i].size() / param_.sampling.num_thread;
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
    float min_dist_temp = 99999999999999.0f;
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      if (min_dist_temp > min_dist_pair_temp[j].second) {
        min_dist_temp = min_dist_pair_temp[j].second;
        primitive_best_index_[i] = min_dist_pair_temp[j].first;
      }
    }
  }
}

void los_keeper::TargetManager3D::CheckPclCollision() {
  std::vector<LinearConstraint3D> safe_corridor = GenLinearConstraint();
  CalculateSafePclIndex(safe_corridor);
}

void los_keeper::TargetManager3D::CheckStructuredObstacleCollision() {
  primitive_safe_structured_obstacle_index_.clear();
  primitive_safe_structured_obstacle_index_.resize(num_target_);
  for (int i = 0; i < num_target_; i++) {
    int num_chunk = (int)primitives_list_[i].size() / param_.sampling.num_thread;
    vector<thread> worker_thread;
    IndexListSet primitive_safe_structured_obstacle_index_temp(param_.sampling.num_thread);
    for (int j = 0; j < param_.sampling.num_thread; j++) {
      worker_thread.emplace_back(&TargetManager3D::CheckStructuredObstacleCollisionSubProcess, this,
                                 i, num_chunk * (j), num_chunk * (j + 1),
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
}

std::vector<LinearConstraint3D> los_keeper::TargetManager3D::GenLinearConstraint() {
  Vec3f pcl_points_temp;
  vec_Vec3f obstacle_pcl;
  for (const auto &point : cloud_.points) {
    pcl_points_temp.coeffRef(0, 0) = point.x;
    pcl_points_temp.coeffRef(1, 0) = point.y;
    pcl_points_temp.coeffRef(2, 0) = point.z;
    obstacle_pcl.push_back(pcl_points_temp);
  }
  polys.clear();
  std::vector<LinearConstraint3D> linear_constraint_temp;
  for (int i = 0; i < num_target_; i++) {
    Vec3f seed = Eigen::Matrix<double, 3, 1>{target_state_list_[i].px, target_state_list_[i].py,
                                             target_state_list_[i].pz};
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

void los_keeper::TargetManager3D::SampleEndPointsSubProcess(const int &target_id,
                                                            const int &chunk_size,
                                                            PointList &endpoint_sub) {
  Point end_point_center{float(target_state_list_[target_id].px +
                               target_state_list_[target_id].vx * param_.horizon.prediction),
                         float(target_state_list_[target_id].py +
                               target_state_list_[target_id].vy * param_.horizon.prediction),
                         float(target_state_list_[target_id].pz +
                               target_state_list_[target_id].vz * param_.horizon.prediction)};
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

void los_keeper::TargetManager3D::ComputePrimitivesSubProcess(const int &target_id,
                                                              const int &start_idx,
                                                              const int &end_idx,
                                                              PrimitiveList &primitive_list_sub) {

  StatePoly primitive_temp;
  primitive_temp.SetDegree(3);
  float time_interval_temp[2]{0.0, param_.horizon.prediction};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[4];
  for (int j = start_idx; j < end_idx; j++) {
    { // x-coefficient
      bernstein_coeff_temp[0] = target_state_list_[target_id].px;
      bernstein_coeff_temp[1] =
          target_state_list_[target_id].px +
          0.33333333f * target_state_list_[target_id].vx * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list_[target_id].px + 0.5f * end_points_[target_id][j].x +
          0.16666667f * target_state_list_[target_id].vx * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].x;
      primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // y-coefficient
      bernstein_coeff_temp[0] = target_state_list_[target_id].py;
      bernstein_coeff_temp[1] =
          target_state_list_[target_id].py +
          0.33333333f * target_state_list_[target_id].vy * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list_[target_id].py + 0.5f * end_points_[target_id][j].y +
          0.16666667f * target_state_list_[target_id].vy * param_.horizon.prediction;
      bernstein_coeff_temp[3] = end_points_[target_id][j].y;
      primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // z-coefficient
      bernstein_coeff_temp[0] = target_state_list_[target_id].pz;
      bernstein_coeff_temp[1] =
          target_state_list_[target_id].pz +
          0.33333333f * target_state_list_[target_id].vz * param_.horizon.prediction;
      bernstein_coeff_temp[2] =
          0.5f * target_state_list_[target_id].pz + 0.5f * end_points_[target_id][j].z +
          0.16666667f * target_state_list_[target_id].vz * param_.horizon.prediction;
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
    IndexList &safe_structured_index_sub) {

  bool flag_store_in = true;
  bool flag_store_out = true;
  float value;
  for (int j = start_idx; j < end_idx; j++) {
    for (int k = 0; k < close_obstacle_index_[target_id].size(); k++) {
      flag_store_out = true;
      for (int l = 0; l <= 2 * 3; l++) {
        flag_store_in = true;
        value = 0.0f;
        for (int m = std::max(0, l - 3); m <= std::min(3, l); m++) {
          value += (float)nchoosek(3, m) * (float)nchoosek(3, l - m) / (float)nchoosek(2 * 3, l) *
                   (primitives_list_[target_id][j].px.GetBernsteinCoefficient()[m] *
                        primitives_list_[target_id][j].px.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].px.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .px.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].px.GetBernsteinCoefficient()[l - m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .px.GetBernsteinCoefficient()[m] +
                    structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .px.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .px.GetBernsteinCoefficient()[l - m] + // x-component
                    primitives_list_[target_id][j].py.GetBernsteinCoefficient()[m] *
                        primitives_list_[target_id][j].py.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].py.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .py.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].py.GetBernsteinCoefficient()[l - m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .py.GetBernsteinCoefficient()[m] +
                    structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .py.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .py.GetBernsteinCoefficient()[l - m] + // y-component
                    primitives_list_[target_id][j].pz.GetBernsteinCoefficient()[m] *
                        primitives_list_[target_id][j].pz.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].pz.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .pz.GetBernsteinCoefficient()[l - m] -
                    primitives_list_[target_id][j].pz.GetBernsteinCoefficient()[l - m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .pz.GetBernsteinCoefficient()[m] +
                    structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .pz.GetBernsteinCoefficient()[m] *
                        structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]]
                            .pz.GetBernsteinCoefficient()[l - m] // y-component
                   );
        }
        if (value <
            powf(structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]].rx +
                     primitives_list_[target_id][j].rx,
                 2) +
                powf(structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]].ry +
                         primitives_list_[target_id][j].ry,
                     2) +
                powf(structured_obstacle_poly_list_[close_obstacle_index_[target_id][k]].rz +
                         primitives_list_[target_id][j].rz,
                     2)) {
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
          (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
               .px.GetTerminalValue() -
           primitives_list_[target_id]
                           [primitive_safe_total_index_
                                [target_id]
                                [primitive_safe_total_index_
                                     [target_id][primitive_safe_total_index_[target_id][j]]]]
                               .px.GetTerminalValue()) *
              (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                   .px.GetTerminalValue() -
               primitives_list_[target_id]
                               [primitive_safe_total_index_
                                    [target_id]
                                    [primitive_safe_total_index_
                                         [target_id][primitive_safe_total_index_[target_id][j]]]]
                                   .px.GetTerminalValue()) +
          (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
               .py.GetTerminalValue() -
           primitives_list_[target_id]
                           [primitive_safe_total_index_
                                [target_id]
                                [primitive_safe_total_index_
                                     [target_id][primitive_safe_total_index_[target_id][j]]]]
                               .py.GetTerminalValue()) *
              (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                   .py.GetTerminalValue() -
               primitives_list_[target_id]
                               [primitive_safe_total_index_
                                    [target_id]
                                    [primitive_safe_total_index_
                                         [target_id][primitive_safe_total_index_[target_id][j]]]]
                                   .py.GetTerminalValue()) +
          (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
               .pz.GetTerminalValue() -
           primitives_list_[target_id]
                           [primitive_safe_total_index_
                                [target_id]
                                [primitive_safe_total_index_
                                     [target_id][primitive_safe_total_index_[target_id][j]]]]
                               .pz.GetTerminalValue()) *
              (primitives_list_[target_id][primitive_safe_total_index_[target_id][i + start_idx]]
                   .pz.GetTerminalValue() -
               primitives_list_[target_id]
                               [primitive_safe_total_index_
                                    [target_id]
                                    [primitive_safe_total_index_
                                         [target_id][primitive_safe_total_index_[target_id][j]]]]
                                   .pz.GetTerminalValue());
    }
  }
  min_dist.second = 99999999999.0f;
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
  this->SetTargetState(target_state_list);
  this->SetObstacleState(point_cloud, structured_obstacle_poly_list);
  //  return std::vector<StatePoly>(3);
  bool is_target_trajectory_exist = PredictTargetTrajectory();
  if (is_target_trajectory_exist) // target trajectories exist
    return GetTargetPredictionResult();
  else // no target trajectory exists
    return std::nullopt;
}
los_keeper::TargetManager3D::TargetManager3D(const los_keeper::PredictionParameter &param)
    : TargetManager(param) {}
