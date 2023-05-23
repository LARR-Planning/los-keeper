#include "los_keeper/target_manager/target_manager.h"
using namespace los_keeper;

void los_keeper::TargetManager::SetTargetState(
    const std::vector<ObjectState> &target_state_list) {
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
  for (int i = 0; i < target_state_list_.size(); i++) {
    Point end_point_center{float(target_state_list_[i].px +
                                 target_state_list_[i].vx * planning_horizon_),
                           float(target_state_list_[i].py +
                                 target_state_list_[i].vy * planning_horizon_),
                           float(target_state_list_[i].pz +
                                 target_state_list_[i].vz * planning_horizon_)};
    uint n_cols = 2;
    uint n_rows = num_sample_;
    using namespace Eigen;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> gaussian_data_eigen;
    gaussian_data_eigen.setZero(n_rows, n_cols);
    Eigen::Vector2f mean;
    Eigen::Matrix2f covar;
    mean << end_point_center.x, end_point_center.y;
    covar << (0.5f * 0.33333333f * acc_max_ * planning_horizon_ *
              planning_horizon_),
        0, 0,
        (0.5f * 0.33333333f * acc_max_ * planning_horizon_ * planning_horizon_);
    Eigen::EigenMultivariateNormal<float> normX_solver1(mean, covar);
    Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
    std::vector<Point> end_points_temp;
    for (int j = 0; j < n_rows; j++) {
      tempPoint.x = gaussian_data_eigen.coeffRef(j, 0);
      tempPoint.y = gaussian_data_eigen.coeffRef(j, 1);
      tempPoint.z = end_point_center.z;
      end_points_temp.push_back(tempPoint);
    }
    end_points_.push_back(end_points_temp);
  }
}
void los_keeper::TargetManager2D::ComputePrimitives() {
  primitives_list_.clear();
  StatePoly primitive_temp;
  primitive_temp.SetDegree(3);
  float time_interval_temp[2]{0.0, planning_horizon_};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[4];
  for (int i = 0; i < num_target_; i++) {
    std::vector<StatePoly> primitive_list_temp;
    for (int j = 0; j < num_sample_; j++) {
      { // x-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].px;
        bernstein_coeff_temp[1] =
            target_state_list_[i].px +
            0.33333333f * target_state_list_[i].vx * planning_horizon_;
        bernstein_coeff_temp[2] =
            0.5f * target_state_list_[i].px + 0.5f * end_points_[i][j].x +
            0.16666667f * target_state_list_[i].vx * planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].x;
        primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      { // y-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].py;
        bernstein_coeff_temp[1] =
            target_state_list_[i].py +
            0.33333333f * target_state_list_[i].vy * planning_horizon_;
        bernstein_coeff_temp[2] =
            0.5f * target_state_list_[i].py + 0.5f * end_points_[i][j].y +
            0.16666667f * target_state_list_[i].vy * planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].y;
        primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      { // z-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].pz;
        bernstein_coeff_temp[1] =
            target_state_list_[i].pz +
            0.33333333f * target_state_list_[i].vz * planning_horizon_;
        bernstein_coeff_temp[2] =
            target_state_list_[i].pz +
            0.66666667f * target_state_list_[i].vz * planning_horizon_;
        bernstein_coeff_temp[3] = target_state_list_[i].pz +
                                  target_state_list_[i].vz * planning_horizon_;
        primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      primitive_list_temp.push_back(primitive_temp);
    }
    primitives_list_.push_back(primitive_list_temp);
  }
}
void los_keeper::TargetManager2D::CalculateCloseObstacleIndex() {
  close_obstacle_index_.clear();
  bool is_close;
  for (int i = 0; i < num_target_; i++) {
    std::vector<int> close_obstacle_index_temp;
    for (int j = 0; j < structured_obstacle_poly_list_.size(); j++) {
      is_close =
          powf(target_state_list_[i].px -
                   structured_obstacle_poly_list_[j].px.GetInitialValue(),
               2) +
              powf(target_state_list_[i].py -
                       structured_obstacle_poly_list_[j].py.GetInitialValue(),
                   2) <
          detect_range_ * detect_range_;
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
      std::vector<int> primitive_safe_total_index_temp_;
      for (int j = 0; j < num_sample_; j++) {
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
      std::vector<int> primitive_safe_total_index_temp;
      for (int j = 0; j < num_sample_; j++) {
        is_primitive_safe_pcl_temp.push_back(false);
        is_primitive_safe_structured_obstacle_temp.push_back(false);
      }
      for (int j : primitive_safe_structured_obstacle_index_[i]) {
        is_primitive_safe_structured_obstacle_temp[j] = true;
      }
      for (int j : primitive_safe_pcl_index_[i]) {
        is_primitive_safe_pcl_temp[j] = true;
      }
      for (int j = 0; j < num_sample_; j++) {
        if (is_primitive_safe_pcl_temp[j] and
            is_primitive_safe_structured_obstacle_temp[j])
          primitive_safe_total_index_temp.push_back(j);
      }
      primitive_safe_total_index_.push_back(primitive_safe_total_index_temp);
    }
  }
  return not primitive_safe_total_index_.empty();
}
void los_keeper::TargetManager2D::CalculateCentroid() {
  primitive_best_index_.clear();
  for (int i = 0; i < num_target_; i++) {
    primitive_best_index_.push_back(0);
    float distance_sum_list[(int)primitive_safe_total_index_[i].size()];
    for (int j = 0; j < (int)primitive_safe_total_index_[i].size(); j++) {
      distance_sum_list[j] = 0.0f;
      for (int k = 0; k < (int)primitive_safe_total_index_[i].size(); k++) {
        distance_sum_list[j] +=
            (primitives_list_[i][primitive_safe_total_index_[i][j]]
                 .px.GetTerminalValue() -
             primitives_list_[i][primitive_safe_total_index_[i][k]]
                 .px.GetTerminalValue()) *
                (primitives_list_[i][primitive_safe_total_index_[i][j]]
                     .px.GetTerminalValue() -
                 primitives_list_[i][primitive_safe_total_index_[i][k]]
                     .px.GetTerminalValue()) +
            (primitives_list_[i][primitive_safe_total_index_[i][j]]
                 .py.GetTerminalValue() -
             primitives_list_[i][primitive_safe_total_index_[i][k]]
                 .py.GetTerminalValue()) *
                (primitives_list_[i][primitive_safe_total_index_[i][j]]
                     .py.GetTerminalValue() -
                 primitives_list_[i][primitive_safe_total_index_[i][k]]
                     .py.GetTerminalValue());
      }
    }
    double min_value = 99999999.0f;
    for (int j = 0; j < (int)primitive_safe_total_index_[i].size(); j++) {
      if (min_value > distance_sum_list[j]) {
        min_value = distance_sum_list[j];
        primitive_best_index_[i] = primitive_safe_total_index_[i][j];
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
  for (int i = 0; i < num_target_; i++) {
    bool flag_store_in = true;
    bool flag_store_out = true;
    float value;
    std::vector<int> primitive_safe_structured_obstacle_index_temp;
    for (int j = 0; j < primitives_list_[i].size(); j++) {
      for (int k = 0; k < close_obstacle_index_[i].size(); k++) {
        flag_store_out = true;
        for (int l = 0; l <= 2 * 3; l++) {
          flag_store_in = true;
          value = 0.0f;
          for (int m = std::max(0, l - 3); m <= std::min(3, l); m++) {
            value +=
                (float)nchoosek(3, m) * (float)nchoosek(3, l - m) /
                (float)nchoosek(2 * 3, l) *
                (primitives_list_[i][j].px.GetBernsteinCoefficient()[m] *
                     primitives_list_[i][j]
                         .px.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].px.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .px.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].px.GetBernsteinCoefficient()[l - m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .px.GetBernsteinCoefficient()[m] +
                 structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .px.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .px.GetBernsteinCoefficient()[l - m] + // x-component
                 primitives_list_[i][j].py.GetBernsteinCoefficient()[m] *
                     primitives_list_[i][j]
                         .py.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].py.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .py.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].py.GetBernsteinCoefficient()[l - m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .py.GetBernsteinCoefficient()[m] +
                 structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .py.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .py.GetBernsteinCoefficient()[l - m] // y-component
                );
          }
          if (value <
              powf(structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                           .rx +
                       primitives_list_[i][j].rx,
                   2) +
                  powf(structured_obstacle_poly_list_[close_obstacle_index_[i]
                                                                           [k]]
                               .ry +
                           primitives_list_[i][j].ry,
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
        primitive_safe_structured_obstacle_index_temp.push_back(j);
    }
    primitive_safe_structured_obstacle_index_.push_back(
        primitive_safe_structured_obstacle_index_temp);
  }
}
std::vector<LinearConstraint2D>
los_keeper::TargetManager2D::GenLinearConstraint() {
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
    Vec2f seed = Eigen::Matrix<double, 2, 1>{target_state_list_[i].px,
                                             target_state_list_[i].py};
    SeedDecomp2D decomp_util(seed);
    decomp_util.set_obs(obstacle_pcl);
    decomp_util.set_local_bbox(
        Vec2f(0.5f * virtual_pcl_zone_width_, 0.5f * virtual_pcl_zone_width_));
    decomp_util.dilate(0.1);
    polys.push_back(decomp_util.get_polyhedron());
    auto poly_hedrons = decomp_util.get_polyhedron();
    LinearConstraint2D safe_corridor_constraint(seed,
                                                poly_hedrons.hyperplanes());
    linear_constraint_temp.push_back(safe_corridor_constraint);
  }
  return linear_constraint_temp;
}
void los_keeper::TargetManager2D::CalculateSafePclIndex(
    const std::vector<LinearConstraint2D> &safe_corridor_list) {
  primitive_safe_pcl_index_.clear();
  int num_total_primitives = num_sample_;
  for (int i = 0; i < num_target_; i++) {
    int num_constraints = (int)safe_corridor_list[i].A().rows();
    int num_vars = (int)safe_corridor_list[i].A().cols(); // 2D
    std::vector<int> safe_pcl_index_temp_;
    Eigen::Vector2f A_comp_temp{0.0, 0.0};
    float b_comp_temp{0.0};
    std::vector<Eigen::Vector2f> LinearConstraintA;
    std::vector<float> LinearConstraintB;
    for (int j = 0; j < num_constraints; j++) {
      A_comp_temp[0] = (float)safe_corridor_list[i].A().coeffRef(j, 0);
      A_comp_temp[1] = (float)safe_corridor_list[i].A().coeffRef(j, 1);
      b_comp_temp = (float)safe_corridor_list[i].b().coeffRef(j, 0);
      LinearConstraintA.push_back(A_comp_temp);
      LinearConstraintB.push_back(b_comp_temp);
    }
    float temp_value = 0.0f;
    bool is_safe = true;
    for (int j = 0; j < num_sample_; j++) {
      is_safe = true;
      for (int k = 0; k < (int)LinearConstraintA.size(); k++) {
        for (int l = 0; l < 4; l++) {
          temp_value =
              LinearConstraintA[k].coeffRef(0, 0) *
                  primitives_list_[i][j].px.GetBernsteinCoefficient()[l] +
              LinearConstraintA[k].coeffRef(1, 0) *
                  primitives_list_[i][j].py.GetBernsteinCoefficient()[l] -
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
        safe_pcl_index_temp_.push_back(j);
    }
    primitive_safe_pcl_index_.push_back(safe_pcl_index_temp_);
  }
}

bool los_keeper::TargetManager3D::PredictTargetTrajectory() {
  SampleEndPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  bool is_safe_traj_exist = CheckCollision();
  if (is_safe_traj_exist)
    CalculateCentroid();
  return is_safe_traj_exist;
}
void los_keeper::TargetManager3D::SampleEndPoints() {
  for (int i = 0; i < target_state_list_.size(); i++) {
    Point end_point_center{float(target_state_list_[i].px +
                                 target_state_list_[i].vx * planning_horizon_),
                           float(target_state_list_[i].py +
                                 target_state_list_[i].vy * planning_horizon_),
                           float(target_state_list_[i].pz +
                                 target_state_list_[i].vz * planning_horizon_)};
    uint n_cols = 3;
    uint n_rows = num_sample_;
    using namespace Eigen;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> gaussian_data_eigen;
    gaussian_data_eigen.setZero(n_rows, n_cols);
    Eigen::Vector3f mean;
    Eigen::Matrix3f covar;
    mean << end_point_center.x, end_point_center.y, end_point_center.z;
    covar << (0.5f * 0.33333333f * acc_max_ * planning_horizon_ *
              planning_horizon_),
        0, 0, 0,
        (0.5f * 0.33333333f * acc_max_ * planning_horizon_ * planning_horizon_),
        0, 0, 0,
        (0.5f * 0.33333333f * acc_max_ * planning_horizon_ * planning_horizon_);
    Eigen::EigenMultivariateNormal<float> normX_solver1(mean, covar);
    gaussian_data_eigen << normX_solver1.samples(n_rows).transpose();

    Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
    std::vector<Point> end_points_temp;
    for (int j = 0; j < n_rows; j++) {
      tempPoint.x = gaussian_data_eigen.coeffRef(j, 0);
      tempPoint.y = gaussian_data_eigen.coeffRef(j, 1);
      tempPoint.z = gaussian_data_eigen.coeffRef(j, 2);
      end_points_temp.push_back(tempPoint);
    }
    end_points_.push_back(end_points_temp);
  }
}
void los_keeper::TargetManager3D::CalculateCloseObstacleIndex() {
  close_obstacle_index_.clear();
  bool is_close;
  for (int i = 0; i < num_target_; i++) {
    std::vector<int> close_obstacle_index_temp;
    for (int j = 0; j < structured_obstacle_poly_list_.size(); j++) {
      is_close =
          powf(target_state_list_[i].px -
                   structured_obstacle_poly_list_[j].px.GetInitialValue(),
               2) +
              powf(target_state_list_[i].py -
                       structured_obstacle_poly_list_[j].py.GetInitialValue(),
                   2) +
              powf(target_state_list_[i].pz -
                       structured_obstacle_poly_list_[j].pz.GetInitialValue(),
                   2) <
          detect_range_ * detect_range_;
      if (is_close)
        close_obstacle_index_temp.push_back(j);
    }
    close_obstacle_index_.push_back(close_obstacle_index_temp);
  }
}
bool los_keeper::TargetManager3D::CheckCollision() {
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
      std::vector<int> primitive_safe_total_index_temp_;
      for (int j = 0; j < num_sample_; j++) {
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
      std::vector<int> primitive_safe_total_index_temp;
      for (int j = 0; j < num_sample_; j++) {
        is_primitive_safe_pcl_temp.push_back(false);
        is_primitive_safe_structured_obstacle_temp.push_back(false);
      }
      for (int j : primitive_safe_structured_obstacle_index_[i]) {
        is_primitive_safe_structured_obstacle_temp[j] = true;
      }
      for (int j : primitive_safe_pcl_index_[i]) {
        is_primitive_safe_pcl_temp[j] = true;
      }
      for (int j = 0; j < num_sample_; j++) {
        if (is_primitive_safe_pcl_temp[j] and
            is_primitive_safe_structured_obstacle_temp[j])
          primitive_safe_total_index_temp.push_back(j);
      }
      primitive_safe_total_index_.push_back(primitive_safe_total_index_temp);
    }
  }
  return not primitive_safe_total_index_.empty();
}
void los_keeper::TargetManager3D::ComputePrimitives() {
  primitives_list_.clear();
  StatePoly primitive_temp;
  primitive_temp.SetDegree(3);
  float time_interval_temp[2]{0.0, planning_horizon_};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[4];
  for (int i = 0; i < num_target_; i++) {
    std::vector<StatePoly> primitive_list_temp;
    for (int j = 0; j < num_sample_; j++) {
      { // x-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].px;
        bernstein_coeff_temp[1] =
            target_state_list_[i].px +
            0.33333333f * target_state_list_[i].vx * planning_horizon_;
        bernstein_coeff_temp[2] =
            0.5f * target_state_list_[i].px + 0.5f * end_points_[i][j].x +
            0.16666667f * target_state_list_[i].vx * planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].x;
        primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      { // y-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].py;
        bernstein_coeff_temp[1] =
            target_state_list_[i].py +
            0.33333333f * target_state_list_[i].vy * planning_horizon_;
        bernstein_coeff_temp[2] =
            0.5f * target_state_list_[i].py + 0.5f * end_points_[i][j].y +
            0.16666667f * target_state_list_[i].vy * planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].y;
        primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      { // z-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].pz;
        bernstein_coeff_temp[1] =
            target_state_list_[i].pz +
            0.33333333f * target_state_list_[i].vz * planning_horizon_;
        bernstein_coeff_temp[2] =
            0.5f * target_state_list_[i].pz + 0.5f * end_points_[i][j].z +
            0.16666667f * target_state_list_[i].vz * planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].z;
        primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      primitive_list_temp.push_back(primitive_temp);
    }
    primitives_list_.push_back(primitive_list_temp);
  }
}
void los_keeper::TargetManager3D::CalculateCentroid() {
  primitive_best_index_.clear();
  for (int i = 0; i < num_target_; i++) {
    primitive_best_index_.push_back(0);
    float distance_sum_list[(int)primitive_safe_total_index_[i].size()];
    for (int j = 0; j < (int)primitive_safe_total_index_[i].size(); j++) {
      distance_sum_list[j] = 0.0f;
      for (int k = 0; k < (int)primitive_safe_total_index_[i].size(); k++) {
        distance_sum_list[j] +=
            (primitives_list_[i][primitive_safe_total_index_[i][j]]
                 .px.GetTerminalValue() -
             primitives_list_[i][primitive_safe_total_index_[i][k]]
                 .px.GetTerminalValue()) *
                (primitives_list_[i][primitive_safe_total_index_[i][j]]
                     .px.GetTerminalValue() -
                 primitives_list_[i][primitive_safe_total_index_[i][k]]
                     .px.GetTerminalValue()) +
            (primitives_list_[i][primitive_safe_total_index_[i][j]]
                 .py.GetTerminalValue() -
             primitives_list_[i][primitive_safe_total_index_[i][k]]
                 .py.GetTerminalValue()) *
                (primitives_list_[i][primitive_safe_total_index_[i][j]]
                     .py.GetTerminalValue() -
                 primitives_list_[i][primitive_safe_total_index_[i][k]]
                     .py.GetTerminalValue()) +
            (primitives_list_[i][primitive_safe_total_index_[i][j]]
                 .pz.GetTerminalValue() -
             primitives_list_[i][primitive_safe_total_index_[i][k]]
                 .pz.GetTerminalValue()) *
                (primitives_list_[i][primitive_safe_total_index_[i][j]]
                     .pz.GetTerminalValue() -
                 primitives_list_[i][primitive_safe_total_index_[i][k]]
                     .pz.GetTerminalValue());
      }
    }
    double min_value = 99999999.0f;
    for (int j = 0; j < (int)primitive_safe_total_index_[i].size(); j++) {
      if (min_value > distance_sum_list[j]) {
        min_value = distance_sum_list[j];
        primitive_best_index_[i] = primitive_safe_total_index_[i][j];
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
  for (int i = 0; i < num_target_; i++) {
    bool flag_store_in = true;
    bool flag_store_out = true;
    float value;
    std::vector<int> primitive_safe_structured_obstacle_index_temp;
    for (int j = 0; j < primitives_list_[i].size(); j++) {
      for (int k = 0; k < close_obstacle_index_[i].size(); k++) {
        flag_store_out = true;
        for (int l = 0; l <= 2 * 3; l++) {
          flag_store_in = true;
          value = 0.0f;
          for (int m = std::max(0, l - 3); m <= std::min(3, l); m++) {
            value +=
                (float)nchoosek(3, m) * (float)nchoosek(3, l - m) /
                (float)nchoosek(2 * 3, l) *
                (primitives_list_[i][j].px.GetBernsteinCoefficient()[m] *
                     primitives_list_[i][j]
                         .px.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].px.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .px.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].px.GetBernsteinCoefficient()[l - m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .px.GetBernsteinCoefficient()[m] +
                 structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .px.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .px.GetBernsteinCoefficient()[l - m] + // x-component
                 primitives_list_[i][j].py.GetBernsteinCoefficient()[m] *
                     primitives_list_[i][j]
                         .py.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].py.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .py.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].py.GetBernsteinCoefficient()[l - m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .py.GetBernsteinCoefficient()[m] +
                 structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .py.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .py.GetBernsteinCoefficient()[l - m] + // y-component
                 primitives_list_[i][j].pz.GetBernsteinCoefficient()[m] *
                     primitives_list_[i][j]
                         .pz.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].pz.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .pz.GetBernsteinCoefficient()[l - m] -
                 primitives_list_[i][j].pz.GetBernsteinCoefficient()[l - m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .pz.GetBernsteinCoefficient()[m] +
                 structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .pz.GetBernsteinCoefficient()[m] *
                     structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                         .pz.GetBernsteinCoefficient()[l - m] // y-component
                );
          }
          if (value <
              powf(structured_obstacle_poly_list_[close_obstacle_index_[i][k]]
                           .rx +
                       primitives_list_[i][j].rx,
                   2) +
                  powf(structured_obstacle_poly_list_[close_obstacle_index_[i]
                                                                           [k]]
                               .ry +
                           primitives_list_[i][j].ry,
                       2) +
                  powf(structured_obstacle_poly_list_[close_obstacle_index_[i]
                                                                           [k]]
                               .rz +
                           primitives_list_[i][j].rz,
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
        primitive_safe_structured_obstacle_index_temp.push_back(j);
    }
    primitive_safe_structured_obstacle_index_.push_back(
        primitive_safe_structured_obstacle_index_temp);
  }
}
std::vector<LinearConstraint3D>
los_keeper::TargetManager3D::GenLinearConstraint() {
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
    Vec3f seed = Eigen::Matrix<double, 3, 1>{target_state_list_[i].px,
                                             target_state_list_[i].py,
                                             target_state_list_[i].pz};
    SeedDecomp3D decomp_util(seed);
    decomp_util.set_obs(obstacle_pcl);
    decomp_util.set_local_bbox(Vec3f(0.5f * virtual_pcl_zone_width_,
                                     0.5f * virtual_pcl_zone_width_,
                                     0.5f * virtual_pcl_zone_height_));
    decomp_util.dilate(0.1);
    polys.push_back(decomp_util.get_polyhedron());
    auto poly_hedrons = decomp_util.get_polyhedron();
    LinearConstraint3D safe_corridor_constraint(seed,
                                                poly_hedrons.hyperplanes());
    linear_constraint_temp.push_back(safe_corridor_constraint);
  }
  return linear_constraint_temp;
}
void los_keeper::TargetManager3D::CalculateSafePclIndex(
    const std::vector<LinearConstraint3D> &safe_corridor_list) {
  primitive_safe_pcl_index_.clear();
  int num_total_primitives = num_sample_;
  for (int i = 0; i < num_target_; i++) {
    int num_constraints = (int)safe_corridor_list[i].A().rows();
    int num_vars = (int)safe_corridor_list[i].A().cols(); // 3D
    std::vector<int> safe_pcl_index_temp_;
    Eigen::Vector3f A_comp_temp{0.0, 0.0, 0.0};
    float b_comp_temp{0.0};
    std::vector<Eigen::Vector3f> LinearConstraintA;
    std::vector<float> LinearConstraintB;
    for (int j = 0; j < num_constraints; j++) {
      A_comp_temp[0] = (float)safe_corridor_list[i].A().coeffRef(j, 0);
      A_comp_temp[1] = (float)safe_corridor_list[i].A().coeffRef(j, 1);
      A_comp_temp[2] = (float)safe_corridor_list[i].A().coeffRef(j, 2);
      b_comp_temp = (float)safe_corridor_list[i].b().coeffRef(j, 0);
      LinearConstraintA.push_back(A_comp_temp);
      LinearConstraintB.push_back(b_comp_temp);
    }
    float temp_value = 0.0f;
    bool is_safe = true;
    for (int j = 0; j < num_sample_; j++) {
      is_safe = true;
      for (int k = 0; k < (int)LinearConstraintA.size(); k++) {
        for (int l = 0; l < 4; l++) {
          temp_value =
              LinearConstraintA[k].coeffRef(0, 0) *
                  primitives_list_[i][j].px.GetBernsteinCoefficient()[l] +
              LinearConstraintA[k].coeffRef(1, 0) *
                  primitives_list_[i][j].py.GetBernsteinCoefficient()[l] +
              LinearConstraintA[k].coeffRef(1, 0) *
                  primitives_list_[i][j].pz.GetBernsteinCoefficient()[l] -
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
        safe_pcl_index_temp_.push_back(j);
    }
    primitive_safe_pcl_index_.push_back(safe_pcl_index_temp_);
  }
}
