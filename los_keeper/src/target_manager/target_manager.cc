#include "los_keeper/target_manager/target_manager.h"

bool los_keeper::TargetManager::CheckCollision(
    const ObstacleManager &obstacle_manager) const {
  return obstacle_manager.GetName() == "ObstacleManager";
}
std::string los_keeper::TargetManager::GetName() const { return name_; }
void los_keeper::TargetManager::SetTargetState(
    const std::vector<ObjectState> &target_state_list) {
  target_state_list_.clear();
  target_state_list_ = target_state_list;
  num_target_ = (int)target_state_list_.size();
}
void los_keeper::TargetManager::SetObstacleState(
    pcl::PointCloud<pcl::PointXYZ> cloud,
    std::vector<StatePoly> structured_obstacle_poly_list) {
  cloud_.points.clear();
  structured_obstacle_poly_list_.clear();
  cloud_.points = cloud.points;
  structured_obstacle_poly_list_ = structured_obstacle_poly_list;
}
bool los_keeper::TargetManager::PredictTargetTrajectory() {
  SampleEndPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  CheckCollision();
  CalculateCentroid();
  return true;
}
void los_keeper::TargetManager::SampleEndPoints() {
  for (int i = 0; i < target_state_list_.size(); i++) {
    Point end_point_center{float(target_state_list_[i].px +
                                 target_state_list_[i].vx * planning_horizon_),
                           float(target_state_list_[i].py +
                                 target_state_list_[i].vy * planning_horizon_),
                           float(target_state_list_[i].pz +
                                 target_state_list_[i].vz * planning_horizon_)};
    if (is_2d_) {
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
          (0.5f * 0.33333333f * acc_max_ * planning_horizon_ *
           planning_horizon_);
      Eigen::EigenMultivariateNormal<float> normX_solver1(mean, covar);
      Point tempPoint{end_point_center.x, end_point_center.y,
                      end_point_center.z};
      std::vector<Point> end_points_temp;
      for (int j = 0; j < n_rows; j++) {
        tempPoint.x = gaussian_data_eigen.coeffRef(j, 0);
        tempPoint.y = gaussian_data_eigen.coeffRef(j, 1);
        tempPoint.z = end_point_center.z;
        end_points_temp.push_back(tempPoint);
      }
      end_points_.push_back(end_points_temp);
    } else {
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
          (0.5f * 0.33333333f * acc_max_ * planning_horizon_ *
           planning_horizon_),
          0, 0, 0,
          (0.5f * 0.33333333f * acc_max_ * planning_horizon_ *
           planning_horizon_);
      Eigen::EigenMultivariateNormal<float> normX_solver1(mean, covar);
      gaussian_data_eigen << normX_solver1.samples(n_rows).transpose();

      Point tempPoint{end_point_center.x, end_point_center.y,
                      end_point_center.z};
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
}
los_keeper::TargetManager::TargetManager() { // Abstract Target Manager
}
void los_keeper::TargetManager::ComputePrimitives() {}
void los_keeper::TargetManager::CalculateCloseObstacleIndex() {}
void los_keeper::TargetManager::CalculateCentroid() {}

bool los_keeper::TargetManager2D::PredictTargetTrajectory() {
  SampleEndPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  bool is_safe_traj_exist = CheckCollision();
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
bool los_keeper::TargetManager2D::CheckCollision() { return false; }
void los_keeper::TargetManager2D::CalculateCentroid() {
  TargetManager::CalculateCentroid();
}

bool los_keeper::TargetManager3D::PredictTargetTrajectory() {
  SampleEndPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  bool is_safe_traj_exist = CheckCollision();
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
bool los_keeper::TargetManager3D::CheckCollision() { return false; }
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
  TargetManager::CalculateCentroid();
}
