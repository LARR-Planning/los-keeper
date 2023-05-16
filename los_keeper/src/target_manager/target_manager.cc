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
  num_target_ = (int) target_state_list_.size();
}
void los_keeper::TargetManager::SetObstacleState(
    pcl::PointCloud<pcl::PointXYZ> cloud,
    std::vector<StatePoly> structured_obstacle_poly_list) {
  cloud_.points.clear();
  structured_obstacle_poly_list_.clear();
  cloud_.points = cloud.points;
  structured_obstacle_poly_list_ = structured_obstacle_poly_list;
}
void los_keeper::TargetManager::PredictTargetTrajectory() {
  SampleEndPoints();
  ComputePrimitives();
  CalculateCloseObstacleIndex();
  CheckCollision();
  CalculateCentroid();
}
void los_keeper::TargetManager::SampleEndPoints() {
  for(int i =0;i<target_state_list_.size();i++){
    Point end_point_center{float(target_state_list_[i].px+target_state_list_[i].vx*planning_horizon_),
                           float(target_state_list_[i].py+target_state_list_[i].vy*planning_horizon_),
                           float(target_state_list_[i].pz+target_state_list_[i].vz*planning_horizon_)};
    if (is_2d_){
      uint n_cols = 2;
      uint n_rows = num_sample_;
      using namespace Eigen;
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> gaussian_data_eigen;
      gaussian_data_eigen.setZero(n_rows,n_cols);
      Eigen::Vector2f mean;
      Eigen::Matrix2f covar;
      mean<<end_point_center.x, end_point_center.y;
      covar<< (0.5f*0.33333333f*acc_max_*planning_horizon_*planning_horizon_), 0,
          0, (0.5f*0.33333333f*acc_max_*planning_horizon_*planning_horizon_);
      Eigen::EigenMultivariateNormal<float> normX_solver1(mean,covar);
      Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
      std::vector<Point> end_points_temp;
      for(int j = 0;j<n_rows;j++){
        tempPoint.x = gaussian_data_eigen.coeffRef(j,0);
        tempPoint.y = gaussian_data_eigen.coeffRef(j,1);
        tempPoint.z = end_point_center.z;
        end_points_temp.push_back(tempPoint);
      }
      end_points_.push_back(end_points_temp);
    }
    else{
      uint n_cols = 3;
      uint n_rows = num_sample_;
      using namespace Eigen;
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> gaussian_data_eigen;
      gaussian_data_eigen.setZero(n_rows,n_cols);
      Eigen::Vector3f mean;
      Eigen::Matrix3f covar;
      mean<<end_point_center.x, end_point_center.y, end_point_center.z;
      covar<< (0.5f*0.33333333f*acc_max_*planning_horizon_*planning_horizon_), 0, 0,
          0, (0.5f*0.33333333f*acc_max_*planning_horizon_*planning_horizon_), 0,
          0, 0, (0.5f*0.33333333f*acc_max_*planning_horizon_*planning_horizon_);
      Eigen::EigenMultivariateNormal<float> normX_solver1(mean,covar);
      gaussian_data_eigen<< normX_solver1.samples(n_rows).transpose();

      Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
      std::vector<Point> end_points_temp;
      for(int j = 0;j<n_rows;j++){
        tempPoint.x = gaussian_data_eigen.coeffRef(j,0);
        tempPoint.y = gaussian_data_eigen.coeffRef(j,1);
        tempPoint.z = gaussian_data_eigen.coeffRef(j,2);
        end_points_temp.push_back(tempPoint);
      }
      end_points_.push_back(end_points_temp);
    }
  }
}

void los_keeper::TargetManager::ComputePrimitives() {}
void los_keeper::TargetManager::CalculateCloseObstacleIndex() {}
void los_keeper::TargetManager::CheckCollision() {}
void los_keeper::TargetManager::CalculateCentroid() {}
