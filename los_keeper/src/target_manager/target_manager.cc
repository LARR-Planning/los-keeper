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
los_keeper::TargetManager::TargetManager() { //Abstract Target Manager
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
  CalculateCentroid();
  return is_safe_traj_exist;
}
void los_keeper::TargetManager2D::SampleEndPoints() {
  for(int i =0;i<target_state_list_.size();i++){
    Point end_point_center{float(target_state_list_[i].px+target_state_list_[i].vx*planning_horizon_),
                           float(target_state_list_[i].py+target_state_list_[i].vy*planning_horizon_),
                           float(target_state_list_[i].pz+target_state_list_[i].vz*planning_horizon_)};
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
}
void los_keeper::TargetManager2D::ComputePrimitives() {
  primitives_list_.clear();
  StatePoly primitive_temp;
  primitive_temp.SetDegree(3);
  float time_interval_temp[2] {0.0,planning_horizon_};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[4];
  for(int i=0;i<num_target_;i++){
    std::vector<StatePoly> primitive_list_temp;
    for(int j =0;j<num_sample_;j++){
      { // x-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].px;
        bernstein_coeff_temp[1] = target_state_list_[i].px + 0.33333333f*target_state_list_[i].vx*planning_horizon_;
        bernstein_coeff_temp[2] = 0.5f*target_state_list_[i].px + 0.5f*end_points_[i][j].x+0.16666667f*target_state_list_[i].vx*planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].x;
        primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      { // y-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].py;
        bernstein_coeff_temp[1] = target_state_list_[i].py + 0.33333333f*target_state_list_[i].vy*planning_horizon_;
        bernstein_coeff_temp[2] = 0.5f*target_state_list_[i].py + 0.5f*end_points_[i][j].y+0.16666667f*target_state_list_[i].vy*planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].y;
        primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      { // z-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].pz;
        bernstein_coeff_temp[1] = target_state_list_[i].pz + 0.33333333f*target_state_list_[i].vz*planning_horizon_;
        bernstein_coeff_temp[2] = target_state_list_[i].pz + 0.66666667f*target_state_list_[i].vz*planning_horizon_;
        bernstein_coeff_temp[3] = target_state_list_[i].pz + target_state_list_[i].vz*planning_horizon_;
        primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      primitive_list_temp.push_back(primitive_temp);
    }
    primitives_list_.push_back(primitive_list_temp);
  }
}
void los_keeper::TargetManager2D::CalculateCloseObstacleIndex() {
  primitive_close_obstacle_index_.clear();
  bool is_close;
  for(int i =0;i<num_target_;i++){
    std::vector<int> close_obstacle_index_temp;
    for(int j =0;j<structured_obstacle_poly_list_.size();j++){
      is_close = powf(target_state_list_[i].px-structured_obstacle_poly_list_[j].px.GetInitialValue(),2)+
                 powf(target_state_list_[i].py-structured_obstacle_poly_list_[j].py.GetInitialValue(),2)<detect_range_*detect_range_;
      if(is_close)
        close_obstacle_index_temp.push_back(j);
    }
    primitive_close_obstacle_index_.push_back(close_obstacle_index_temp);
  }
}
bool los_keeper::TargetManager2D::CheckCollision() {
  bool is_cloud_empty = cloud_.points.empty();
  bool is_structured_obstacle_empty = structured_obstacle_poly_list_.empty();
  if(not is_cloud_empty){
    CheckPclCollision();
  }
  return true;
}
void los_keeper::TargetManager2D::CalculateCentroid() {

}
void los_keeper::TargetManager2D::CheckPclCollision() {
  std::vector<LinearConstraint2D> safe_corridor = GenLinearConstraint();
  GetSafePclIndex(safe_corridor);
}
void los_keeper::TargetManager2D::CheckStructuredObstacleCollision() {

}
std::vector<LinearConstraint2D> los_keeper::TargetManager2D::GenLinearConstraint() {
  Vec2f pcl_points_temp;
  vec_Vec2f obstacle_pcl;
  for(const auto & point : cloud_.points){
    pcl_points_temp.coeffRef(0,0) = point.x;
    pcl_points_temp.coeffRef(1,0) = point.y;
    obstacle_pcl.push_back(pcl_points_temp);
  }
  polys.clear();
  std::vector<LinearConstraint2D> linear_constraint_temp;
  for(int i =0;i<num_target_;i++){
    Vec2f seed = Eigen::Matrix<double,2,1> {target_state_list_[i].px,target_state_list_[i].py};
    SeedDecomp2D decomp_util(seed);
    decomp_util.set_obs(obstacle_pcl);
    decomp_util.set_local_bbox(Vec2f(0.5f*virtual_pcl_zone_width_,0.5f*virtual_pcl_zone_width_));
    decomp_util.dilate(0.1);
    polys.push_back(decomp_util.get_polyhedron());
    auto poly_hedrons = decomp_util.get_polyhedron();
    LinearConstraint2D safe_corridor_constraint(seed,poly_hedrons.hyperplanes());
    linear_constraint_temp.push_back(safe_corridor_constraint);
  }
  return linear_constraint_temp;
}
void los_keeper::TargetManager2D::GetSafePclIndex(
    const std::vector<LinearConstraint2D> &safe_corridor_list) {
  primitive_safe_pcl_index_.clear();
  int num_total_primitives = num_sample_;
  for(int i =0;i<num_target_;i++){
    int num_constraints = (int)safe_corridor_list[i].A().rows();
    int num_vars = (int)safe_corridor_list[i].A().cols(); //2D
    std::vector<int> safe_pcl_index_temp_;
    Eigen::Vector2f A_comp_temp{0.0,0.0};
    float b_comp_temp{0.0};
    std::vector<Eigen::Vector2f> LinearConstraintA;
    std::vector<float> LinearConstraintB;
    for(int j=0;j<num_constraints;j++){
      A_comp_temp[0] = (float)safe_corridor_list[i].A().coeffRef(j,0);
      A_comp_temp[1] = (float)safe_corridor_list[i].A().coeffRef(j,1);
      b_comp_temp =(float)safe_corridor_list[i].b().coeffRef(j,0);
      LinearConstraintA.push_back(A_comp_temp);
      LinearConstraintB.push_back(b_comp_temp);
    }
    float temp_value = 0.0f;
    bool is_safe = true;
    for(int j=0;j<num_sample_;j++){
      is_safe = true;
      for(int k=0;k<(int)LinearConstraintA.size();k++){
        for(int l=0;l<4;l++){
          temp_value = LinearConstraintA[k].coeffRef(0,0)*primitives_list_[i][j].px.GetBernsteinCoefficient()[l]+
              LinearConstraintA[k].coeffRef(1,0)*primitives_list_[i][j].py.GetBernsteinCoefficient()[l]-
              LinearConstraintB[k];
          if(temp_value>0.0f){
            is_safe = false;
            break;
          }
        }
        if(not is_safe)
          break;
      }
      if(is_safe)
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
  CalculateCentroid();
  return is_safe_traj_exist;
}
void los_keeper::TargetManager3D::SampleEndPoints() {
  for(int i =0;i<target_state_list_.size();i++){
    Point end_point_center{float(target_state_list_[i].px+target_state_list_[i].vx*planning_horizon_),
                           float(target_state_list_[i].py+target_state_list_[i].vy*planning_horizon_),
                           float(target_state_list_[i].pz+target_state_list_[i].vz*planning_horizon_)};
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
void los_keeper::TargetManager3D::CalculateCloseObstacleIndex() {
  primitive_close_obstacle_index_.clear();
  bool is_close;
  for(int i =0;i<num_target_;i++){
    std::vector<int> close_obstacle_index_temp;
    for(int j =0;j<structured_obstacle_poly_list_.size();j++){
      is_close = powf(target_state_list_[i].px-structured_obstacle_poly_list_[j].px.GetInitialValue(),2)+
                     powf(target_state_list_[i].py-structured_obstacle_poly_list_[j].py.GetInitialValue(),2)+
                     powf(target_state_list_[i].pz-structured_obstacle_poly_list_[j].pz.GetInitialValue(),2)
                 <detect_range_*detect_range_;
      if(is_close)
        close_obstacle_index_temp.push_back(j);
    }
    primitive_close_obstacle_index_.push_back(close_obstacle_index_temp);
  }
}
bool los_keeper::TargetManager3D::CheckCollision() {
  bool is_cloud_empty = cloud_.points.empty();
  bool is_structured_obstacle_empty = structured_obstacle_poly_list_.empty();
  if(not is_cloud_empty){
    CheckPclCollision();
  }
  return true;
}
void los_keeper::TargetManager3D::ComputePrimitives() {
  primitives_list_.clear();
  StatePoly primitive_temp;
  primitive_temp.SetDegree(3);
  float time_interval_temp[2] {0.0,planning_horizon_};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[4];
  for(int i=0;i<num_target_;i++){
    std::vector<StatePoly> primitive_list_temp;
    for(int j =0;j<num_sample_;j++){
      { // x-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].px;
        bernstein_coeff_temp[1] = target_state_list_[i].px + 0.33333333f*target_state_list_[i].vx*planning_horizon_;
        bernstein_coeff_temp[2] = 0.5f*target_state_list_[i].px + 0.5f*end_points_[i][j].x+0.16666667f*target_state_list_[i].vx*planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].x;
        primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      { // y-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].py;
        bernstein_coeff_temp[1] = target_state_list_[i].py + 0.33333333f*target_state_list_[i].vy*planning_horizon_;
        bernstein_coeff_temp[2] = 0.5f*target_state_list_[i].py + 0.5f*end_points_[i][j].y+0.16666667f*target_state_list_[i].vy*planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].y;
        primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      { // z-coefficient
        bernstein_coeff_temp[0] = target_state_list_[i].pz;
        bernstein_coeff_temp[1] = target_state_list_[i].pz + 0.33333333f*target_state_list_[i].vz*planning_horizon_;
        bernstein_coeff_temp[2] = 0.5f*target_state_list_[i].pz + 0.5f*end_points_[i][j].z+ 0.16666667f*target_state_list_[i].vz*planning_horizon_;
        bernstein_coeff_temp[3] = end_points_[i][j].z;
        primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
      }
      primitive_list_temp.push_back(primitive_temp);
    }
    primitives_list_.push_back(primitive_list_temp);
  }
}
void los_keeper::TargetManager3D::CalculateCentroid() {

}
void los_keeper::TargetManager3D::CheckPclCollision() {
  std::vector<LinearConstraint3D> safe_corridor = GenLinearConstraint();
  GetSafePclIndex(safe_corridor);
}
void los_keeper::TargetManager3D::CheckStructuredObstacleCollision() {

}
std::vector<LinearConstraint3D>
los_keeper::TargetManager3D::GenLinearConstraint() {
  Vec3f pcl_points_temp;
  vec_Vec3f obstacle_pcl;
  for(const auto & point : cloud_.points){
    pcl_points_temp.coeffRef(0,0) = point.x;
    pcl_points_temp.coeffRef(1,0) = point.y;
    pcl_points_temp.coeffRef(2,0) = point.z;
    obstacle_pcl.push_back(pcl_points_temp);
  }
  polys.clear();
  std::vector<LinearConstraint3D> linear_constraint_temp;
  for(int i =0;i<num_target_;i++){
    Vec3f seed = Eigen::Matrix<double,3,1> {target_state_list_[i].px,target_state_list_[i].py, target_state_list_[i].pz};
    SeedDecomp3D decomp_util(seed);
    decomp_util.set_obs(obstacle_pcl);
    decomp_util.set_local_bbox(Vec3f(0.5f*virtual_pcl_zone_width_,0.5f*virtual_pcl_zone_width_,0.5f*virtual_pcl_zone_height_));
    decomp_util.dilate(0.1);
    polys.push_back(decomp_util.get_polyhedron());
    auto poly_hedrons = decomp_util.get_polyhedron();
    LinearConstraint3D safe_corridor_constraint(seed,poly_hedrons.hyperplanes());
    linear_constraint_temp.push_back(safe_corridor_constraint);
  }
  return linear_constraint_temp;
}
void los_keeper::TargetManager3D::GetSafePclIndex(
    const std::vector<LinearConstraint3D> &safe_corridor_list) {
  primitive_safe_pcl_index_.clear();
  int num_total_primitives = num_sample_;
  for(int i =0;i<num_target_;i++){
    int num_constraints = (int)safe_corridor_list[i].A().rows();
    int num_vars = (int)safe_corridor_list[i].A().cols(); //3D
    std::vector<int> safe_pcl_index_temp_;
    Eigen::Vector3f A_comp_temp{0.0,0.0, 0.0};
    float b_comp_temp{0.0};
    std::vector<Eigen::Vector3f> LinearConstraintA;
    std::vector<float> LinearConstraintB;
    for(int j=0;j<num_constraints;j++){
      A_comp_temp[0] = (float)safe_corridor_list[i].A().coeffRef(j,0);
      A_comp_temp[1] = (float)safe_corridor_list[i].A().coeffRef(j,1);
      A_comp_temp[2] = (float)safe_corridor_list[i].A().coeffRef(j,2);
      b_comp_temp =(float)safe_corridor_list[i].b().coeffRef(j,0);
      LinearConstraintA.push_back(A_comp_temp);
      LinearConstraintB.push_back(b_comp_temp);
    }
    float temp_value = 0.0f;
    bool is_safe = true;
    for(int j=0;j<num_sample_;j++){
      is_safe = true;
      for(int k=0;k<(int)LinearConstraintA.size();k++){
        for(int l=0;l<4;l++){
          temp_value = LinearConstraintA[k].coeffRef(0,0)*primitives_list_[i][j].px.GetBernsteinCoefficient()[l]+
                       LinearConstraintA[k].coeffRef(1,0)*primitives_list_[i][j].py.GetBernsteinCoefficient()[l]+
                       LinearConstraintA[k].coeffRef(1,0)*primitives_list_[i][j].pz.GetBernsteinCoefficient()[l]-
                       LinearConstraintB[k];
          if(temp_value>0.0f){
            is_safe = false;
            break;
          }
        }
        if(not is_safe)
          break;
      }
      if(is_safe)
        safe_pcl_index_temp_.push_back(j);
    }
    primitive_safe_pcl_index_.push_back(safe_pcl_index_temp_);
  }
}
