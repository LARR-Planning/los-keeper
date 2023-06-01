#include "los_keeper/trajectory_planner/trajectory_planner.h"
std::optional<StatePoly> TrajectoryPlanner::ComputeChasingTrajectory(
    const std::vector<StatePoly> &target_prediction_list,
    const los_keeper::PclPointCloud &obstacle_points,
    const std::vector<StatePoly> &structured_obstacle_poly_list) const {
  return std::optional<StatePoly>();
}
TrajectoryPlanner::TrajectoryPlanner() {}
void TrajectoryPlanner::SetTargetState(const PrimitiveList &target_trajectory_list) {
  target_trajectory_list_.clear();
  target_trajectory_list_ = target_trajectory_list;
  num_target_ = (int)target_trajectory_list.size();
}
void TrajectoryPlanner::SetObstacleState(const pcl::PointCloud<pcl::PointXYZ>& cloud,
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
bool TrajectoryPlanner2D::PlanKeeperTrajectory() {
  SampleShootingPoints();
  return false;
}
void TrajectoryPlanner2D::SampleShootingPoints() {
  shooting_points_.clear();
  for(int i =0;i<num_target_;i++){
    int num_chunk =num_sample_/num_thread_/num_target_;
    vector<thread> worker_thread;
    vector<vector<Point>> shooting_point_temp(num_thread_);
    for(int j =0;j<num_thread_;j++){
      worker_thread.emplace_back(&TrajectoryPlanner2D::SampleShootingPointsSubProcess,this,i,num_chunk,std::ref(shooting_point_temp[j]));
    }
    for(int j =0;j<num_thread_;j++){
      worker_thread[j].join();
    }
    for(int j=0;j<num_thread_;j++){
      for(int k =0;k<shooting_point_temp[j].size();k++){
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
  std::uniform_real_distribution<> r_dis(target_distance_min, target_distance_max);
  std::uniform_real_distribution<> theta_dis(-M_PI, M_PI);
  double r, theta;
  Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
  for(int i=0;i<chunk_size;i++){
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
  int num_chunk = num_sample_/num_thread_;
  vector<thread> worker_thread;
  PrimitiveListSet primitive_list_temp;
  for(int i = 0; i<num_thread_;i++){
    worker_thread.emplace_back(&TrajectoryPlanner2D::ComputePrimitivesSubProcess,this,num_chunk*(i),num_chunk*(i+1),std::ref(primitive_list_temp[i]));
  }
  for(int i = 0;i<num_thread_;i++){
    worker_thread[i].join();
  }
  for(int i =0;i<num_thread_;i++){
    for(int j =0;j<primitive_list_temp[i].size();j++){
      primitives_list_.push_back(primitive_list_temp[i][j]);
    }
  }
}
void TrajectoryPlanner2D::ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                      PrimitiveList &primitive_list_sub) {
  StatePoly primitive_temp;
  primitive_temp.SetDegree(5);
  float time_interval_temp[2]{0.0, planning_horizon_};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[6];
  float planning_horizon_square = planning_horizon_*planning_horizon_;

  for(int i = start_idx; i<end_idx;i++){
    { // x-component
     bernstein_coeff_temp[0] = drone_state_.px;
     bernstein_coeff_temp[1] = drone_state_.px + 0.2f*planning_horizon_*drone_state_.vx;
     bernstein_coeff_temp[2] = drone_state_.px + 0.4f*planning_horizon_*drone_state_.vx + 0.05f*planning_horizon_square*drone_state_.ax;
     bernstein_coeff_temp[3] = 0.16666667f*shooting_points_[i].x + 0.83333333f*drone_state_.px + 0.43333333f*planning_horizon_*drone_state_.vx + 0.06666667f*planning_horizon_square*drone_state_.ax;
     bernstein_coeff_temp[4] = 0.5f*shooting_points_[i].x + 0.5f*drone_state_.px + 0.3f*planning_horizon_*drone_state_.vx + 0.05f*planning_horizon_square*drone_state_.ax;
     bernstein_coeff_temp[5] = shooting_points_[i].x;
     primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // y-component
     bernstein_coeff_temp[0] = drone_state_.py;
     bernstein_coeff_temp[1] = drone_state_.py + 0.2f*planning_horizon_*drone_state_.vy;
     bernstein_coeff_temp[2] = drone_state_.py + 0.4f*planning_horizon_*drone_state_.vy + 0.05f*planning_horizon_square*drone_state_.ay;
     bernstein_coeff_temp[3] = 0.16666667f*shooting_points_[i].y + 0.83333333f*drone_state_.py + 0.43333333f*planning_horizon_*drone_state_.vy + 0.06666667f*planning_horizon_square*drone_state_.ay;
     bernstein_coeff_temp[4] = 0.5f*shooting_points_[i].y + 0.5f*drone_state_.py + 0.3f*planning_horizon_*drone_state_.vy + 0.05f*planning_horizon_square*drone_state_.ay;
     bernstein_coeff_temp[5] = shooting_points_[i].y;
     primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // z-component
      bernstein_coeff_temp[0] = drone_state_.pz;
      bernstein_coeff_temp[1] = drone_state_.pz + 0.2f*planning_horizon_*drone_state_.vz;
      bernstein_coeff_temp[2] = drone_state_.pz + 0.4f*planning_horizon_*drone_state_.vz;
      bernstein_coeff_temp[3] = drone_state_.pz + 0.6f*planning_horizon_*drone_state_.vz;
      bernstein_coeff_temp[4] = drone_state_.pz + 0.8f*planning_horizon_*drone_state_.vz;
      bernstein_coeff_temp[5] = drone_state_.pz + 1.0f*planning_horizon_*drone_state_.vz;
      primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    primitive_list_sub.push_back(primitive_temp);
  }
}
bool TrajectoryPlanner3D::PlanKeeperTrajectory() {
  SampleShootingPoints();
  return false; }
void TrajectoryPlanner3D::SampleShootingPoints() {
  shooting_points_.clear();
  for(int i =0;i<num_target_;i++){
    int num_chunk =num_sample_/num_thread_/num_target_;
    vector<thread> worker_thread;
    vector<vector<Point>> shooting_point_temp(num_thread_);
    for(int j =0;j<num_thread_;j++){
      worker_thread.emplace_back(&TrajectoryPlanner3D::SampleShootingPointsSubProcess,this,i,num_chunk,std::ref(shooting_point_temp[j]));
    }
    for(int j =0;j<num_thread_;j++){
      worker_thread[j].join();
    }
    for(int j=0;j<num_thread_;j++){
      for(int k =0;k<shooting_point_temp[j].size();k++){
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
  std::uniform_real_distribution<> r_dis(target_distance_min, target_distance_max);
  std::uniform_real_distribution<> azimuth_dis(-M_PI, M_PI);
  std::uniform_real_distribution<> elevation_dis(-M_PI, M_PI);
  double r, theta, phi;
  Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
  for(int i=0;i<chunk_size;i++){
    r = r_dis(gen);
    theta = azimuth_dis(gen);
    phi = elevation_dis(gen);
    tempPoint.x = float(end_point_center.x + r * cos(phi)*cos(theta));
    tempPoint.y = float(end_point_center.y + r * cos(phi)*sin(theta));
    tempPoint.z = float(end_point_center.z + r * sin(phi));
    shooting_points_sub.push_back(tempPoint);
  }
}
void TrajectoryPlanner3D::ComputePrimitives() {
  primitives_list_.clear();
  int num_chunk = num_sample_/num_thread_;
  vector<thread> worker_thread;
  PrimitiveListSet primitive_list_temp;
  for(int i = 0; i<num_thread_;i++){
    worker_thread.emplace_back(&TrajectoryPlanner3D::ComputePrimitivesSubProcess,this,num_chunk*(i),num_chunk*(i+1),std::ref(primitive_list_temp[i]));
  }
  for(int i = 0;i<num_thread_;i++){
    worker_thread[i].join();
  }
  for(int i =0;i<num_thread_;i++){
    for(int j =0;j<primitive_list_temp[i].size();j++){
      primitives_list_.push_back(primitive_list_temp[i][j]);
    }
  }
}
void TrajectoryPlanner3D::ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                      PrimitiveList &primitive_list_sub) {
  StatePoly primitive_temp;
  primitive_temp.SetDegree(5);
  float time_interval_temp[2]{0.0, planning_horizon_};
  primitive_temp.SetTimeInterval(time_interval_temp);
  float bernstein_coeff_temp[6];
  float planning_horizon_square = planning_horizon_*planning_horizon_;
  primitive_temp.rx = rx_;
  primitive_temp.ry = ry_;
  primitive_temp.rz = rz_;
  for(int i = start_idx; i<end_idx;i++){
    { // x-component
      bernstein_coeff_temp[0] = drone_state_.px;
      bernstein_coeff_temp[1] = drone_state_.px + 0.2f*planning_horizon_*drone_state_.vx;
      bernstein_coeff_temp[2] = drone_state_.px + 0.4f*planning_horizon_*drone_state_.vx + 0.05f*planning_horizon_square*drone_state_.ax;
      bernstein_coeff_temp[3] = 0.16666667f*shooting_points_[i].x + 0.83333333f*drone_state_.px + 0.43333333f*planning_horizon_*drone_state_.vx + 0.06666667f*planning_horizon_square*drone_state_.ax;
      bernstein_coeff_temp[4] = 0.5f*shooting_points_[i].x + 0.5f*drone_state_.px + 0.3f*planning_horizon_*drone_state_.vx + 0.05f*planning_horizon_square*drone_state_.ax;
      bernstein_coeff_temp[5] = shooting_points_[i].x;
      primitive_temp.px.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // y-component
      bernstein_coeff_temp[0] = drone_state_.py;
      bernstein_coeff_temp[1] = drone_state_.py + 0.2f*planning_horizon_*drone_state_.vy;
      bernstein_coeff_temp[2] = drone_state_.py + 0.4f*planning_horizon_*drone_state_.vy + 0.05f*planning_horizon_square*drone_state_.ay;
      bernstein_coeff_temp[3] = 0.16666667f*shooting_points_[i].y + 0.83333333f*drone_state_.py + 0.43333333f*planning_horizon_*drone_state_.vy + 0.06666667f*planning_horizon_square*drone_state_.ay;
      bernstein_coeff_temp[4] = 0.5f*shooting_points_[i].y + 0.5f*drone_state_.py + 0.3f*planning_horizon_*drone_state_.vy + 0.05f*planning_horizon_square*drone_state_.ay;
      bernstein_coeff_temp[5] = shooting_points_[i].y;
      primitive_temp.py.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    { // z-component
      bernstein_coeff_temp[0] = drone_state_.pz;
      bernstein_coeff_temp[1] = drone_state_.pz + 0.2f*planning_horizon_*drone_state_.vz;
      bernstein_coeff_temp[2] = drone_state_.pz + 0.4f*planning_horizon_*drone_state_.vz + 0.05f*planning_horizon_square*drone_state_.az;
      bernstein_coeff_temp[3] = 0.16666667f*shooting_points_[i].z + 0.83333333f*drone_state_.pz + 0.43333333f*planning_horizon_*drone_state_.vz + 0.06666667f*planning_horizon_square*drone_state_.az;
      bernstein_coeff_temp[4] = 0.5f*shooting_points_[i].z + 0.5f*drone_state_.pz + 0.3f*planning_horizon_*drone_state_.vz + 0.05f*planning_horizon_square*drone_state_.az;
      bernstein_coeff_temp[5] = shooting_points_[i].z;
      primitive_temp.pz.SetBernsteinCoeff(bernstein_coeff_temp);
    }
    primitive_list_sub.push_back(primitive_temp);
  }
}
