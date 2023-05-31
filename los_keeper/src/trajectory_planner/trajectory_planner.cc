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


}
void TrajectoryPlanner2D::ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                      PrimitiveList &primitive_list_sub) {

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

}
void TrajectoryPlanner3D::ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                      PrimitiveList &primitive_list_sub) {

}
