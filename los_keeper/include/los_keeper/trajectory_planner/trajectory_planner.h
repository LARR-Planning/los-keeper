//
// Created by larr-planning on 23. 5. 16.
//

#ifndef HEADER_TRAJECTORY_PLANNER
#define HEADER_TRAJECTORY_PLANNER
#include "los_keeper/math_utils/eigenmvn.h"
#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/type_manager/type_manager.h"
using namespace std;
namespace los_keeper {
class TrajectoryPlanner {
private:
protected:
  // INGREDIENT
  vector<StatePoly> structured_obstacle_poly_list_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  PrimitiveList target_trajectory_list_;
  int num_target_;
  DroneState drone_state_;

  // PARAMETER
  PlanningParameter param_;

  PointList shooting_points_;
  PrimitiveList primitives_list_;
  IndexList good_target_distance_index_list_;

  // RefinePcl
  // SampleShootingPoints Done
  // GeneratePrimitives Done

  // TargetDistance Done
  // CheckCollision
  // CheckDynamicFeasibility
  // CheckFovLimit
  // CalculateBestIndex

  virtual void SampleShootingPoints();
  virtual void SampleShootingPointsSubProcess(const int &target_id, const int &chunk_size,
                                              PointList &shooting_points_sub);
  virtual void ComputePrimitives();

  virtual void ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                           PrimitiveList &primitive_list_sub);
  virtual void CheckDistanceFromTargets();
  virtual void CheckDistanceFromTargetsSubProcess(const int &start_idx, const int &end_idx,
                                                  IndexList &dist_idx_sub);

  void SetTargetState(const PrimitiveList &target_trajectory_list);
  void SetObstacleState(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                        const PrimitiveList &structured_obstacle_poly_list);
  virtual bool PlanKeeperTrajectory() = 0;
  StatePoly GetBestKeeperTrajectory();

public:
  TrajectoryPlanner() = default;
  explicit TrajectoryPlanner(const PlanningParameter &param);
  virtual optional<StatePoly>
  ComputeChasingTrajectory(const vector<StatePoly> &target_prediction_list,
                           const PclPointCloud &obstacle_points,
                           const vector<StatePoly> &structured_obstacle_poly_list) = 0;
};

class TrajectoryPlanner2D : public TrajectoryPlanner {
private:
  void SampleShootingPoints() override;
  void SampleShootingPointsSubProcess(const int &target_id, const int &chunk_size,
                                      PointList &shooting_points_sub) override;
  void ComputePrimitives() override;
  void ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                   PrimitiveList &primitive_list_sub) override;
  void CheckDistanceFromTargets() override;
  void CheckDistanceFromTargetsSubProcess(const int &start_idx, const int &end_idx,
                                          IndexList &dist_idx_sub) override;
  bool PlanKeeperTrajectory() override;

public:
  TrajectoryPlanner2D() = default;
  explicit TrajectoryPlanner2D(const PlanningParameter &param);
  optional<StatePoly>
  ComputeChasingTrajectory(const vector<StatePoly> &target_prediction_list,
                           const PclPointCloud &obstacle_points,
                           const vector<StatePoly> &structured_obstacle_poly_list) override;
};

class TrajectoryPlanner3D : public TrajectoryPlanner {
private:
  void SampleShootingPoints() override;
  void SampleShootingPointsSubProcess(const int &target_id, const int &chunk_size,
                                      PointList &shooting_points_sub) override;
  void ComputePrimitives() override;
  void ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                   PrimitiveList &primitive_list_sub) override;
  void CheckDistanceFromTargets() override;
  void CheckDistanceFromTargetsSubProcess(const int &start_idx, const int &end_idx,
                                          IndexList &dist_idx_sub) override;
  bool PlanKeeperTrajectory() override;

public:
  TrajectoryPlanner3D() = default;
  explicit TrajectoryPlanner3D(const PlanningParameter &param);
  optional<StatePoly>
  ComputeChasingTrajectory(const vector<StatePoly> &target_prediction_list,
                           const PclPointCloud &obstacle_points,
                           const vector<StatePoly> &structured_obstacle_poly_list) override;
};

} // namespace los_keeper
#endif /* HEADER_TRAJECTORY_PLANNER */
