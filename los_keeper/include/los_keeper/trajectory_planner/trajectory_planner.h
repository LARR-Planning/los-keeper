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
struct PlanningDebugInfo {
  PrimitiveList primitives_list;
  PointListSet end_points;
  IndexList close_obstacle_index;
  IndexList safe_visibility_index;
  IndexList dynamically_feasible_index;
  IndexList primitive_best_index;
  bool success_flag{false};
  double planning_time{0.0};
};
class TrajectoryPlanner {
private:
protected:
  double planning_time_{0.0};
  int num_target_;
  // PARAMETER
  PlanningParameter param_;

  PointList shooting_points_;
  PrimitiveList primitives_list_;
  IndexList close_obstacle_index_;
  IndexList good_target_distance_index_list_;
  IndexList visible_total_index_;
  IndexList visible_structured_index_;
  IndexList visible_pcl_index_;
  int best_index_{-1};

  // RefinePcl
  // SampleShootingPoints Done
  // GeneratePrimitives Done
  // TargetDistance Done
  // CheckCollision
  // CheckDynamicFeasibility
  // CheckFovLimit
  // CalculateBestIndex

  virtual void SampleShootingPoints(const PrimitiveList &target_prediction_list);
  virtual void SampleShootingPointsSubProcess(const PrimitiveList &target_prediction_list,
                                              const int &target_id, const int &chunk_size,
                                              PointList &shooting_points_sub);
  virtual void ComputePrimitives(const DroneState &drone_state);

  virtual void ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                           const DroneState &drone_state,
                                           PrimitiveList &primitive_list_sub);
  virtual void
  CalculateCloseObstacleIndex(const DroneState &drone_state,
                              const PrimitiveList &structured_obstacle_trajectory_list);
  virtual void CheckDistanceFromTargets(const PrimitiveList &target_trajectory_list);
  virtual void CheckDistanceFromTargetsSubProcess(const int &start_idx, const int &end_idx,
                                                  const PrimitiveList &target_trajectory_list,
                                                  IndexList &dist_idx_sub);
  virtual bool CheckVisibility(const PrimitiveList &target_trajectory_list,
                               const los_keeper::PclPointCloud &cloud,
                               const PrimitiveList &structured_obstacle_poly_list) = 0;
  virtual bool
  CheckVisibilityAgainstStructuredObstacle(const PrimitiveList &structured_obstacle_poly_list,
                                           const PrimitiveList &target_prediction_list) = 0;
  virtual void CheckVisibilityAgainstStructuredObstacleSubProcess(
      const int &start_idx, const int &end_idx, const PrimitiveList &structured_obstacle_poly_list,
      const PrimitiveList &target_prediction_list, IndexList &visible_idx);
  virtual void CheckVisibilityAgainstPcl();
  virtual void CalculateBestIndex();
  virtual void CalculateBestIndexSubProcess(const int &start_idx, const int &end_idx,
                                            pair<int, float> &min_jerk_pair);
  //  void SetTargetState(const PrimitiveList &target_trajectory_list);
  PrimitiveList TranslateTargetPrediction(const PrimitiveList &target_trajectory_list);
  PrimitiveList
  TranslateStructuredObstaclePrediction(const PrimitiveList &structured_obstacle_trajectory_list);
  StatePoly GetBestKeeperTrajectory();

public:
  TrajectoryPlanner() = default;
  explicit TrajectoryPlanner(const PlanningParameter &param);
  virtual optional<StatePoly>
  ComputeChasingTrajectory(const DroneState &drone_state,
                           const vector<StatePoly> &target_prediction_list,
                           const PclPointCloud &obstacle_points,
                           const vector<StatePoly> &structured_obstacle_poly_list) = 0;
  PlanningDebugInfo GetDebugInfo() const;
};

class TrajectoryPlanner2D : public TrajectoryPlanner {
private:
  void SampleShootingPoints(const PrimitiveList &target_prediction_list) override;
  void SampleShootingPointsSubProcess(const PrimitiveList &target_prediction_list,
                                      const int &target_id, const int &chunk_size,
                                      PointList &shooting_points_sub) override;
  void ComputePrimitives(const DroneState &drone_state) override;
  void ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                   const DroneState &drone_state,
                                   PrimitiveList &primitive_list_sub) override;
  void
  CalculateCloseObstacleIndex(const DroneState &drone_state,
                              const PrimitiveList &structured_obstacle_trajectory_list) override;
  void CheckDistanceFromTargets(const PrimitiveList &target_trajectory_list) override;
  void CheckDistanceFromTargetsSubProcess(const int &start_idx, const int &end_idx,
                                          const PrimitiveList &target_trajectory_list,
                                          IndexList &dist_idx_sub) override;
  bool CheckVisibility(const PrimitiveList &target_trajectory_list,
                       const los_keeper::PclPointCloud &cloud,
                       const PrimitiveList &structured_obstacle_poly_list) override;
  bool
  CheckVisibilityAgainstStructuredObstacle(const PrimitiveList &structured_obstacle_poly_list,
                                           const PrimitiveList &target_prediction_list) override;
  void CheckVisibilityAgainstStructuredObstacleSubProcess(
      const int &start_idx, const int &end_idx, const PrimitiveList &structured_obstacle_poly_list,
      const PrimitiveList &target_prediction_list, IndexList &visible_idx) override;
  void CheckVisibilityAgainstPcl() override;
  void CalculateBestIndex() override;
  void CalculateBestIndexSubProcess(const int &start_idx, const int &end_idx,
                                    pair<int, float> &min_jerk_pair) override;

public:
  TrajectoryPlanner2D() = default;
  explicit TrajectoryPlanner2D(const PlanningParameter &param);
  optional<StatePoly>
  ComputeChasingTrajectory(const DroneState &drone_state,
                           const vector<StatePoly> &target_prediction_list,
                           const PclPointCloud &obstacle_points,
                           const vector<StatePoly> &structured_obstacle_poly_list) override;
};

class TrajectoryPlanner3D : public TrajectoryPlanner {
private:
  void SampleShootingPoints(const PrimitiveList &target_prediction_list) override;
  void SampleShootingPointsSubProcess(const PrimitiveList &target_prediction_list,
                                      const int &target_id, const int &chunk_size,
                                      PointList &shooting_points_sub) override;
  void ComputePrimitives(const DroneState &drone_state) override;
  void ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                   const DroneState &drone_state,
                                   PrimitiveList &primitive_list_sub) override;
  void
  CalculateCloseObstacleIndex(const DroneState &drone_state,
                              const PrimitiveList &structured_obstacle_trajectory_list) override;
  void CheckDistanceFromTargets(const PrimitiveList &target_trajectory_list) override;
  void CheckDistanceFromTargetsSubProcess(const int &start_idx, const int &end_idx,
                                          const PrimitiveList &target_trajectory_list,
                                          IndexList &dist_idx_sub) override;
  bool CheckVisibility(const PrimitiveList &target_trajectory_list,
                       const los_keeper::PclPointCloud &cloud,
                       const PrimitiveList &structured_obstacle_poly_list) override;
  bool
  CheckVisibilityAgainstStructuredObstacle(const PrimitiveList &structured_obstacle_poly_list,
                                           const PrimitiveList &target_prediction_list) override;
  void CheckVisibilityAgainstStructuredObstacleSubProcess(
      const int &start_idx, const int &end_idx, const PrimitiveList &structured_obstacle_poly_list,
      const PrimitiveList &target_prediction_list, IndexList &visible_idx) override;
  void CheckVisibilityAgainstPcl() override;
  void CalculateBestIndex() override;
  void CalculateBestIndexSubProcess(const int &start_idx, const int &end_idx,
                                    pair<int, float> &min_jerk_pair) override;

public:
  TrajectoryPlanner3D() = default;
  explicit TrajectoryPlanner3D(const PlanningParameter &param);
  optional<StatePoly>
  ComputeChasingTrajectory(const DroneState &drone_state,
                           const vector<StatePoly> &target_prediction_list,
                           const PclPointCloud &obstacle_points,
                           const vector<StatePoly> &structured_obstacle_poly_list) override;
};

} // namespace los_keeper
#endif /* HEADER_TRAJECTORY_PLANNER */
