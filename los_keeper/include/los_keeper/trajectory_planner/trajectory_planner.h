//
// Created by larr-planning on 23. 5. 16.
//

#ifndef HEADER_TRAJECTORY_PLANNER
#define HEADER_TRAJECTORY_PLANNER
#include "los_keeper/math_utils/eigenmvn.h"
#include "los_keeper/type_manager/type_manager.h"
#include "los_keeper/obstacle_manager/obstacle_manager.h"

namespace los_keeper {
class TrajectoryPlanner {
private:
protected:
  // INGREDIENT
  vector<StatePoly> structured_obstacle_poly_list_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  PrimitiveList target_trajectory_list_;

  int num_target_;

  // PARAMETER
  int num_sample_;
  int num_thread_;
  float target_distance_min;
  float target_distance_max;
  float planning_horizon_;
  float acc_max_;
  bool is_2d_;
  float detect_range_;
  float virtual_pcl_zone_width_;
  float virtual_pcl_zone_height_;

  PointList shooting_points_;
  PrimitiveList primitives_list_;

  // RefinePcl
  // SampleShootingPoints
  // GeneratePrimitives

  // TargetDistance
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

public:
  TrajectoryPlanner();
  std::optional<StatePoly> ComputeChasingTrajectory(
      const std::vector<StatePoly> &target_prediction_list,
      const PclPointCloud &obstacle_points,
      const std::vector<StatePoly> &structured_obstacle_poly_list) const;
  void SetTargetState(const PrimitiveList &target_trajectory_list);
  void SetObstacleState(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                        const PrimitiveList &structured_obstacle_poly_list);
  virtual bool PlanKeeperTrajectory() = 0;
};

class TrajectoryPlanner2D: public TrajectoryPlanner{
private:
  void SampleShootingPoints() override;
  void SampleShootingPointsSubProcess(const int &target_id, const int &chunk_size, PointList & shooting_points_sub) override;
  void ComputePrimitives() override;
  void ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx, PrimitiveList &primitive_list_sub) override;
public:
  bool PlanKeeperTrajectory() override;
};

class TrajectoryPlanner3D: public TrajectoryPlanner{
private:
  void SampleShootingPoints() override;
  void SampleShootingPointsSubProcess(const int &target_id, const int &chunk_size, PointList & shooting_points_sub) override;
  void ComputePrimitives() override;
  void ComputePrimitivesSubProcess(const int &start_idx, const int &end_idx, PrimitiveList &primitive_list_sub) override;
public:
  bool PlanKeeperTrajectory() override;
};



}
#endif /* HEADER_TRAJECTORY_PLANNER */
