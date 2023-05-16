#ifndef HEADER_TARGET_MANAGER
#define HEADER_TARGET_MANAGER
#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/type_manager/type_manager.h"
#include "los_keeper/math_utils/eigenmvn.h"
#include <Eigen/Core>
#include <string>

namespace los_keeper {

class TargetManager {
private:
  // INGREDIENT
  std::vector<StatePoly> structured_obstacle_poly_list_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  std::vector<ObjectState> target_state_list_;
  int num_target_;

  // PARAMETER
  int num_sample_;
  float planning_horizon_;
  float acc_max_;
  bool is_2d_;

  std::string name_{"TargetManager"};

  std::vector<std::vector<Point>> end_points_;
  std::vector<std::vector<StatePoly>> primitives_list_;

  // FUNCTION
  void PredictTargetTrajectory();
  void SampleEndPoints();
  void ComputePrimitives();
  void CalculateCloseObstacleIndex();
  void CheckCollision();
  void CalculateCentroid();

public:
  TargetManager(); //TODO: Make classes TargetManager2D and TargetManager3D which inherit TargetManager
  std::string GetName() const;
  bool CheckCollision(const ObstacleManager &obstacle_manager) const;
  void SetTargetState(const std::vector<ObjectState> & target_state_list);
  void SetObstacleState(pcl::PointCloud<pcl::PointXYZ> cloud, std::vector<StatePoly> structured_obstacle_poly_list);

};
} // namespace los_keeper

#endif /* HEADER_TARGET_MANAGER */
