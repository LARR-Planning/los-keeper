#ifndef HEADER_OBSTACLE_MANAGER
#define HEADER_OBSTACLE_MANAGER

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "los_keeper/type_manager/type_manager.h"
#include "gtest/gtest.h"
#include <vector>

namespace los_keeper {
class ObstacleManager {
  friend class ApiTestFixtureObstacle;
  FRIEND_TEST(ApiTestFixtureObstacle, CheckCollisionBetweenTrajectoryAndObstacles);
  FRIEND_TEST(ApiTestFixtureObstacle, CheckCollisionBetweenPointsAndObstacles);

private:
  PclPointCloud cloud_;
  std::vector<ObjectState> structured_obstacle_state_list_;
  std::vector<StatePoly> structured_obstacle_poly_list_;
  ObstacleParameter param_;

  Eigen::Affine3d pose_;
  void TranslateStateToPoly();

public:
  ObstacleManager() = default;
  explicit ObstacleManager(const ObstacleParameter &param);
  void SetObstacleCloud(const PclPointCloud &cloud);
  void SetStructuredObstacleState(const std::vector<ObjectState> &structured_obstacle_state_list);
  std::vector<StatePoly> GetStructuredObstaclePolyList() { return structured_obstacle_poly_list_; };
  PclPointCloud GetPointCloud();
  bool CheckCollisionAlongTrajectory(const StatePoly &trajectory);
  bool CheckCollisionWithPoint(const DroneState &drone_state);
};
} // namespace los_keeper
#endif /* HEADER_OBSTACLE_MANAGER */
