#ifndef HEADER_OBSTACLE_MANAGER
#define HEADER_OBSTACLE_MANAGER

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "los_keeper/type_manager/type_manager.h"
#include <vector>

namespace los_keeper {
class ObstacleManager {
private:
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  std::vector<ObjectState> structured_obstacle_state_list_;
  std::vector<StatePoly> structured_obstacle_poly_list_;
  float planning_horizon_;

  Eigen::Affine3d pose_;
  std::string name_{"ObstacleManager"};

public:
  std::string GetName() const;
  void SetObstacleInformation(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::vector<ObjectState>& structured_obstacle_state_list);
  void TranslateStateToPoly();
  std::vector<StatePoly> GetStructuredObstaclePolyList(){return structured_obstacle_poly_list_;};
  pcl::PointCloud<pcl::PointXYZ> GetPcl(){return cloud_;};

};
} // namespace los_keeper
#endif /* HEADER_OBSTACLE_MANAGER */
