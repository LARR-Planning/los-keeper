#ifndef HEADER_TARGET_MANAGER
#define HEADER_TARGET_MANAGER
#include "los_keeper/math_utils/eigenmvn.h"
#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/third_party/decomp_util/decomp_util/decomp_base.h"
#include "los_keeper/third_party/decomp_util/decomp_util/ellipsoid_decomp.h"
#include "los_keeper/third_party/decomp_util/decomp_util/seed_decomp.h"
#include "los_keeper/type_manager/type_manager.h"

#include <Eigen/Core>
#include <string>

namespace los_keeper {
typedef std::vector<int> IndexList;
typedef std::vector<IndexList> IndexListSet;
typedef std::vector<Point> PointList;
typedef std::vector<PointList> PointListSet;
typedef std::vector<StatePoly> PrimitiveList;
typedef std::vector<PrimitiveList> PrimitiveListSet;

class TargetManager {
private:
protected:
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
  float detect_range_;
  float virtual_pcl_zone_width_;
  float virtual_pcl_zone_height_;
  bool is_lite;

  std::string name_{"TargetManager"};

  PointListSet end_points_;          // Sampled End Points from Dynamics Model
  PrimitiveListSet primitives_list_; // Raw primitives from
  IndexListSet close_obstacle_index_;
  IndexListSet primitive_safe_pcl_index_;
  IndexListSet primitive_safe_structured_obstacle_index_;
  IndexListSet primitive_safe_total_index_;
  IndexList primitive_best_index_;

  // FUNCTION
  virtual bool PredictTargetTrajectory() = 0; // Return true if at least one possible target
                                              // trajectory exists
  virtual void SampleEndPoints();
  virtual void ComputePrimitives();
  virtual void CalculateCloseObstacleIndex(); // Return true if at least one non-colliding
                                              // target trajectory exists
  virtual bool CheckCollision() = 0;
  virtual void CheckPclCollision();
  virtual void CheckStructuredObstacleCollision();
  virtual void CalculateCentroid();

public:
  TargetManager();
  std::string GetName() const;
  bool CheckCollision(const ObstacleManager &obstacle_manager) const;
  void SetTargetState(const std::vector<ObjectState> &target_state_list);
  void SetObstacleState(pcl::PointCloud<pcl::PointXYZ> cloud,
                        const PrimitiveList &structured_obstacle_poly_list);
};

class TargetManager2D : public TargetManager {
private:
  std::vector<LinearConstraint2D> GenLinearConstraint();
  vec_E<Polyhedron2D> polys;
  void SampleEndPoints() override;
  void ComputePrimitives() override;
  void CalculateCloseObstacleIndex() override;
  bool CheckCollision() override;
  void CheckPclCollision() override;
  void CheckStructuredObstacleCollision() override;
  void CalculateCentroid() override;
  void CalculateSafePclIndex(
      const std::vector<LinearConstraint2D> &safe_corridor_list);

public:
  bool PredictTargetTrajectory() override;
};
class TargetManager3D : public TargetManager {
private:
  std::vector<LinearConstraint3D> GenLinearConstraint();
  vec_E<Polyhedron3D> polys;
  void SampleEndPoints() override;
  void ComputePrimitives() override;
  void CalculateCloseObstacleIndex() override;
  bool CheckCollision() override;
  void CheckPclCollision() override;
  void CheckStructuredObstacleCollision() override;
  void CalculateCentroid() override;
  void CalculateSafePclIndex(
      const std::vector<LinearConstraint3D> &safe_corridor_list);

public:
  bool PredictTargetTrajectory() override;
};

} // namespace los_keeper

#endif /* HEADER_TARGET_MANAGER */
