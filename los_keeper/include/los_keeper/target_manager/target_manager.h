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
using namespace std;

namespace los_keeper {


class TargetManager {
private:
protected:
  // INGREDIENT
  vector<StatePoly> structured_obstacle_poly_list_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  vector<ObjectState> target_state_list_;
  int num_target_;

  // PARAMETER
  int num_sample_;
  int num_thread_;
  float planning_horizon_;
  float acc_max_;
  bool is_2d_;
  float detect_range_;
  float virtual_pcl_zone_width_;
  float virtual_pcl_zone_height_;
  bool is_lite;

  string name_{"TargetManager"};

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
  virtual void SampleEndPointsSubProcess(const int &target_id, const int &chunk_size,
                                         PointList &endpoint_sub);
  virtual void ComputePrimitives();
  virtual void ComputePrimitivesSubProcess(const int &target_id, const int &start_idx,
                                           const int &end_idx, PrimitiveList &primitive_list_sub);
  virtual void CalculateCloseObstacleIndex(); // Return true if at least one non-colliding
                                              // target trajectory exists
  virtual bool CheckCollision() = 0;
  virtual void CheckPclCollision();
  virtual void CheckStructuredObstacleCollision();
  virtual void CheckStructuredObstacleCollisionSubProcess(const int &target_id,
                                                          const int &start_idx, const int &end_idx,
                                                          IndexList &safe_structured_index_sub);
  virtual void CalculateCentroid();
  virtual void CalculateCentroidSubProcess(const int &target_id, const int &start_idx,
                                           const int &end_idx, pair<int, float> &min_dist);

public:
  TargetManager();
  string GetName() const;
  bool CheckCollision(const ObstacleManager &obstacle_manager) const;
  void SetTargetState(const vector<ObjectState> &target_state_list);
  void SetObstacleState(pcl::PointCloud<pcl::PointXYZ> cloud,
                        const PrimitiveList &structured_obstacle_poly_list);
  virtual std::optional<std::vector<StatePoly>> PredictTargetList(
      const std::vector<ObjectState> &target_state_list,
      const PclPointCloud &point_cloud,
      const std::vector<StatePoly> &structured_obstacle_poly_list) = 0;
};


class TargetManager2D : public los_keeper::TargetManager {
private:
  vector<LinearConstraint2D> GenLinearConstraint();
  vec_E<Polyhedron2D> polys;
  void SampleEndPoints() override;
  void SampleEndPointsSubProcess(const int &target_id, const int &chunk_size,
                                 PointList &endpoint_sub) override;
  void ComputePrimitives() override;
  void ComputePrimitivesSubProcess(const int &target_id, const int &start_idx, const int &end_idx,
                                   PrimitiveList &primitive_list_sub) override;
  void CalculateCloseObstacleIndex() override;
  bool CheckCollision() override;
  void CheckPclCollision() override;
  void CheckPclCollisionSubProcess(const int &target_id, const LinearConstraint2D &constraints,
                                   const int &start_idx, const int &end_idx,
                                   IndexList &safe_pcl_index_sub);
  void CheckStructuredObstacleCollision() override;
  void CheckStructuredObstacleCollisionSubProcess(const int &target_id, const int &start_idx,
                                                  const int &end_idx,
                                                  IndexList &safe_structured_index_sub) override;
  void CalculateCentroid() override;
  void CalculateCentroidSubProcess(const int &target_id, const int &start_idx, const int &end_idx,
                                   pair<int, float> &min_dist) override;

  void CalculateSafePclIndex(const vector<LinearConstraint2D> &safe_corridor_list);

public:
  bool PredictTargetTrajectory() override;
  std::optional<std::vector<StatePoly>> PredictTargetList(
      const std::vector<ObjectState> &target_state_list,
      const los_keeper::PclPointCloud &point_cloud,
      const std::vector<StatePoly> &structured_obstacle_poly_list) override;
};
class TargetManager3D : public los_keeper::TargetManager {
private:
  vector<LinearConstraint3D> GenLinearConstraint();
  vec_E<Polyhedron3D> polys;
  void SampleEndPoints() override;
  void SampleEndPointsSubProcess(const int &target_id, const int &chunk_size,
                                 PointList &endpoint_sub) override;
  void ComputePrimitives() override;
  void ComputePrimitivesSubProcess(const int &target_id, const int &start_idx, const int &end_idx,
                                   PrimitiveList &primitive_list_sub) override;
  void CalculateCloseObstacleIndex() override;
  bool CheckCollision() override;
  void CheckPclCollision() override;
  void CheckPclCollisionSubProcess(const int &target_id, const LinearConstraint3D &constraints,
                                   const int &start_idx, const int &end_idx,
                                   IndexList &safe_pcl_index_sub);
  void CheckStructuredObstacleCollision() override;
  void CheckStructuredObstacleCollisionSubProcess(const int &target_id, const int &start_idx,
                                                  const int &end_idx,
                                                  IndexList &safe_structured_index_sub) override;
  void CalculateCentroid() override;
  void CalculateCentroidSubProcess(const int &target_id, const int &start_idx, const int &end_idx,
                                   pair<int, float> &min_dist) override;
  void CalculateSafePclIndex(const vector<LinearConstraint3D> &safe_corridor_list);

public:
  bool PredictTargetTrajectory() override;
  std::optional<std::vector<StatePoly>> PredictTargetList(
      const std::vector<ObjectState> &target_state_list,
      const PclPointCloud &point_cloud,
      const std::vector<StatePoly> &structured_obstacle_poly_list) override;
};
};

#endif /* HEADER_TARGET_MANAGER */
