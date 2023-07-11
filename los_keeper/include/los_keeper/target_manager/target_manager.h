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
struct TargetManagerDebugInfo {
  int num_target{-1};
  bool success_flag{true};
  //  PointListSet end_points{};
  PrimitiveListSet primitives_list;
  IndexListSet primitive_safe_total_index;
  IndexList primitive_best_index;
};
class TargetManager {
private:
protected:
  // INGREDIENT
  vector<StatePoly> structured_obstacle_poly_list_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  vector<ObjectState> target_state_list_;
  int num_target_{};
  PredictionParameter param_;

  PointListSet end_points_;               // Sampled End Points from Dynamics Model //CLEAR:
  PrimitiveListSet primitives_list_;      // Raw primitives from //CLEAR: OK
  IndexListSet close_obstacle_index_;     // CLEAR: OK
  IndexListSet primitive_safe_pcl_index_; // CLEAR: OK
  IndexListSet primitive_safe_structured_obstacle_index_; // CLEAR:
  IndexListSet primitive_safe_total_index_;               // CLEAR:
  IndexList primitive_best_index_;                        // CLEAR: OK

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
  virtual bool CheckStructuredObstacleCollision() = 0;
  virtual void CheckStructuredObstacleCollisionSubProcess(const int &target_id,
                                                          const int &start_idx, const int &end_idx,
                                                          IndexList &safe_structured_index_sub);
  virtual void CalculateCentroid();
  virtual void CalculateCentroidSubProcess(const int &target_id, const int &start_idx,
                                           const int &end_idx, pair<int, float> &min_dist);
  PrimitiveList GetTargetPredictionResult();
  void SetTargetState(const vector<ObjectState> &target_state_list);
  void SetObstacleState(pcl::PointCloud<pcl::PointXYZ> cloud,
                        const PrimitiveList &structured_obstacle_poly_list);

public:
  TargetManager();
  explicit TargetManager(const PredictionParameter &param) : param_(param){};
  virtual std::optional<std::vector<StatePoly>>
  PredictTargetList(const std::vector<ObjectState> &target_state_list,
                    const PclPointCloud &point_cloud,
                    const PrimitiveList &structured_obstacle_poly_list) = 0;
  TargetManagerDebugInfo GetDebugInfo() const;
};

class TargetManager2D : public los_keeper::TargetManager {
  friend class ApiTestFixtureTargetManager2D;
  FRIEND_TEST(ApiTestFixtureTargetManager2D, CheckWhenNoObstacles2D);

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
  bool CheckStructuredObstacleCollision() override;
  void CheckStructuredObstacleCollisionSubProcess(const int &target_id, const int &start_idx,
                                                  const int &end_idx,
                                                  IndexList &safe_structured_index_sub) override;
  void CalculateCentroid() override;
  void CalculateCentroidSubProcess(const int &target_id, const int &start_idx, const int &end_idx,
                                   pair<int, float> &min_dist) override;

  void CalculateSafePclIndex(const vector<LinearConstraint2D> &safe_corridor_list);
  bool PredictTargetTrajectory() override;

public:
  TargetManager2D() = default;
  explicit TargetManager2D(const PredictionParameter &param);

  std::optional<std::vector<StatePoly>>
  PredictTargetList(const std::vector<ObjectState> &target_state_list,
                    const los_keeper::PclPointCloud &point_cloud,
                    const PrimitiveList &structured_obstacle_poly_list) override;
};
class TargetManager3D : public los_keeper::TargetManager {
  friend class ApiTestFixtureTargetManager3D;
  FRIEND_TEST(ApiTestFixtureTargetManager3D, CheckWhenNoObstacles3D);

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
  bool CheckStructuredObstacleCollision() override;
  void CheckStructuredObstacleCollisionSubProcess(const int &target_id, const int &start_idx,
                                                  const int &end_idx,
                                                  IndexList &safe_structured_index_sub) override;
  void CalculateCentroid() override;
  void CalculateCentroidSubProcess(const int &target_id, const int &start_idx, const int &end_idx,
                                   pair<int, float> &min_dist) override;
  void CalculateSafePclIndex(const vector<LinearConstraint3D> &safe_corridor_list);
  bool PredictTargetTrajectory() override;

public:
  TargetManager3D() = default;
  explicit TargetManager3D(const PredictionParameter &param);

  std::optional<std::vector<StatePoly>>
  PredictTargetList(const std::vector<ObjectState> &target_state_list,
                    const PclPointCloud &point_cloud,
                    const PrimitiveList &structured_obstacle_poly_list) override;
};
}; // namespace los_keeper

#endif /* HEADER_TARGET_MANAGER */
