//
// Created by larr-planning on 23. 5. 16.
//

#ifndef HEADER_TRAJECTORY_PLANNER
#define HEADER_TRAJECTORY_PLANNER
#include "los_keeper/math_utils/eigenmvn.h"
#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/type_manager/type_manager.h"

namespace los_keeper {
class TrajectoryPlanner {
private:
public:
  TrajectoryPlanner();
  std::optional<StatePoly> ComputeChasingTrajectory(
      const std::vector<StatePoly> &target_prediction_list,
      const PclPointCloud &obstacle_points,
      const std::vector<StatePoly> &structured_obstacle_poly_list) const;
};
} // namespace los_keeper

#endif /* HEADER_TRAJECTORY_PLANNER */
