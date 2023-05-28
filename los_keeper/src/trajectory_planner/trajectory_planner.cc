#include "los_keeper/trajectory_planner/trajectory_planner.h"
using namespace los_keeper;
TrajectoryPlanner::TrajectoryPlanner(){};
std::optional<StatePoly> TrajectoryPlanner::ComputeChasingTrajectory(
    const std::vector<StatePoly> &target_prediction_list,
    const los_keeper::PclPointCloud &obstacle_points,
    const std::vector<StatePoly> &structured_obstacle_poly_list) const {
  return std::optional<StatePoly>();
}