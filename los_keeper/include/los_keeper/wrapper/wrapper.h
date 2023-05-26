#ifndef HEADER_WRAPPER
#define HEADER_WRAPPER

#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/target_manager/target_manager.h"
#include "los_keeper/trajectory_planner/trajectory_planner.h"
#include "los_keeper/wrapper/store.h"

namespace los_keeper {

class Wrapper {
private:
  DroneState robot_state_;
  store::State state_;
  PlanningOutput planning_output_;

  struct {
    std::mutex drone_state;
    std::mutex pointcloud;
    std::mutex control;
  } mutex_list_;

  std::shared_ptr<ObstacleManager> obstacle_manager_;
  std::shared_ptr<TargetManager> target_manager_;
  std::shared_ptr<TrajectoryPlanner> trajectory_planner_;

  void UpdateState(store::State &state);

  void HandleStopAction() const;
  void HandleInitializeAction() const;
  void HandleReplanAction() const;

public:
  Wrapper();

  void SetPoints(const pcl::PointCloud<pcl::PointXYZ> &points);
  void SetDroneState(const DroneState &drone_state);
  int GenerateControlInputFromPlanning(
      const PlanningOutput &planning_output,
      double time) const; // TODO(Lee): change to jerk input

  void OnPlanningTimerCallback();
  void OnStartServiceCallback();
};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
