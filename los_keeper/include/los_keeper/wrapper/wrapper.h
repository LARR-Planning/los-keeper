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
  store::State state_;

  struct {
    std::mutex drone_state;
    std::mutex pointcloud;
    std::mutex control;
  } mutex_list_;

  std::shared_ptr<ObstacleManager> obstacle_manager_;
  std::shared_ptr<TargetManager> target_manager_;
  std::shared_ptr<TrajectoryPlanner> trajectory_planner_;

  store::State HandleUpdateMonitorAction(store::State &previous_state) const;
  store::State HandleInitializeAction(store::State &previous_state) const;
  store::State HandleReplanAction(store::State &previous_state) const;

public:
  Wrapper();

  void SetPoints(const pcl::PointCloud<pcl::PointXYZ> &points);
  void SetDroneState(const DroneState &drone_state);
  int GetControlInput(double time) const; // TODO(Lee): change to jerk input

  void OnPlanningTimerCallback();
  void OnStartServiceCallback();
};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
