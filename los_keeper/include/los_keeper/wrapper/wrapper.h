#ifndef HEADER_WRAPPER
#define HEADER_WRAPPER

#include <chrono>
#include <string>
#include <thread>

#include "los_keeper/wrapper/store.h"
#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/target_manager/target_manager.h"
#include "los_keeper/trajectory_planner/trajectory_planner.h"
#include "los_keeper/type_manager/type_manager.h"

namespace los_keeper {

class Wrapper {
 private:
  State state_;

  std::string name_{"Wrapper"};
  std::shared_ptr<ObstacleManager> obstacle_manager_;
  std::shared_ptr<TargetManager> target_manager_;
  std::shared_ptr<TrajectoryPlanner> trajectory_planner_;

  bool Plan() const;
  void
  SetProblem(const pcl::PointCloud<pcl::PointXYZ> &cloud,
             const std::vector<ObjectState> &structured_obstacle_state_list,
             const std::vector<ObjectState> &target_state_list);

  void ApplyStartAction(State &state);
  void ApplyPauseAction(State &state);
  void ApplyResetAction(State &state);
  void ApplyUpdateAction(State &state);

 public:
  Wrapper();
  void ApplyAction(const Action &action);

};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
