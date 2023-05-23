#ifndef HEADER_WRAPPER
#define HEADER_WRAPPER

#include <chrono>
#include <string>
#include <thread>

#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/target_manager/target_manager.h"
#include "los_keeper/trajectory_planner/trajectory_planner.h"
#include "los_keeper/type_manager/type_manager.h"

namespace los_keeper {
class Wrapper {
private:
  std::string name_{"Wrapper"};
  ObstacleManager obstacle_manager_;
  TargetManager *target_manager_;
  TrajectoryPlanner trajectory_planner_;

  std::string long_string_;
  std::string short_string_;

public:
  Wrapper();
  bool Plan() const;
  void
  SetProblem(const pcl::PointCloud<pcl::PointXYZ> &cloud,
             const std::vector<ObjectState> &structured_obstacle_state_list,
             const std::vector<ObjectState> &target_state_list);
  void SetLongString(const std::string &long_string);
  void SetShortString(const std::string &short_string);
  std::string GetConcatString() const;
};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
