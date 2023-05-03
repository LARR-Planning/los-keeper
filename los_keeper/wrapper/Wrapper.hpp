#ifndef HEADER_WRAPPER
#define HEADER_WRAPPER

#include <string>

#include <chasing_planner/ChasingPlanner.hpp>
#include <obstacle_manager/ObstacleManager.hpp>
#include <target_manager/TargetManager.hpp>

namespace los_keeper {
class Wrapper {
private:
  std::string name_{"Wrapper"};
  ObstacleManager obstacle_manager_;
  TargetManager target_manager_;
  ChasingPlanner chasing_planner_;

public:
  std::string GetName() const;
};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
