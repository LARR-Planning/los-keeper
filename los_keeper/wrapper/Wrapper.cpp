#include "Wrapper.hpp"

bool los_keeper::Wrapper::Plan() const {
  return target_manager_.GetName() == "TargetManager" &&
         obstacle_manager_.GetName() == "ObstacleManager";
}
