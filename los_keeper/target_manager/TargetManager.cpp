#include "target_manager/TargetManager.hpp"

bool los_keeper::TargetManager::CheckCollision(
    const ObstacleManager &obstacle_manager) const {
  return obstacle_manager.GetName() == "ObstacleManager";
}
std::string los_keeper::TargetManager::GetName() const { return name_; }