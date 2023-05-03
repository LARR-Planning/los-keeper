#ifndef HEADER_TARGETMANAGER
#define HEADER_TARGETMANAGER
#include "obstacle_manager/ObstacleManager.hpp"
#include <Eigen/Core>
#include <string>

namespace los_keeper {

class TargetManager {
private:
  std::string name_{"TargetManager"};

public:
  std::string GetName() const;
  bool CheckCollision(const ObstacleManager &obstacle_manager) const;
};
} // namespace los_keeper

#endif /* HEADER_TARGETMANAGER */
