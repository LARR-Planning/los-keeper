#ifndef HEADER_WRAPPER
#define HEADER_WRAPPER

#include <string>

#include "obstacle_manager/obstacle_manager.h"
#include "target_manager/target_manager.h"

namespace los_keeper {
class Wrapper {
private:
  std::string name_{"Wrapper"};
  ObstacleManager obstacle_manager_;
  TargetManager target_manager_;

public:
  bool Plan() const;
};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
