#ifndef HEADER_WRAPPER
#define HEADER_WRAPPER

#include <string>

#include <obstacle_manager/ObstacleManager.hpp>
#include <target_manager/TargetManager.hpp>

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
