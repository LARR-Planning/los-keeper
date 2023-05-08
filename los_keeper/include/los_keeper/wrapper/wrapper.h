#ifndef HEADER_WRAPPER
#define HEADER_WRAPPER

#include <string>

#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/target_manager/target_manager.h"

namespace los_keeper {
class Wrapper {
private:
  std::string name_{"Wrapper"};
  ObstacleManager obstacle_manager_;
  TargetManager target_manager_;
  std::string long_string_;
  std::string short_string_;

public:
  bool Plan() const;
  void SetLongString(const std::string &long_string);
  void SetShortString(const std::string &short_string);
  std::string GetConcatString() const;
};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
