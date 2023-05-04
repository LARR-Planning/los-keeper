#ifndef HEADER_LOS_SERVER
#define HEADER_LOS_SERVER
#include <los_keeper/wrapper/wrapper.h>
#include <rclcpp/rclcpp.hpp>

namespace los_keeper {
class LosServer {
private:
  Wrapper wrapper_;

public:
  LosServer();
  bool Update();
};

} // namespace los_keeper
#endif /* HEADER_LOS_SERVER */
