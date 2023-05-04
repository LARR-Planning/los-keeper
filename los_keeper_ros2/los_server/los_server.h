#ifndef HEADER_LOS_SERVER
#define HEADER_LOS_SERVER
#include <los_keeper/obstacle_manager/obstacle_manager.h>
namespace los_keeper {
class LosServer {
private:
  ObstacleManager obstacle_manager_;

public:
  LosServer();
};

} // namespace los_keeper
#endif /* HEADER_LOS_SERVER */
