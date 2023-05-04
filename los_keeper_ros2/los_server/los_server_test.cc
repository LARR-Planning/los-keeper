#include "los_server/los_server.h"
#include "gtest/gtest.h"
#include <Eigen/Core>

namespace los_keeper {

TEST(LosServerTest, IsObstacleManagerIncludedAutomatically) {

  ObstacleManager obstacle_manager;
  EXPECT_EQ(obstacle_manager.GetName(), "ObstacleManager");
}

} // namespace los_keeper