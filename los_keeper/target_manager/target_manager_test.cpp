#include "obstacle_manager/ObstacleManager.hpp"
#include "target_manager/TargetManager.hpp"
#include "gtest/gtest.h"
#include <Eigen/Core>

namespace los_keeper {

TEST(TargetTest, NameShouldCorrect) {

  ObstacleManager obstacle_manager;
  EXPECT_EQ(obstacle_manager.GetName(), "ObstacleManager");

  TargetManager target_manager;
  EXPECT_EQ(target_manager.CheckCollision(obstacle_manager), true);
}

} // namespace los_keeper