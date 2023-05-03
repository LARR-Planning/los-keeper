#include "obstacle_manager/ObstacleManager.hpp"
#include "gtest/gtest.h"

namespace los_keeper {

TEST(ObstacleTest, NameShouldCorrect) {

  ObstacleManager obstacle_manager;
  EXPECT_EQ(obstacle_manager.GetName(), "ObstacleManager");
}

} // namespace los_keeper