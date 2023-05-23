#include "los_keeper/wrapper/wrapper.h"
#include "gtest/gtest.h"
#include <Eigen/Core>

namespace los_keeper {

TEST(WrapperTest, NameShouldCorrect) {
  printf("testing wrapper..\n");
  Wrapper wrapper;
  EXPECT_TRUE(true);
}

} // namespace los_keeper