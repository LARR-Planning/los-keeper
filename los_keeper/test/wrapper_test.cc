#include "los_keeper/wrapper/wrapper.h"
#include "gtest/gtest.h"
#include <Eigen/Core>

namespace los_keeper {

class ApiTestFixture : public ::testing::Test {
public:
  ApiTestFixture() {}

protected:
  Wrapper wrapper_;
  virtual void SetUp() override {}

  virtual void TearDown() override {}
};

TEST_F(ApiTestFixture, ControlShouldNullWhenNotActivated) {
  wrapper_.OnPlanningTimerCallback();
  EXPECT_EQ(wrapper_.GenerateControlInputFromPlanning(0), std::nullopt);

  wrapper_.OnStartServiceCallback();
  wrapper_.OnPlanningTimerCallback();
  wrapper_.OnPlanningTimerCallback();

  EXPECT_EQ(wrapper_.planning_result_.seq, 2);
}

} // namespace los_keeper