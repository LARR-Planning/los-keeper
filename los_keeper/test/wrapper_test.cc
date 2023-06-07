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

TEST_F(ApiTestFixture, PlanningShouldNullWhenNotActivatedOrNotReceived) {
  wrapper_.OnPlanningTimerCallback();
  EXPECT_EQ(wrapper_.GenerateControlInputFromPlanning(0), std::nullopt);

  wrapper_.OnToggleActivateServiceCallback();
  wrapper_.OnPlanningTimerCallback();

  EXPECT_EQ(wrapper_.planning_result_.seq, 0);
}

TEST_F(ApiTestFixture, PlanningShouldTriedWhenActivatedAndReceived) {
  DroneState drone_state;
  drone_state.t_sec = 1.0;
  wrapper_.SetDroneState(drone_state);

  wrapper_.OnToggleActivateServiceCallback();

  // replan tried as planning is not safe
  wrapper_.OnPlanningTimerCallback();
  EXPECT_EQ(wrapper_.state_.is_data_received, true);

  // planning tried, having seq = 1
  EXPECT_GT(wrapper_.planning_result_.seq, 0);

  // However, planning is not valid as prediction is not successful.
  EXPECT_EQ(wrapper_.planning_result_.chasing_trajectory, std::nullopt);
}

} // namespace los_keeper