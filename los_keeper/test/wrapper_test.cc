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
  std::vector<ObjectState> object_state_list(2);
  drone_state.t_sec = 1.0;
  wrapper_.SetDroneState(drone_state);
  wrapper_.SetObjectStateArray(object_state_list);

  wrapper_.OnToggleActivateServiceCallback();

  // replan tried as planning is not safe
  wrapper_.OnPlanningTimerCallback();
  EXPECT_EQ(wrapper_.state_.is_data_received, true);

  // planning tried, having seq = 1
  EXPECT_GT(wrapper_.planning_result_.seq, 0);

  // However, planning is not valid as prediction is not successful.
  // EXPECT_EQ(wrapper_.planning_result_.chasing_trajectory, std::nullopt);
}

TEST_F(ApiTestFixture, RePlanningShouldTriedWhenSomethingWrong) {
  // Once crone state and object state are received, planning will triggerred by toggle
  DroneState drone_state;
  std::vector<ObjectState> object_state_list(2);
  drone_state.t_sec = 1.0;
  wrapper_.SetDroneState(drone_state);
  wrapper_.SetObjectStateArray(object_state_list);

  wrapper_.OnToggleActivateServiceCallback();

  wrapper_.OnPlanningTimerCallback();
  int current_planning_seq = wrapper_.planning_result_.seq;
  EXPECT_GT(current_planning_seq, 0);

  // If the planning success, the planing will be safe and visible.
  // Currently, we simulate this
  // TODO(Lee): when `Wrapper::UpdateState` is done, change these
  wrapper_.state_.is_planning_safe = true;
  wrapper_.state_.is_currently_safe = true;
  wrapper_.state_.is_planning_visible = true;

  // Still, planning not expired
  wrapper_.OnPlanningTimerCallback();
  EXPECT_EQ(wrapper_.planning_result_.seq, current_planning_seq);

  // But if planning expired, planning again
  std::chrono::seconds duration(int(wrapper_.parameters_.planning.replan_period_sec));
  std::this_thread::sleep_for(duration);
  wrapper_.OnPlanningTimerCallback();
  EXPECT_GT(wrapper_.planning_result_.seq, current_planning_seq);
  current_planning_seq = wrapper_.planning_result_.seq;

  // Even if planning is not expired, planning if something wrong
  wrapper_.state_.is_planning_visible = false;
  wrapper_.OnPlanningTimerCallback();
  EXPECT_GT(wrapper_.planning_result_.seq, current_planning_seq);
}

} // namespace los_keeper