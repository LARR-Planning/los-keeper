#include "los_keeper/wrapper/wrapper.h"
#include "gtest/gtest.h"
#include <Eigen/Core>

namespace los_keeper {

class ApiTestFixture : public ::testing::Test {
public:
  ApiTestFixture() : wrapper_(Parameters()) {}

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
  PredictionParameter predict_param;
  predict_param.sampling.num_sample = 500;
  predict_param.sampling.num_thread = 2;
  predict_param.sampling.is_lite = false;
  predict_param.distance.obstacle_max = 100.0f;
  predict_param.virtual_pcl_bbox.height = 100.0f;
  predict_param.virtual_pcl_bbox.width = 100.0f;
  predict_param.dynamic_limits.vel_max = 1.0f;
  predict_param.dynamic_limits.acc_max = 3.0f;
  predict_param.horizon.prediction = 1.0f;
  wrapper_.target_manager_.reset(new TargetManager2D(predict_param));
  ObstacleParameter obstacle_param;
  obstacle_param.planning_horizon = 1.0f;
  wrapper_.obstacle_manager_.reset(new ObstacleManager(obstacle_param));

  PlanningParameter planning_param;
  planning_param.sampling.num_sample = 500;
  planning_param.sampling.num_thread = 2;
  planning_param.sampling.is_lite = false;
  planning_param.horizon.planning = 1.0f;
  planning_param.dynamic_limits.vel_max = 3.0f;
  planning_param.dynamic_limits.acc_max = 10.0f;
  ProblemParameter problem_param;
  problem_param.replanning_period = 0.1f;
  planning_param.virtual_pcl_bbox.width = 100.0f;
  planning_param.virtual_pcl_bbox.height = 100.0f;
  planning_param.distance.obstacle_max = 100.0f;
  planning_param.distance.target_min = 0.3f;
  planning_param.distance.target_max = 5.0f;
  planning_param.safe_distance.rx = 0.1f;
  planning_param.safe_distance.ry = 0.1f;
  planning_param.safe_distance.rz = 0.1f;

  KeeperState drone_state;
  //  std::vector<ObjectState> object_state_list(2);
  std::vector<ObjectState> target_state_list(1);
  target_state_list[0].id = 0;
  target_state_list[0].px = 0;
  target_state_list[0].py = 0;
  target_state_list[0].pz = 0;
  target_state_list[0].vx = 0;
  target_state_list[0].vy = 0;
  target_state_list[0].vz = 0;
  target_state_list[0].rx = 0.1;
  target_state_list[0].ry = 0.1;
  target_state_list[0].rz = 0.1;
  target_state_list[0].t_sec = 1.0;
  drone_state.t_sec = 1.0;
  drone_state.px = 0.4;
  drone_state.py = 0.4;
  drone_state.pz = 0.4;
  drone_state.vx = 0.0;
  drone_state.vy = 0.0;
  drone_state.vz = 0.0;
  drone_state.ax = 0.0;
  drone_state.ay = 0.0;
  drone_state.az = 0.0;
  drone_state.rx = 0.1;
  drone_state.ry = 0.1;
  drone_state.rz = 0.15;

  wrapper_.SetKeeperState(drone_state);
  wrapper_.SetTargetStateArray(target_state_list);
  //  wrapper_.SetObjectStateArray(object_state_list);

  wrapper_.OnToggleActivateServiceCallback();

  // replan tried as planning is not safe
  wrapper_.OnPlanningTimerCallback();
  EXPECT_EQ(wrapper_.state_.is_data_received, true);

  // planning tried, having seq = 1 //TODO: Increase seq when TRY? OR SUCCESS?
  //  printf("AAAAAAAAAA\n");
  //  EXPECT_GT(wrapper_.planning_result_.seq, 0);
  //  printf("BBBBBBBBBB\n");
  // However, planning is not valid as prediction is not successful.
  // EXPECT_EQ(wrapper_.planning_result_.chasing_trajectory, std::nullopt);
}
//
TEST_F(ApiTestFixture, RePlanningShouldTriedWhenSomethingWrong) {
  // Once crone state and object state are received, planning will triggerred by toggle
  PredictionParameter predict_param;
  predict_param.sampling.num_sample = 500;
  predict_param.sampling.num_thread = 2;
  predict_param.sampling.is_lite = false;
  predict_param.distance.obstacle_max = 100.0f;
  predict_param.virtual_pcl_bbox.height = 100.0f;
  predict_param.virtual_pcl_bbox.width = 100.0f;
  predict_param.dynamic_limits.vel_max = 1.0f;
  predict_param.dynamic_limits.acc_max = 3.0f;
  predict_param.horizon.prediction = 1.0f;
  wrapper_.target_manager_.reset(new TargetManager2D(predict_param));
  ObstacleParameter obstacle_param;
  obstacle_param.planning_horizon = 1.0f;
  wrapper_.obstacle_manager_.reset(new ObstacleManager(obstacle_param));

  PlanningParameter planning_param;
  planning_param.sampling.num_sample = 500;
  planning_param.sampling.num_thread = 2;
  planning_param.sampling.is_lite = false;
  planning_param.horizon.planning = 1.0f;
  planning_param.dynamic_limits.vel_max = 3.0f;
  planning_param.dynamic_limits.acc_max = 10.0f;
  ProblemParameter problem_param;
  problem_param.replanning_period = 0.5f;
  planning_param.virtual_pcl_bbox.width = 100.0f;
  planning_param.virtual_pcl_bbox.height = 100.0f;
  planning_param.distance.obstacle_max = 100.0f;
  planning_param.distance.target_min = 0.3f;
  planning_param.distance.target_max = 5.0f;
  planning_param.safe_distance.rx = 0.1f;
  planning_param.safe_distance.ry = 0.1f;
  planning_param.safe_distance.rz = 0.1f;

  KeeperState drone_state;
  //  std::vector<ObjectState> object_state_list(2);
  std::vector<ObjectState> target_state_list(1);
  target_state_list[0].id = 0;
  target_state_list[0].px = 0;
  target_state_list[0].py = 0;
  target_state_list[0].pz = 0;
  target_state_list[0].vx = 0;
  target_state_list[0].vy = 0;
  target_state_list[0].vz = 0;
  target_state_list[0].rx = 0.1;
  target_state_list[0].ry = 0.1;
  target_state_list[0].rz = 0.1;
  target_state_list[0].t_sec = 1.0;
  drone_state.t_sec = 1.0;
  drone_state.px = 0.4;
  drone_state.py = 0.4;
  drone_state.pz = 0.4;
  drone_state.vx = 0.0;
  drone_state.vy = 0.0;
  drone_state.vz = 0.0;
  drone_state.ax = 0.0;
  drone_state.ay = 0.0;
  drone_state.az = 0.0;
  drone_state.rx = 0.1;
  drone_state.ry = 0.1;
  drone_state.rz = 0.15;
  wrapper_.SetKeeperState(drone_state);
  //  wrapper_.SetObjectStateArray(object_state_list);
  wrapper_.SetTargetStateArray(target_state_list);
  wrapper_.OnToggleActivateServiceCallback();

  wrapper_.OnPlanningTimerCallback();
  int current_planning_seq =
      wrapper_.planning_result_.seq; // TODO: Increase seq when try or success?
                                     //  EXPECT_GT(current_planning_seq, 0);

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
  //  std::chrono::seconds duration(int(wrapper_.parameters_.problem.replanning_period));
  //  std::this_thread::sleep_for(duration);
  //  wrapper_.OnPlanningTimerCallback();
  //  EXPECT_GT(wrapper_.planning_result_.seq, current_planning_seq);
  //  current_planning_seq = wrapper_.planning_result_.seq; //TODO (LEE): Regardless of SUCCESS?
  //
  //  // Even if planning is not expired, planning if something wrong
  //  wrapper_.state_.is_planning_visible = false;
  //  wrapper_.OnPlanningTimerCallback();
  //  EXPECT_GT(wrapper_.planning_result_.seq, current_planning_seq);
}

TEST_F(ApiTestFixture, ControlInput) {
  KeeperState drone_state;
  std::vector<ObjectState> object_state_list(2);
  drone_state.t_sec = 1.0;
  wrapper_.SetKeeperState(drone_state);
  wrapper_.SetTargetStateArray(object_state_list);

  wrapper_.OnToggleActivateServiceCallback();

  // TODO(Lee): once your prediction works, uncomment and test them
  /**
  double t_eval = 1.0;
  wrapper_.OnPlanningTimerCallback();
  auto control_input = wrapper_.GenerateControlInputFromPlanning(t_eval);
  EXPECT_EQ(control_input->seq, 1);

  wrapper_.OnPlanningTimerCallback();
  control_input = wrapper_.GenerateControlInputFromPlanning(t_eval);
  EXPECT_EQ(control_input->seq, 2);
   */
}

} // namespace los_keeper