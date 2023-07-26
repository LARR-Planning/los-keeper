#ifndef HEADER_WRAPPER
#define HEADER_WRAPPER

#include "gtest/gtest.h"
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "los_keeper/obstacle_manager/obstacle_manager.h"
#include "los_keeper/target_manager/target_manager.h"
#include "los_keeper/trajectory_planner/trajectory_planner.h"
#include "los_keeper/wrapper/store.h"

namespace los_keeper {

struct DebugInfo {
  ObstacleManagerDebugInfo obstacle_manager;
  TargetManagerDebugInfo target_manager;
  PlanningDebugInfo planning;
};

struct PlanningProblem {
  DroneState drone_state;
  PclPointCloud point_cloud;
  std::vector<StatePoly> structured_obstacle_poly_list;
  std::vector<ObjectState> target_state_list;
};

struct PlanningResult {
  double last_plan_success_t_sec{0};
  int seq{0};
  std::optional<StatePoly> chasing_trajectory;
  std::optional<Point> GetPointAtTime(double t) const;
  std::optional<JerkControlInput> GetJerkInputAtTime(double t) const;
  std::optional<VelocityControlInput> GetVelocityInputAtTime(double t) const;
  std::optional<AccelControlInput> GetAccelInputAtTime(double t) const;
};

class Wrapper {
  // TODO(@): removed when releasing
  friend class ApiTestFixture;
  FRIEND_TEST(ApiTestFixture, PlanningShouldNullWhenNotActivatedOrNotReceived);
  FRIEND_TEST(ApiTestFixture, PlanningShouldTriedWhenActivatedAndReceived);
  FRIEND_TEST(ApiTestFixture, RePlanningShouldTriedWhenSomethingWrong);

private:
  Parameters parameters_;
  DroneState drone_state_;
  store::State state_;
  std::vector<ObjectState> object_state_list_;
  std::vector<ObjectState> target_state_list_;
  PlanningResult planning_result_;
  DebugInfo debug_info_;

  struct {
    std::mutex drone_state;
    std::mutex object_state_list;
    std::mutex target_state_list;
    std::mutex point_cloud;
    std::mutex control;
    std::mutex debug_obstacle_info;
    std::mutex debug_target_info;
    std::mutex debug_planning_info;
  } mutex_list_;

  std::shared_ptr<ObstacleManager> obstacle_manager_;
  std::shared_ptr<TargetManager> target_manager_;
  std::shared_ptr<TrajectoryPlanner> trajectory_planner_;

  void UpdateState(store::State &state);

  void HandleStopAction();
  void HandleActivateAction();
  void HandleReplanAction();
  void HandleIdleAction();

public:
  Wrapper();
  explicit Wrapper(const Parameters &parameters);

  void SetPoints(const pcl::PointCloud<pcl::PointXYZ> &points);
  void SetDroneState(const DroneState &drone_state);
  void SetObjectStateArray(const std::vector<ObjectState> &object_state_list);
  void SetTargetStateArray(const std::vector<ObjectState> &target_state_list);
  std::optional<JerkControlInput> GenerateControlInputFromPlanning(double time);
  std::optional<AccelControlInput> GenerateAccelControlInputFromPlanning(double time);
  std::optional<VelocityControlInput> GenerateVelocityControlInputFromPlanning(double time);
  DebugInfo GetDebugInfo();
  void UpdateTargetDebugInfo();
  void UpdateObstacleDeubgInfo();
  void UpdatePlanningDebugInfo();
  void OnPlanningTimerCallback();
  void OnToggleActivateServiceCallback();
};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
