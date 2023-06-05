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

struct PlanningProblem {
  DroneState drone_state;
  PclPointCloud point_cloud;
  std::vector<StatePoly> structured_obstacle_poly_list;
  std::vector<ObjectState> target_state_list;
};

struct PlanningResult {
  int seq{0};
  std::optional<StatePoly> chasing_trajectory;
  std::optional<Point> GetPointAtTime(double t) const;
};

class Wrapper {
  // TODO(@): removed when releasing
  friend class ApiTestFixture;
  FRIEND_TEST(ApiTestFixture, PlanningShouldNullWhenNotActivatedOrNotReceived);
  FRIEND_TEST(ApiTestFixture, PlanningShouldTriedWhenActivatedAndReceived);

private:
  ProblemParam problem_param_;
  ObstacleParam obstacle_param_;
  PredictionParam prediction_param_;
  PlanningParam planning_param_;

  DroneState drone_state_;
  store::State state_;
  PlanningResult planning_result_;

  struct {
    std::mutex drone_state;
    std::mutex point_cloud;
    std::mutex control;
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
  explicit Wrapper(const ProblemParam &problem_param, const ObstacleParam &obstacle_param,
                   const PredictionParam &prediction_param, const PlanningParam &planning_param);
  void SetParameters(const ProblemParam &problem_param, const ObstacleParam &obstacle_param,
                     const PredictionParam &prediction_param, const PlanningParam &planning_param) {
    problem_param_ = problem_param;
    obstacle_param_ = obstacle_param;
    prediction_param_ = prediction_param;
    planning_param_ = planning_param;
  };
  void SetPoints(const pcl::PointCloud<pcl::PointXYZ> &points);
  void SetDroneState(const DroneState &drone_state);
  std::optional<Point> GenerateControlInputFromPlanning(double time);

  void OnPlanningTimerCallback();
  void OnStartServiceCallback();
};
} // namespace los_keeper

#endif /* HEADER_WRAPPER */
