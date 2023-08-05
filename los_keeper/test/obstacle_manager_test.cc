#include "los_keeper/obstacle_manager/obstacle_manager.h"

namespace los_keeper {

class ApiTestFixtureObstacle : public ::testing::Test {
public:
  ApiTestFixtureObstacle() {}

protected:
  ObstacleManager *obstace_manager;
  virtual void SetUp() override {}
  virtual void TearDown() override {}
};

TEST_F(ApiTestFixtureObstacle, CheckCollisionBetweenPointsAndObstacles) {
  printf("CheckCollisionBetweenPointsAndObstacles\n");
  ObstacleParameter obstacle_param;
  obstacle_param.planning_horizon = 1.0f;
  obstace_manager = new ObstacleManager(obstacle_param);
  ObjectState single_obstacle;
  single_obstacle.id = 0;
  single_obstacle.px = 4;
  single_obstacle.py = 5;
  single_obstacle.pz = 6;
  single_obstacle.vx = 0;
  single_obstacle.vy = 0;
  single_obstacle.vz = 0;
  single_obstacle.rx = 0.1;
  single_obstacle.ry = 0.1;
  single_obstacle.rz = 0.1;
  vector<ObjectState> obstacle_array;
  obstacle_array.push_back(single_obstacle);
  obstace_manager->SetStructuredObstacleState(obstacle_array);
  KeeperState drone_state;
  drone_state.px = 4, drone_state.vx = 0, drone_state.ax = 0;
  drone_state.py = 4, drone_state.vy = 0, drone_state.ay = 0;
  drone_state.py = 4, drone_state.vz = 0, drone_state.az = 0;
  drone_state.rx = 0.1, drone_state.ry = 0.1, drone_state.rz = 0.1;
  bool is_collision = obstace_manager->CheckCollisionWithPoint(drone_state);
  EXPECT_EQ(false, is_collision);
}

TEST_F(ApiTestFixtureObstacle, CheckCollisionBetweenTrajectoryAndObstacles) {
  printf("CheckCollisionBetweenTrajectoryAndObstacles\n");
  ObstacleParameter obstacle_param;
  obstacle_param.planning_horizon = 1.0f;
  obstace_manager = new ObstacleManager(obstacle_param);
  StatePoly trajectory;
  double time_interval[2]{0.0, obstacle_param.planning_horizon};
  trajectory.SetTimeInterval(time_interval);
  trajectory.SetDegree(3);
  BernsteinCoefficients bernstein_coeff_x{0.0, 1.0, 2.0, 3.0};
  BernsteinCoefficients bernstein_coeff_y{0.0, 1.0, 2.0, 3.0};
  BernsteinCoefficients bernstein_coeff_z{0.0, 1.0, 2.0, 3.0};
  trajectory.px.SetBernsteinCoeff(bernstein_coeff_x);
  trajectory.py.SetBernsteinCoeff(bernstein_coeff_y);
  trajectory.pz.SetBernsteinCoeff(bernstein_coeff_z);
  trajectory.rx = 0.5;
  trajectory.ry = 0.5;
  trajectory.rz = 0.5;

  ObjectState single_obstacle;
  single_obstacle.id = 0;
  single_obstacle.px = 4;
  single_obstacle.py = 5;
  single_obstacle.pz = 6;
  single_obstacle.vx = 0;
  single_obstacle.vy = 0;
  single_obstacle.vz = 0;
  single_obstacle.rx = 0.1;
  single_obstacle.ry = 0.1;
  single_obstacle.rz = 0.1;
  vector<ObjectState> obstacle_array;
  obstacle_array.push_back(single_obstacle);
  obstace_manager->SetStructuredObstacleState(obstacle_array);
  bool is_collision = obstace_manager->CheckCollisionAlongTrajectory(trajectory);
  EXPECT_EQ(false, is_collision);
}

} // namespace los_keeper