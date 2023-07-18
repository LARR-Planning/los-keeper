#include "los_keeper/target_manager/target_manager.h"
#include "gtest/gtest.h"

namespace los_keeper {
class ApiTestFixtureTargetManager2D : public ::testing::Test {
public:
  ApiTestFixtureTargetManager2D() {}

protected:
  TargetManager2D target_manager_2d_;
  virtual void SetUp() override {}
  virtual void TearDown() override {}
};

TEST_F(ApiTestFixtureTargetManager2D, CheckWhenNoObstacles2D) {
  target_manager_2d_.param_.distance.obstacle_max = 1.0f;
  target_manager_2d_.param_.dynamic_limits.vel_max = 1.0f;
  target_manager_2d_.param_.dynamic_limits.acc_max = 0.1f;
  target_manager_2d_.param_.virtual_pcl_bbox.height = 1.0f;
  target_manager_2d_.param_.virtual_pcl_bbox.width = 1.0f;
  target_manager_2d_.param_.horizon.prediction = 1.0f;
  target_manager_2d_.param_.sampling.num_sample = 500;
  target_manager_2d_.param_.sampling.num_thread = 4;
  target_manager_2d_.param_.sampling.is_lite = false;
  ObjectState target_1;
  ObjectState target_2;
  target_1.id = 0;
  target_1.rx = 0.1f;
  target_1.ry = 0.1f;
  target_1.rz = 0.1f;
  target_1.px = 0.0f;
  target_1.py = 1.0f;
  target_1.pz = 0.0f;
  target_1.vx = 1.0f;
  target_1.vy = 0.0f;
  target_1.vz = 0.0f;
  target_2.id = 1;
  target_2.rx = 0.1f;
  target_2.ry = 0.1f;
  target_2.rz = 0.1f;
  target_2.px = -0.5f;
  target_2.py = 1.0f;
  target_2.pz = 0.1f;
  target_2.vx = 0.0f;
  target_2.vy = 1.0f;
  target_2.vz = 0.0f;
  vector<ObjectState> target_state_list;
  target_state_list.push_back(target_1);
  target_state_list.push_back(target_2);
  vector<StatePoly> empty_obs;
  los_keeper::PclPointCloud empty_pcl;
  target_manager_2d_.num_target_ = target_state_list.size();
  target_manager_2d_.SampleEndPoints(target_state_list);
  EXPECT_EQ(target_manager_2d_.end_points_[0].size(),
            target_manager_2d_.param_.sampling.num_sample);
  EXPECT_EQ(target_manager_2d_.end_points_[1].size(),
            target_manager_2d_.param_.sampling.num_sample);
  target_manager_2d_.ComputePrimitives(target_state_list);
  EXPECT_EQ(target_manager_2d_.primitives_list_[0].size(),
            target_manager_2d_.param_.sampling.num_sample);
  EXPECT_EQ(target_manager_2d_.primitives_list_[1].size(),
            target_manager_2d_.param_.sampling.num_sample);
  target_manager_2d_.CalculateCloseObstacleIndex(target_state_list, empty_obs);
  EXPECT_EQ(target_manager_2d_.close_obstacle_index_[0].size(), 0);
  EXPECT_EQ(target_manager_2d_.close_obstacle_index_[1].size(), 0);
  target_manager_2d_.CheckCollision(empty_pcl, empty_obs);
  EXPECT_EQ(target_manager_2d_.primitive_safe_total_index_[0].size(),
            target_manager_2d_.primitives_list_[0].size());
  EXPECT_EQ(target_manager_2d_.primitive_safe_total_index_[1].size(),
            target_manager_2d_.primitives_list_[1].size());
  target_manager_2d_.CalculateCentroid();
  printf("%d-th target best index: %d\n", 0, target_manager_2d_.primitive_best_index_[0]);
  printf("%d-th target best index: %d\n", 1, target_manager_2d_.primitive_best_index_[1]);
  printf("Time Interval: [%f, %f]\n",
         target_manager_2d_.primitives_list_[0][target_manager_2d_.primitive_best_index_[0]]
             .px.GetTimeInterval()[0],
         target_manager_2d_.primitives_list_[0][target_manager_2d_.primitive_best_index_[0]]
             .px.GetTimeInterval()[1]);
  printf("0 th target x bernstein_coeff: ");
  for (int i = 0; i < 4; i++) {
    printf("%f, ",
           target_manager_2d_.primitives_list_[0][target_manager_2d_.primitive_best_index_[0]]
               .px.GetBernsteinCoefficient()[i]);
  }
  printf("\n");
  printf("1 th target x bernstein_coeff: ");
  for (int i = 0; i < 4; i++) {
    printf("%f, ",
           target_manager_2d_.primitives_list_[1][target_manager_2d_.primitive_best_index_[1]]
               .px.GetBernsteinCoefficient()[i]);
  }
}

class ApiTestFixtureTargetManager3D : public ::testing::Test {
public:
  ApiTestFixtureTargetManager3D() {}

protected:
  TargetManager3D target_manager_3d_;
  virtual void SetUp() override {}
  virtual void TearDown() override {}
};

TEST_F(ApiTestFixtureTargetManager3D, CheckWhenNoObstacles3D) {
  target_manager_3d_.param_.distance.obstacle_max = 1.0f;
  target_manager_3d_.param_.dynamic_limits.vel_max = 1.0f;
  target_manager_3d_.param_.dynamic_limits.acc_max = 1.0f;
  target_manager_3d_.param_.virtual_pcl_bbox.height = 1.0f;
  target_manager_3d_.param_.virtual_pcl_bbox.width = 1.0f;
  target_manager_3d_.param_.horizon.prediction = 1.0f;
  target_manager_3d_.param_.sampling.num_sample = 2000;
  target_manager_3d_.param_.sampling.num_thread = 4;
  target_manager_3d_.param_.sampling.is_lite = false;

  ObjectState target_1;
  ObjectState target_2;
  target_1.id = 0;
  target_1.rx = 0.1f;
  target_1.ry = 0.1f;
  target_1.rz = 0.1f;
  target_1.px = 0.0f;
  target_1.py = 0.0f;
  target_1.pz = 0.0f;
  target_1.vx = 1.0f;
  target_1.vy = 0.0f;
  target_1.vz = 0.5f;
  target_2.id = 1;
  target_2.rx = 0.1f;
  target_2.ry = 0.1f;
  target_2.rz = 0.1f;
  target_2.px = 0.0f;
  target_2.py = 1.0f;
  target_2.pz = 0.1f;
  target_2.vx = 2.0f;
  target_2.vy = 1.0f;
  target_2.vz = -1.0f;
  vector<ObjectState> target_state_list;
  target_state_list.push_back(target_1);
  target_state_list.push_back(target_2);
  vector<StatePoly> empty_obs;
  los_keeper::PclPointCloud empty_pcl;
  target_manager_3d_.num_target_ = target_state_list.size();
  target_manager_3d_.SampleEndPoints(target_state_list);
  EXPECT_EQ(target_manager_3d_.end_points_[0].size(),
            target_manager_3d_.param_.sampling.num_sample);
  EXPECT_EQ(target_manager_3d_.end_points_[1].size(),
            target_manager_3d_.param_.sampling.num_sample);
  target_manager_3d_.ComputePrimitives(target_state_list);
  EXPECT_EQ(target_manager_3d_.primitives_list_[0].size(),
            target_manager_3d_.param_.sampling.num_sample);
  EXPECT_EQ(target_manager_3d_.primitives_list_[1].size(),
            target_manager_3d_.param_.sampling.num_sample);
  target_manager_3d_.CalculateCloseObstacleIndex(target_state_list, empty_obs);
  EXPECT_EQ(target_manager_3d_.close_obstacle_index_[0].size(), 0);
  EXPECT_EQ(target_manager_3d_.close_obstacle_index_[1].size(), 0);
  target_manager_3d_.CheckCollision(empty_pcl, empty_obs);
  EXPECT_EQ(target_manager_3d_.primitive_safe_total_index_[0].size(),
            target_manager_3d_.primitives_list_[0].size());
  EXPECT_EQ(target_manager_3d_.primitive_safe_total_index_[1].size(),
            target_manager_3d_.primitives_list_[1].size());
  target_manager_3d_.CalculateCentroid();
  printf("%d-th target best index: %d\n", 0, target_manager_3d_.primitive_best_index_[0]);
  printf("%d-th target best index: %d\n", 1, target_manager_3d_.primitive_best_index_[1]);

  printf("0 th target z bernstein_coeff: ");
  for (int i = 0; i < 4; i++) {
    printf("%f, ",
           target_manager_3d_.primitives_list_[0][target_manager_3d_.primitive_best_index_[0]]
               .pz.GetBernsteinCoefficient()[i]);
  }
  printf("\n");
  printf("1 th target z bernstein_coeff: ");
  for (int i = 0; i < 4; i++) {
    printf("%f, ",
           target_manager_3d_.primitives_list_[1][target_manager_3d_.primitive_best_index_[1]]
               .pz.GetBernsteinCoefficient()[i]);
  }
  printf("\n");
}

} // namespace los_keeper
