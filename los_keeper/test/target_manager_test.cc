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

TEST_F(ApiTestFixtureTargetManager2D, CheckSampleEndPoints) {}

class ApiTestFixtureTargetManager3D : public ::testing::Test {
public:
  ApiTestFixtureTargetManager3D() {}

protected:
  TargetManager3D target_manager_3d_;
  virtual void SetUp() override {}
  virtual void TearDown() override {}
};
TEST_F(ApiTestFixtureTargetManager3D, CheckSampleEndPoints) {}

} // namespace los_keeper
