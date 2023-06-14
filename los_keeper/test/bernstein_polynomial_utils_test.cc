#include "los_keeper/bernstein_polynomial_utils/bernstein_polynomial_utils.h"
#include "gtest/gtest.h"

namespace los_keeper {
class ApiTestFixtureBernsteinUtils : public ::testing::Test {
public:
  ApiTestFixtureBernsteinUtils() {}

protected:
  BernsteinPoly bernstein_poly_;
  virtual void SetUp() override {}
  virtual void TearDown() override {}
};

TEST_F(ApiTestFixtureBernsteinUtils, CheckSetGetDegree) {
  bernstein_poly_.SetDegree(3);
  EXPECT_EQ(bernstein_poly_.GetDegree(), 3);
}

TEST_F(ApiTestFixtureBernsteinUtils, CheckSetGetBernsteinCoefficients) {
  bernstein_poly_.SetDegree(3);
  float coeff1[4]{0.0f, 1.0f, 2.0f, 3.0f};
  bernstein_poly_.SetBernsteinCoeff(coeff1);
  coeff1[0] = -1.0f;
  float coeff2[4]{0.0f, 1.0f, 2.0f, 3.0f};
  EXPECT_EQ(bernstein_poly_.GetBernsteinCoefficient()[0], coeff2[0]);
  EXPECT_EQ(bernstein_poly_.GetBernsteinCoefficient()[1], coeff2[1]);
  EXPECT_EQ(bernstein_poly_.GetBernsteinCoefficient()[2], coeff2[2]);
  EXPECT_EQ(bernstein_poly_.GetBernsteinCoefficient()[3], coeff2[3]);
}

TEST_F(ApiTestFixtureBernsteinUtils, CheckSetGetTimeInterval) {
  float time_interval1[2]{0.0f, 3.0f};
  bernstein_poly_.SetTimeInterval(time_interval1);
  time_interval1[1] = 2.0f;
  float time_interval2[2]{0.0f, 3.0f};
  EXPECT_EQ(bernstein_poly_.GetTimeInterval()[0], time_interval2[0]);
  EXPECT_EQ(bernstein_poly_.GetTimeInterval()[1], time_interval2[1]);
}

TEST_F(ApiTestFixtureBernsteinUtils, CheckGetStartEndPoints) {
  float coeff[4]{3.0f, 2.0f, 1.0f, 0.0f};
  bernstein_poly_.SetDegree(3);
  bernstein_poly_.SetBernsteinCoeff(coeff);
  EXPECT_EQ(bernstein_poly_.GetInitialValue(), coeff[0]);
  EXPECT_EQ(bernstein_poly_.GetTerminalValue(), coeff[3]);
}

TEST_F(ApiTestFixtureBernsteinUtils, CheckGetBernsteinValue) {
  float coeff[4]{0.0f, 1.0f, 2.0f, 3.0f};
  float interval[2]{0.0f, 1.0f};
  int degree = 3;
  bernstein_poly_.SetDegree(degree);
  bernstein_poly_.SetBernsteinCoeff(coeff);
  bernstein_poly_.SetTimeInterval(interval);
  EXPECT_LT(abs(bernstein_poly_.GetValue(interval[0]) - bernstein_poly_.GetInitialValue()), 0.001f);
  EXPECT_LT(abs(bernstein_poly_.GetValue(interval[1]) - bernstein_poly_.GetTerminalValue()),
            0.001f);
}
TEST_F(ApiTestFixtureBernsteinUtils, CheckCopyConstructor) {
  float coeff[4]{0.0f, 1.0f, 2.0f, 3.0f};
  float interval[2]{0.0f, 1.0f};
  int degree = 3;
  bernstein_poly_.SetDegree(degree);
  bernstein_poly_.SetBernsteinCoeff(coeff);
  bernstein_poly_.SetTimeInterval(interval);

  BernsteinPoly copied_one(bernstein_poly_);
  EXPECT_EQ(bernstein_poly_.GetDegree(), copied_one.GetDegree());
  EXPECT_EQ(bernstein_poly_.GetTimeInterval()[0], copied_one.GetTimeInterval()[0]);
  EXPECT_EQ(bernstein_poly_.GetTimeInterval()[1], copied_one.GetTimeInterval()[1]);
  EXPECT_EQ(bernstein_poly_.GetBernsteinCoefficient()[0], copied_one.GetBernsteinCoefficient()[0]);
  EXPECT_EQ(bernstein_poly_.GetBernsteinCoefficient()[1], copied_one.GetBernsteinCoefficient()[1]);
  EXPECT_EQ(bernstein_poly_.GetBernsteinCoefficient()[2], copied_one.GetBernsteinCoefficient()[2]);
  EXPECT_EQ(bernstein_poly_.GetBernsteinCoefficient()[3], copied_one.GetBernsteinCoefficient()[3]);
}
TEST_F(ApiTestFixtureBernsteinUtils, CheckAllSet) {
  EXPECT_EQ(bernstein_poly_.IsSet(), false);
  bernstein_poly_.SetDegree(3);
  EXPECT_EQ(bernstein_poly_.IsSet(), false);
  float coeff[4]{0.0f, 1.0f, 2.0f, 3.0f};
  bernstein_poly_.SetBernsteinCoeff(coeff);
  EXPECT_EQ(bernstein_poly_.IsSet(), false);
  float time_interval[2]{0.0f, 2.5f};
  bernstein_poly_.SetTimeInterval(time_interval);
  EXPECT_EQ(bernstein_poly_.IsSet(), true);
}

} // namespace los_keeper