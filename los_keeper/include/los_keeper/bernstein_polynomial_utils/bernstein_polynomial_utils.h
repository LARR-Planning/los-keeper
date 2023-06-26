//
// Created by larr-planning on 23. 5. 4.
//
#ifndef HEADER_BERNSTEIN_POLYNOMIAL_UTILS
#define HEADER_BERNSTEIN_POLYNOMIAL_UTILS
int factorial(int num);
int nchoosek(int n, int r);
#include "gtest/gtest.h"
#include <cmath>
using namespace std;
using BernsteinCoefficients = vector<float>;
class BernsteinPoly {
  friend class ApiTestFixtureBernstein;
  FRIEND_TEST(ApiTestFixtureBernstein, CheckSetGetDegree);
  FRIEND_TEST(ApiTestFixtureBernstein, CheckSetGetBernsteinCoefficients);
  FRIEND_TEST(ApiTestFixtureBernstein, CheckSetGetTimeInterval);
  FRIEND_TEST(ApiTestFixtureBernstein, CheckCopyConstructor);
  FRIEND_TEST(ApiTestFixtureBernstein, CheckAllSet);

private:
  float time_interval_[2]{-9999.0f, -9999.1f}; // initial time and terminal time
  BernsteinCoefficients bernstein_coeff_;      // bernstein coefficient
  int degree_;                                 // The degree of a polynomial
public:
  BernsteinPoly() { degree_ = -1; };
  BernsteinPoly(const float time_interval[], const BernsteinCoefficients &bernstein_coeff,
                const int &degree);
  BernsteinPoly(const BernsteinPoly &bern_poly) {
    time_interval_[0] = bern_poly.time_interval_[0];
    time_interval_[1] = bern_poly.time_interval_[1];
    degree_ = bern_poly.degree_;
    bernstein_coeff_ = bern_poly.bernstein_coeff_;
  };
  ~BernsteinPoly();
  void SetTimeInterval(float time_interval_[]);
  void SetBernsteinCoeff(const BernsteinCoefficients &bernstein_coeff);
  void SetDegree(int degree_);
  int GetDegree() const;
  bool IsSet() {
    return ((time_interval_[0] < time_interval_[1]) and (not bernstein_coeff_.empty())) and
           (degree_ != -1);
  };
  const float *GetTimeInterval() const { return time_interval_; };
  BernsteinCoefficients GetBernsteinCoefficient() const { return this->bernstein_coeff_; };
  float GetValue(float t) const;
  float GetInitialValue() { return bernstein_coeff_[0]; };
  float GetTerminalValue() { return bernstein_coeff_[degree_]; };
  BernsteinPoly ElevateDegree(int m); // Change to higher degree (m)
  BernsteinPoly operator+(const BernsteinPoly &rhs_);
  BernsteinPoly operator-(const BernsteinPoly &rhs_);
  BernsteinPoly operator*(const BernsteinPoly &rhs_);
  BernsteinPoly operator*(const float &scalar_);
};

#endif /* HEADER_BERNSTEIN_POLYNOMIAL_UTILS */
