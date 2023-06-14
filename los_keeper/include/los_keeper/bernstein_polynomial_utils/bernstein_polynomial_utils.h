//
// Created by larr-planning on 23. 5. 4.
//
#ifndef HEADER_BERNSTEIN_POLYNOMIAL_UTILS
#define HEADER_BERNSTEIN_POLYNOMIAL_UTILS
int factorial(int num);
int nchoosek(int n, int r);
#include "gtest/gtest.h"
#include <cmath>

class BernsteinPoly {
  friend class ApiTestFixtureBernstein;
  FRIEND_TEST(ApiTestFixtureBernstein, CheckSetGetDegree);
  FRIEND_TEST(ApiTestFixtureBernstein, CheckSetGetBernsteinCoefficients);
  FRIEND_TEST(ApiTestFixtureBernstein, CheckSetGetTimeInterval);
  FRIEND_TEST(ApiTestFixtureBernstein, CheckCopyConstructor);

private:
  float time_interval_[2]; // initial time and terminal time
  float *bernstein_coeff_; // bernstein coefficient
  int degree_;             // The degree of a polynomial
public:
  BernsteinPoly() {
    degree_ = -1;
    bernstein_coeff_ = nullptr;
  };
  BernsteinPoly(float time_interval[], float bernstein_coeff[], const int &degree);
  BernsteinPoly(const BernsteinPoly &bern_poly) {
    time_interval_[0] = bern_poly.time_interval_[0];
    time_interval_[1] = bern_poly.time_interval_[1];
    degree_ = bern_poly.degree_;
    bernstein_coeff_ = bern_poly.bernstein_coeff_;
  };
  ~BernsteinPoly();
  void SetTimeInterval(float time_interval_[]);
  void SetBernsteinCoeff(float bernstein_coeff_[]);
  void SetDegree(int degree_);
  int GetDegree() const;
  bool IsSet() { return ((time_interval_ != nullptr) and (bernstein_coeff_ != nullptr)); };
  float *GetTimeInterval() { return time_interval_; };
  float *GetBernsteinCoefficient() { return this->bernstein_coeff_; };
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
