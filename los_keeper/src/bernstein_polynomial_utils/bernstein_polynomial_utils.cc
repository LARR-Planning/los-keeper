#include "los_keeper/bernstein_polynomial_utils/bernstein_polynomial_utils.h"
int factorial(int num) {
  if (num <= 1)
    return 1;
  return num * factorial(num - 1);
}
int nchoosek(int n, int r) {
  long long int number = 1;
  if (n - r > r) {
    for (int i = n; i > n - r; i--) {
      number = number * i;
    }
    return (int)(number / factorial(r));
  } else {
    for (int i = n; i > r; i--) {
      number = number * i;
    }
    return (int)(number / factorial(n - r));
  }
}

BernsteinPoly::BernsteinPoly(const float time_interval[],
                             const BernsteinCoefficients &bernstein_coeff, const int &degree) {
  time_interval_[0] = time_interval[0];
  time_interval_[1] = time_interval[1];
  degree_ = degree;
  bernstein_coeff_ = bernstein_coeff;
}

void BernsteinPoly::SetTimeInterval(float time_interval[]) {
  this->time_interval_[0] = time_interval[0];
  this->time_interval_[1] = time_interval[1];
}

void BernsteinPoly::SetBernsteinCoeff(const BernsteinCoefficients &bernstein_coeff) {
  bernstein_coeff_ = bernstein_coeff;
}

void BernsteinPoly::SetDegree(int degree) { degree_ = degree; }

int BernsteinPoly::GetDegree() const { return degree_; }

float BernsteinPoly::GetValue(float t) const {
  int poly_order = this->degree_;
  float value = 0.0f;
  for (int i = 0; i < poly_order + 1; i++)
    value += bernstein_coeff_[i] * float(nchoosek(poly_order, i)) *
             (float)pow(t - time_interval_[0], i) *
             (float)pow(time_interval_[1] - t, poly_order - i) /
             (float)pow(time_interval_[1] - time_interval_[0], poly_order);
  return value;
}

BernsteinPoly BernsteinPoly::ElevateDegree(int m) {
  int poly_order = degree_;
  vector<BernsteinCoefficients> mat(poly_order + 1);
  for (int i = 0; i < poly_order + 1; i++)
    mat[i].resize(m + 1);

  for (int i = 0; i < poly_order + 1; i++) {
    for (int j = 0; j <= m - poly_order; j++)
      mat[i][i + j] = float(nchoosek(m - poly_order, j)) * float(nchoosek(poly_order, i)) /
                      float(nchoosek(m, i + j));
  }
  float element;
  BernsteinCoefficients dataPtr(m + 1);
  for (int i = 0; i < m + 1; i++) {
    element = 0.0f;
    for (int j = 0; j < poly_order + 1; j++) {
      element += bernstein_coeff_[j] * mat[j][i];
    }
    dataPtr[i] = element;
  }
  BernsteinPoly result(this->GetTimeInterval(), dataPtr, m);
  return result;
}

BernsteinPoly BernsteinPoly::operator+(const BernsteinPoly &rhs) {
  int poly_order = degree_;
  BernsteinCoefficients added_coeffs;
  for (int i = 0; i < poly_order + 1; i++)
    added_coeffs.push_back(this->bernstein_coeff_[i] + rhs.bernstein_coeff_[i]);
  BernsteinPoly result(this->GetTimeInterval(), added_coeffs, poly_order);
  return result;
}

BernsteinPoly BernsteinPoly::operator-(const BernsteinPoly &rhs) {
  int poly_order = degree_;
  BernsteinCoefficients subtracted_coeffs;
  for (int i = 0; i < poly_order + 1; i++)
    subtracted_coeffs.push_back(this->bernstein_coeff_[i] - rhs.bernstein_coeff_[i]);
  BernsteinPoly result(this->GetTimeInterval(), subtracted_coeffs, poly_order);
  return result;
}

BernsteinPoly BernsteinPoly::operator*(const BernsteinPoly &rhs) {
  int n_lhs = degree_;
  int n_rhs = rhs.GetDegree();
  BernsteinCoefficients dataPtr;
  dataPtr.resize(n_rhs + n_lhs + 1);
  //  auto *dataPtr = new float[n_lhs + n_rhs + 1];
  if (n_lhs >= n_rhs) {
    for (int i = 0; i <= n_lhs + n_rhs; i++) {
      if (i <= n_rhs) {
        for (int j = i; j >= 0; j--)
          dataPtr[i] += float(nchoosek(n_lhs, j)) * float(nchoosek(n_rhs, i - j)) /
                        float(nchoosek(n_lhs + n_rhs, i)) * this->bernstein_coeff_[j] *
                        rhs.bernstein_coeff_[i - j];
      } else if ((i >= n_rhs + 1) and (i <= n_lhs)) {
        for (int j = i; j >= i - n_rhs; j--)
          dataPtr[i] += float(nchoosek(n_lhs, j)) * float(nchoosek(n_rhs, i - j)) /
                        float(nchoosek(n_lhs + n_rhs, i)) * this->bernstein_coeff_[j] *
                        rhs.bernstein_coeff_[i - j];
      } else if (i >= n_lhs + 1) {
        for (int j = n_lhs; j >= i - n_rhs; j--)
          dataPtr[i] += float(nchoosek(n_lhs, j)) * float(nchoosek(n_rhs, i - j)) /
                        float(nchoosek(n_lhs + n_rhs, i)) * this->bernstein_coeff_[j] *
                        rhs.bernstein_coeff_[i - j];
      }
    }
  } else { // n_lhs<n_rhs
    for (int i = 0; i <= n_lhs + n_rhs; i++) {
      if (i <= n_lhs) {
        for (int j = i; j >= 0; j--)
          dataPtr[i] += float(nchoosek(n_rhs, j)) * float(nchoosek(n_lhs, i - j)) /
                        float(nchoosek(n_lhs + n_rhs, i)) * rhs.bernstein_coeff_[j] *
                        this->bernstein_coeff_[i - j];
      } else if ((i >= n_lhs + 1) and (i <= n_rhs)) {
        for (int j = i; j >= i - n_rhs; j--)
          dataPtr[i] += float(nchoosek(n_rhs, j)) * float(nchoosek(n_lhs, i - j)) /
                        float(nchoosek(n_lhs + n_rhs, i)) * rhs.bernstein_coeff_[j] *
                        this->bernstein_coeff_[i - j];
      } else if (i >= n_rhs + 1) {
        for (int j = n_rhs; j >= i - n_lhs; j--)
          dataPtr[i] += float(nchoosek(n_rhs, j)) * float(nchoosek(n_lhs, i - j)) /
                        float(nchoosek(n_lhs + n_rhs, i)) * rhs.bernstein_coeff_[j] *
                        this->bernstein_coeff_[i - j];
      }
    }
  }
  BernsteinPoly result(this->GetTimeInterval(), dataPtr, n_lhs + n_rhs);
  return result;
}

BernsteinPoly BernsteinPoly::operator*(const float &scalar) {
  int poly_order = degree_;
  BernsteinCoefficients constant_multiplied_coeffs;
  for (int i = 0; i < poly_order + 1; i++)
    constant_multiplied_coeffs.push_back(scalar * this->bernstein_coeff_[i]);
  BernsteinPoly result(this->GetTimeInterval(), constant_multiplied_coeffs, poly_order);
  return result;
}