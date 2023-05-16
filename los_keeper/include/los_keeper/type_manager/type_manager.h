//
// Created by larr-planning on 23. 5. 16.
//
#ifndef LOS_KEEPER_TYPE_MANAGER_H
#define LOS_KEEPER_TYPE_MANAGER_H
#include "los_keeper/bernstein_polynomial_utils/bernstein_polynomial_utils.h"
struct ObjectState{
  float px;
  float py;
  float pz;
  float vx;
  float vy;
  float vz;
  float rx;
  float ry;
  float rz;
};
struct DroneState{
  float px;
  float py;
  float pz;
  float vx;
  float vy;
  float vz;
  float ax;
  float ay;
  float az;
};

struct StatePoly{
  BernsteinPoly px;
  BernsteinPoly py;
  BernsteinPoly pz;
  float rx;
  float ry;
  float rz;
  void SetDegree(const int &degree) {px.SetDegree(degree),py.SetDegree(degree),pz.SetDegree(degree);};
};
struct Point{
  float x;
  float y;
  float z;
};

#endif // LOS_KEEPER_TYPE_MANAGER_H
