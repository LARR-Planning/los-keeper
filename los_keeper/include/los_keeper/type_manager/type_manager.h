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
  void SetTimeInterval(float time_interval[2]){px.SetTimeInterval(time_interval),
        py.SetTimeInterval(time_interval),
        pz.SetTimeInterval(time_interval);};
};
struct Point{
  float x;
  float y;
  float z;
};

#endif // LOS_KEEPER_TYPE_MANAGER_H
