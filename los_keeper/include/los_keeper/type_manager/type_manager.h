//
// Created by larr-planning on 23. 5. 16.
//
#ifndef HEADER_TYPE_MANAGER
#define HEADER_TYPE_MANAGER
#include "los_keeper/bernstein_polynomial_utils/bernstein_polynomial_utils.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace los_keeper {

typedef pcl::PointXYZ PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

struct ObjectState {
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
struct DroneState {
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

struct Point {
  float x{0.0};
  float y{0.0};
  float z{0.0};
};

struct StatePoly {
  BernsteinPoly px;
  BernsteinPoly py;
  BernsteinPoly pz;
  float rx;
  float ry;
  float rz;
  void SetDegree(const int &degree) {
    px.SetDegree(degree), py.SetDegree(degree), pz.SetDegree(degree);
  };
  void SetTimeInterval(float time_interval[2]) {
    px.SetTimeInterval(time_interval), py.SetTimeInterval(time_interval),
        pz.SetTimeInterval(time_interval);
  };
  Point GetPointAtTime(double time) const { return Point(); };
};

} // namespace los_keeper
#endif /* HEADER_TYPE_MANAGER */
