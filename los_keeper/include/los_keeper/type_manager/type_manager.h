//
// Created by larr-planning on 23. 5. 16.
//
#ifndef LOS_KEEPER_TYPE_MANAGER_H
#define LOS_KEEPER_TYPE_MANAGER_H
#include "los_keeper/bernstein_polynomial_utils/bernstein_polynomial_utils.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "thread"
namespace los_keeper {
typedef pcl::PointXYZ PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;
}
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

using namespace std;
typedef vector<int> IndexList;
typedef vector<IndexList> IndexListSet;
typedef vector<Point> PointList;
typedef vector<PointList> PointListSet;
typedef vector<StatePoly> PrimitiveList;
typedef vector<PrimitiveList> PrimitiveListSet;
#endif // LOS_KEEPER_TYPE_MANAGER_H
