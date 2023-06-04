//
// Created by larr-planning on 23. 5. 16.
//
#ifndef LOS_KEEPER_TYPE_MANAGER_H
#define LOS_KEEPER_TYPE_MANAGER_H
#include "los_keeper/bernstein_polynomial_utils/bernstein_polynomial_utils.h"
#include "thread"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace los_keeper {
typedef pcl::PointXYZ PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

namespace Parameters {
struct VirtualPclBbox {
  float width{1.0f};
  float height{1.0f};
};
struct DynamicLimits {
  float vel_max{0.0f};
  float acc_max{0.0f};
};
struct SamplingParam {
  int num_sample{0};
  int num_thread{0};
  bool is_lite{false};
};
struct DroneParam {
  float rx{0.0f};
  float ry{0.0f};
  float rz{0.0f};
  float fov_angle{1.5707f};
};
struct DistanceParam {
  float distance_min{0.0f};
  float distance_max{0.0f};
};

struct HorizonParam {
  float planning_horizon{0.0f};
};

struct PredictionParam {
  struct SamplingParam sample_param;
  struct HorizonParam horizon_param;
  struct DynamicLimits dynamic_param;
  struct DistanceParam detect_param;
  struct VirtualPclBbox sfc_param;
};
struct PlanningParam {
  struct SamplingParam sample_param;
  struct HorizonParam horizon_param;
  struct DistanceParam distance_obstacle_param;
  struct DistanceParam distance_target_param;
  struct DynamicLimits dynamic_param;
  struct DroneParam drone_param;
  struct VirtualPclBbox sfc_param;
};
struct ObstacleParam {
  HorizonParam horizon_param;
};
struct ProblemParam {
  bool is_2d{false};
};
} // namespace Parameters
} // namespace los_keeper
struct ObjectState {
  double t_sec{0.0};
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
  double t_sec{0.0};
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
